"""Wrist-camera and keyboard publishers for the YAM bridge.

``WristCameraStreamer`` renders the simulated Logitech webcam (``yam_sim.camera.WristCamera``) or
reads a real one (``yam_sim.camera.RealWebcam``, needs OpenCV) in its own thread and publishes

    /yam_wrist_camera/image_raw             sensor_msgs/Image            rgb8
    /yam_wrist_camera/image_raw/compressed  sensor_msgs/CompressedImage  jpeg (Pillow)
    /yam_wrist_camera/camera_info           sensor_msgs/CameraInfo       same stamp as the image

with ``frame_id = <prefix>wrist_camera_optical_frame``. Rendering runs in a dedicated thread
(not an executor callback) so a 10-20 ms render never delays the bridge's 200 Hz command timer.
``WristCamera`` copies the robot state under ``robot.lock`` and renders from the copy, so the
physics thread is blocked only for that copy.

``KeyboardPublisher`` (sim only, ``objects`` contains ``keyboard``) publishes

    /yam_keyboard/pressed_keys   std_msgs/String   space-separated keys currently down (every tick)
    /yam_keyboard/typed          std_msgs/String   one message per new key press (the key name)
    /yam_keyboard/markers        visualization_msgs/MarkerArray (latched), one CUBE per key in
                                 namespace "keys" (+ "base" plate and "labels" text), pressed keys
                                 drawn orange; republished when the pressed set changes
    /tf_static                   <prefix>base -> <prefix>keyboard (keyboard body pose in the sim)
"""

from __future__ import annotations

import array
import io
import threading
import time
from typing import List, Optional, Tuple

import numpy as np
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

IMAGE_TOPIC = "/yam_wrist_camera/image_raw"
COMPRESSED_TOPIC = "/yam_wrist_camera/image_raw/compressed"
INFO_TOPIC = "/yam_wrist_camera/camera_info"
PRESSED_TOPIC = "/yam_keyboard/pressed_keys"
TYPED_TOPIC = "/yam_keyboard/typed"
MARKER_TOPIC = "/yam_keyboard/markers"


def mat_to_quat_xyzw(R: np.ndarray) -> Tuple[float, float, float, float]:
    """Rotation matrix -> (x, y, z, w), w >= 0."""
    m = np.asarray(R, dtype=float)
    t = np.trace(m)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        w, x, y, z = 0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        w, x, y, z = (m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        w, x, y, z = (m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s
    else:
        s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        w, x, y, z = (m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s
    q = np.array([x, y, z, w])
    q /= np.linalg.norm(q)
    if q[3] < 0:
        q = -q
    return tuple(float(v) for v in q)


def parse_resolution(text: str, default: Tuple[int, int]) -> Tuple[int, int]:
    """``"1280x720"`` -> (1280, 720); ``""`` -> ``default``."""
    text = str(text or "").strip().lower()
    if not text:
        return tuple(default)
    try:
        w, h = (int(v) for v in text.replace("*", "x").split("x"))
    except ValueError as e:
        raise ValueError(f"camera_resolution must look like 1280x720, got {text!r}") from e
    if w <= 0 or h <= 0:
        raise ValueError(f"bad camera_resolution {text!r}")
    return w, h


class WristCameraStreamer:
    """Publishes the wrist webcam image + camera_info at a fixed rate from a worker thread."""

    def __init__(self, node, robot, backend: str, camera: str, rate: float, resolution: str,
                 prefix: str, device: str = "0", jpeg_quality: int = 85):
        from yam_sim.camera import camera_spec

        self.node = node
        self.log = node.get_logger()
        self.robot = robot
        self.backend = backend
        self.spec = camera_spec(camera)
        self.rate = float(rate)
        if self.rate <= 0:
            raise ValueError("camera_rate must be > 0")
        self.width, self.height = parse_resolution(resolution, self.spec.default_resolution)
        self.frame_id = f"{prefix}wrist_camera_optical_frame"
        self.device = device
        self.jpeg_quality = int(jpeg_quality)

        self.image_pub = node.create_publisher(Image, IMAGE_TOPIC, 2)
        self.compressed_pub = node.create_publisher(CompressedImage, COMPRESSED_TOPIC, 2)
        self.info_pub = node.create_publisher(CameraInfo, INFO_TOPIC, 2)

        self._stop = threading.Event()
        self._ready = threading.Event()
        self._error: Optional[BaseException] = None
        self.source = None
        # timing statistics (ms), reset at every report
        self._stats_lock = threading.Lock()
        self._stats = {"render": [], "encode": [], "frames": 0, "late": 0}
        self.thread = threading.Thread(target=self._run, name="wrist_camera", daemon=True)
        self.thread.start()
        self._ready.wait(timeout=30.0)
        if self._error is not None:
            raise self._error

    # ------------------------------------------------------------------ source
    def _open_source(self):
        if self.backend == "sim":
            from yam_sim.camera import WristCamera

            # The GL context is created here, in the thread that renders with it.
            return WristCamera(self.robot, spec=self.spec, width=self.width, height=self.height)
        try:
            import cv2  # noqa: F401
        except ImportError:
            self.log.warn(
                "camera: backend:=real needs OpenCV for yam_sim.camera.RealWebcam, which is not installed "
                "in this environment (pip install opencv-python-headless). Not publishing wrist-camera topics."
            )
            return None
        from yam_sim.camera import RealWebcam

        idx = int(self.device) if str(self.device).isdigit() else self.device
        cam = RealWebcam(idx, spec=self.spec, width=self.width, height=self.height)
        self.width, self.height = cam.width, cam.height
        return cam

    def camera_info_msg(self, stamp) -> CameraInfo:
        K = self.spec.K(self.width, self.height)
        msg = CameraInfo()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.width, msg.height = int(self.width), int(self.height)
        msg.distortion_model = "plumb_bob"
        msg.d = [float(v) for v in self.spec.distortion]
        msg.k = [float(v) for v in K.reshape(-1)]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.p = [float(v) for v in np.hstack([K, np.zeros((3, 1))]).reshape(-1)]
        return msg

    # ------------------------------------------------------------------ worker
    def _run(self) -> None:
        try:
            self.source = self._open_source()
        except BaseException as e:  # surfaced to the constructor
            self._error = e
            self._ready.set()
            return
        self._ready.set()
        if self.source is None:
            return
        self.log.info(
            f"wrist camera: {self.spec.name} ({self.backend}) {self.width}x{self.height} at {self.rate:g} Hz, "
            f"frame {self.frame_id}"
        )
        period = 1.0 / self.rate
        next_t = time.monotonic()
        while not self._stop.is_set():
            try:
                self._publish_frame()
            except Exception as e:  # keep the bridge alive; report once in a while
                self.log.error(f"wrist camera frame failed: {e}", throttle_duration_sec=5.0)
            next_t += period
            delay = next_t - time.monotonic()
            if delay < 0:  # fell behind: do not try to catch up with a burst
                with self._stats_lock:
                    self._stats["late"] += 1
                next_t = time.monotonic()
                delay = 0.0
            self._stop.wait(delay)
        try:
            self.source.close()
        except Exception:
            pass

    def _publish_frame(self) -> None:
        want_raw = self.image_pub.get_subscription_count() > 0
        want_jpeg = self.compressed_pub.get_subscription_count() > 0
        stamp = self.node.get_clock().now().to_msg()
        render_ms = encode_ms = None
        if want_raw or want_jpeg:
            t0 = time.perf_counter()
            rgb = self.source.render()
            render_ms = (time.perf_counter() - t0) * 1e3
            h, w = rgb.shape[:2]
            if want_raw:
                img = Image()
                img.header.stamp = stamp
                img.header.frame_id = self.frame_id
                img.height, img.width = int(h), int(w)
                img.encoding = "rgb8"
                img.is_bigendian = 0
                img.step = int(w) * 3
                # array('B', bytes) is a memcpy; assigning bytes/ndarray directly is slow in rclpy
                img.data = array.array("B", np.ascontiguousarray(rgb).tobytes())
                self.image_pub.publish(img)
            if want_jpeg:
                t1 = time.perf_counter()
                from PIL import Image as PILImage

                buf = io.BytesIO()
                PILImage.fromarray(np.ascontiguousarray(rgb)).save(buf, format="JPEG", quality=self.jpeg_quality)
                c = CompressedImage()
                c.header.stamp = stamp
                c.header.frame_id = self.frame_id
                c.format = "rgb8; jpeg compressed bgr8"  # what image_transport's compressed plugin writes
                c.data = array.array("B", buf.getvalue())
                self.compressed_pub.publish(c)
                encode_ms = (time.perf_counter() - t1) * 1e3
        self.info_pub.publish(self.camera_info_msg(stamp))
        with self._stats_lock:
            self._stats["frames"] += 1
            if render_ms is not None:
                self._stats["render"].append(render_ms)
            if encode_ms is not None:
                self._stats["encode"].append(encode_ms)

    def take_stats(self) -> dict:
        with self._stats_lock:
            s = self._stats
            self._stats = {"render": [], "encode": [], "frames": 0, "late": 0}
        return s

    def close(self) -> None:
        self._stop.set()
        if self.thread.is_alive():
            self.thread.join(timeout=5.0)


class KeyboardPublisher:
    """Pressed keys, typed events, keyboard TF and RViz markers for the simulated keyboard."""

    def __init__(self, node, robot, prefix: str, rate: float = 50.0, publish_markers: bool = True,
                 callback_group=None):
        import mujoco
        import tf2_ros
        from yam_sim.keyboard import Keyboard

        self.node = node
        self.robot = robot
        self.prefix = prefix
        model = robot.model
        self.kb = Keyboard.from_model(model, robot.data)
        self.base_frame = f"{prefix}base"
        self.kb_frame = f"{prefix}keyboard"

        # Keyboard pose in the arm base frame (the keyboard body is static).
        d = mujoco.MjData(model)
        mujoco.mj_kinematics(model, d)
        bid = model.body("base").id
        T_wb = np.eye(4)
        T_wb[:3, :3] = d.xmat[bid].reshape(3, 3)
        T_wb[:3, 3] = d.xpos[bid]
        self.T_world_base = T_wb
        self.T_base_world = np.linalg.inv(T_wb)
        kid = model.body(Keyboard.BODY).id
        T_wk = np.eye(4)
        T_wk[:3, :3] = d.xmat[kid].reshape(3, 3)
        T_wk[:3, 3] = d.xpos[kid]
        self.T_base_kb = self.T_base_world @ T_wk
        # nominal (unpressed) key boxes in the base frame, from the same model
        self._key_data = d

        self.static_tf = tf2_ros.StaticTransformBroadcaster(node)
        t = TransformStamped()
        t.header.stamp = node.get_clock().now().to_msg()
        t.header.frame_id = self.base_frame
        t.child_frame_id = self.kb_frame
        p = self.T_base_kb[:3, 3]
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = (float(v) for v in p)
        qx, qy, qz, qw = mat_to_quat_xyzw(self.T_base_kb[:3, :3])
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = qx, qy, qz, qw
        self.static_tf.sendTransform(t)

        self.pressed_pub = node.create_publisher(String, PRESSED_TOPIC, 10)
        self.typed_pub = node.create_publisher(String, TYPED_TOPIC, 50)
        self.publish_markers = bool(publish_markers)
        if self.publish_markers:
            latched = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
            self.marker_pub = node.create_publisher(MarkerArray, MARKER_TOPIC, latched)
            self._publish_markers(set())
        self._prev: List[str] = []
        self.timer = node.create_timer(1.0 / float(rate), self._tick, callback_group=callback_group)
        node.get_logger().info(
            f"keyboard: {self.kb.layout} layout, {len(self.kb.key_names)} keys, {self.kb_frame} at "
            f"{np.round(p, 4).tolist()} in {self.base_frame}"
        )

    def _tick(self) -> None:
        with self.robot.lock:
            pressed = self.kb.pressed_keys(self.robot.data)
        self.pressed_pub.publish(String(data=" ".join(pressed)))
        prev = set(self._prev)
        for k in pressed:
            if k not in prev:
                self.typed_pub.publish(String(data=k))
        if self.publish_markers and pressed != self._prev:
            self._publish_markers(set(pressed))
        self._prev = pressed

    def _publish_markers(self, pressed: set) -> None:
        from yam_sim.keyboard import CAP_HALF_HEIGHT

        stamp = self.node.get_clock().now().to_msg()
        R_bk = self.T_base_kb[:3, :3]
        q = mat_to_quat_xyzw(R_bk)
        arr = MarkerArray()

        def marker(ns, mid, mtype, pos, scale, rgba, quat=q):
            m = Marker()
            m.header.stamp = stamp
            m.header.frame_id = self.base_frame
            m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(v) for v in pos)
            m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = quat
            m.scale.x, m.scale.y, m.scale.z = (float(v) for v in scale)
            m.color.r, m.color.g, m.color.b, m.color.a = (float(v) for v in rgba)
            m.frame_locked = True
            return m

        # base plate
        size = self.kb.size
        base_c = self.T_base_kb[:3, :3] @ np.array([0.0, 0.0, size[2] / 2]) + self.T_base_kb[:3, 3]
        arr.markers.append(marker("base", 0, Marker.CUBE, base_c, size, (0.16, 0.16, 0.17, 1.0)))
        for i, k in enumerate(self.kb.key_names):
            # key_pose (unbound -> live data of our static copy) is the top-face centre in world
            T_top = self.T_base_world @ self.kb.key_pose(k, self._key_data)
            centre = T_top[:3, 3] - T_top[:3, 2] * CAP_HALF_HEIGHT
            hs = self.kb.key_half_size(k)
            down = k in pressed
            rgba = (1.0, 0.55, 0.1, 1.0) if down else (0.70, 0.70, 0.68, 1.0)
            if down:
                centre = centre - T_top[:3, 2] * 0.004
            arr.markers.append(marker("keys", i, Marker.CUBE, centre, 2 * hs, rgba))
            label = marker("labels", i, Marker.TEXT_VIEW_FACING, T_top[:3, 3] + T_top[:3, 2] * 0.004,
                           (0.0, 0.0, 0.007), (0.05, 0.05, 0.05, 1.0), quat=(0.0, 0.0, 0.0, 1.0))
            label.text = k
            arr.markers.append(label)
        self.marker_pub.publish(arr)
