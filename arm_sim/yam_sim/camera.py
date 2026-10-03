"""Wrist webcam: the camera model mounted on the gripper, a sim renderer, and a real-webcam reader.

The camera is a Logitech webcam (C920 by default, or C270) clamped on top of the gripper housing.
Everything about it (mount pose relative to the ``gripper`` body, field of view, resolutions,
intrinsics, mass and rough dimensions) lives in one YAML file per model,
``yam_sim/models/camera/logitech_<model>.yml``, so the ROS side can read the same numbers.

Frames:

* ``gripper`` body (the tool flange): +Z approach, +X right, +Y down. The camera sits on the -Y face.
* ``wrist_camera`` body = the **optical frame** (ROS convention: +Z forward along the optical
  axis, +X right in the image, +Y down in the image). The site ``wrist_cam_optical`` marks it.
* The MuJoCo ``<camera name="wrist_cam">`` looks along its own -Z with +Y up, so it is the optical
  frame turned 180 degrees about X.

Usage::

    from yam_sim.robot import YamSimRobot
    from yam_sim.camera import WristCamera
    robot = YamSimRobot(objects="keyboard", camera="c920")
    cam = WristCamera(robot)                 # 1280x720 by default
    rgb = cam.render()                       # (720, 1280, 3) uint8
    uv = cam.project(robot.get_ee_pose()[:3, 3])

    python -m yam_sim.camera --out wrist.png                 # sim snapshot
    pixi run -e webcam python -m yam_sim.camera --webcam 0   # grab a frame from a real USB webcam
"""

from __future__ import annotations

import argparse
import os
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Tuple, Union

import mujoco
import numpy as np
import yaml

CAMERA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "models", "camera")
CAMERA_FILES = {"c920": "logitech_c920.yml", "c270": "logitech_c270.yml"}
CAMERA_BODY = "wrist_camera"
CAMERA_NAME = "wrist_cam"
OPTICAL_SITE = "wrist_cam_optical"
# Rotation from the optical frame (+Z forward, +Y down) to a MuJoCo camera frame (-Z forward, +Y up).
_OPT_TO_MJCAM_QUAT = np.array([0.0, 1.0, 0.0, 0.0])
_OPT_TO_MJCAM_R = np.diag([1.0, -1.0, -1.0])


def _fmt(values: Sequence[float]) -> str:
    return " ".join(f"{float(v):.8g}" for v in values)


@dataclass(frozen=True)
class CameraSpec:
    """One webcam model, as loaded from ``models/camera/logitech_<key>.yml``."""

    key: str
    name: str
    frame_name: str
    mount_parent: str
    mount_pos: np.ndarray  # optical frame origin in the parent (gripper) frame, m
    mount_quat: np.ndarray  # optical frame orientation in the parent frame, wxyz
    tilt_deg: float
    hfov_deg: float
    vfov_deg: float
    diagonal_fov_deg: float
    default_resolution: Tuple[int, int]
    modes: Tuple[Dict[str, float], ...]
    body: Dict[str, Any] = field(default_factory=dict)
    distortion: Tuple[float, ...] = (0.0, 0.0, 0.0, 0.0, 0.0)
    path: str = ""

    @property
    def fovy(self) -> float:
        """Vertical field of view in degrees (MuJoCo ``fovy``)."""
        return self.vfov_deg

    @property
    def mass(self) -> float:
        return float(self.body.get("mass", 0.0))

    def intrinsics(self, width: Optional[int] = None, height: Optional[int] = None) -> Dict[str, float]:
        """Pinhole intrinsics for an image size (default: the default resolution).

        Computed from ``fovy``: fx = fy = (H / 2) / tan(fovy / 2), cx = (W - 1) / 2, cy = (H - 1) / 2
        (OpenCV/ROS convention: pixel centres at integer coordinates). This is exactly what MuJoCo
        renders: square pixels, principal point at the image centre, no distortion.
        """
        w, h = (int(width), int(height)) if width else self.default_resolution
        f = (h / 2.0) / np.tan(np.deg2rad(self.vfov_deg) / 2.0)
        return {"width": w, "height": h, "fx": float(f), "fy": float(f), "cx": (w - 1) / 2.0, "cy": (h - 1) / 2.0}

    def K(self, width: Optional[int] = None, height: Optional[int] = None) -> np.ndarray:
        i = self.intrinsics(width, height)
        return np.array([[i["fx"], 0.0, i["cx"]], [0.0, i["fy"], i["cy"]], [0.0, 0.0, 1.0]])


def camera_spec(name: Union[str, CameraSpec, None]) -> Optional[CameraSpec]:
    """Load a webcam spec: ``"c920"`` (default model), ``"c270"``, a YAML path, or ``"none"`` -> None."""
    if isinstance(name, CameraSpec):
        return name
    if name is None or str(name).lower() in ("none", "", "false"):
        return None
    key = str(name).lower().replace("logitech_", "")
    path = os.path.join(CAMERA_DIR, CAMERA_FILES[key]) if key in CAMERA_FILES else str(name)
    if not os.path.exists(path):
        raise ValueError(f"unknown camera {name!r}; use one of {sorted(CAMERA_FILES)} or 'none', or a YAML path")
    with open(path) as f:
        y = yaml.safe_load(f)
    opt = y["optics"]
    return CameraSpec(
        key=y.get("key", key),
        name=y["name"],
        frame_name=y.get("frame_name", "wrist_camera_optical_frame"),
        mount_parent=y["mount"].get("parent", "gripper"),
        mount_pos=np.array(y["mount"]["pos"], dtype=float),
        mount_quat=np.array(y["mount"]["quat_wxyz"], dtype=float) / np.linalg.norm(y["mount"]["quat_wxyz"]),
        tilt_deg=float(y["mount"].get("tilt_deg", 0.0)),
        hfov_deg=float(opt["hfov_deg"]),
        vfov_deg=float(opt["vfov_deg"]),
        diagonal_fov_deg=float(opt["diagonal_fov_deg"]),
        default_resolution=tuple(int(v) for v in opt["default_resolution"]),
        modes=tuple(dict(m) for m in y.get("modes", [])),
        body=dict(y.get("body", {})),
        distortion=tuple(float(v) for v in opt.get("distortion", [0, 0, 0, 0, 0])),
        path=path,
    )


def camera_body_mjcf(spec: CameraSpec) -> ET.Element:
    """The ``wrist_camera`` body (to be placed inside the gripper mount body)."""
    b = spec.body
    sx, sy, sz = (float(v) / 2.0 for v in b.get("size", [0.09, 0.03, 0.025]))
    off = np.array(b.get("center_offset", [0.0, 0.0, -sz]), dtype=float)
    lens_r = float(b.get("lens_radius", 0.008))
    lens_l = float(b.get("lens_length", 0.004))
    rgba = _fmt(b.get("rgba", [0.06, 0.06, 0.07, 1]))
    lens_rgba = _fmt(b.get("lens_rgba", [0.02, 0.02, 0.03, 1]))
    body = ET.Element("body", {"name": CAMERA_BODY, "pos": _fmt(spec.mount_pos), "quat": _fmt(spec.mount_quat)})
    vis = {"contype": "0", "conaffinity": "0", "group": "2"}
    ET.SubElement(body, "geom", {"name": "wrist_camera_body", "type": "box", "size": _fmt([sx, sy, sz]), "pos": _fmt(off),
                                 "rgba": rgba, "mass": f"{spec.mass:g}", **vis})
    # The lens ends 1 mm behind the optical centre so it never shows up in its own image.
    ET.SubElement(body, "geom", {"name": "wrist_camera_lens", "type": "cylinder", "size": _fmt([lens_r, lens_l / 2]),
                                 "pos": _fmt([0, 0, -0.001 - lens_l / 2]), "rgba": lens_rgba, "mass": "0", **vis})
    # Bracket from the bottom of the camera head down to the gripper housing (the -Y face is at
    # y = -0.0325 in the gripper frame). Expressed in the optical frame.
    R = np.empty(9)
    mujoco.mju_quat2Mat(R, spec.mount_quat)
    R = R.reshape(3, 3)
    head_c = spec.mount_pos + R @ off
    top_y = -0.0325
    low_y = head_c[1] + abs(R[1, 1]) * sy + abs(R[1, 2]) * sz  # lowest point (largest y) of the head box
    if low_y < top_y - 1e-4:
        c = np.array([0.0, (low_y + top_y) / 2, head_c[2]])
        h = (top_y - low_y) / 2 + 0.001
        local = R.T @ (c - spec.mount_pos)
        q = np.empty(4)
        mujoco.mju_negQuat(q, spec.mount_quat)
        ET.SubElement(body, "geom", {"name": "wrist_camera_bracket", "type": "box", "size": _fmt([0.012, h, 0.009]),
                                     "pos": _fmt(local), "quat": _fmt(q), "rgba": "0.15 0.15 0.16 1", "mass": "0", **vis})
    w, h = spec.default_resolution
    ET.SubElement(body, "camera", {"name": CAMERA_NAME, "mode": "fixed", "pos": "0 0 0", "quat": _fmt(_OPT_TO_MJCAM_QUAT),
                                   "fovy": f"{spec.vfov_deg:.6g}", "resolution": f"{w} {h}"})
    ET.SubElement(body, "site", {"name": OPTICAL_SITE, "pos": "0 0 0", "size": "0.004", "rgba": "0.2 0.6 1 1", "group": "4"})
    return body


# ================================================================================================
# Simulated wrist camera
# ================================================================================================
class WristCamera:
    """Render the wrist camera of a simulated YAM arm.

    Args:
        source: a :class:`~yam_sim.robot.YamSimRobot` (its model/data are used; the state is copied
            under the robot's lock, so it is safe while the physics thread runs), or an
            ``MjModel``. With an ``MjModel``, pass ``data`` (rendered as is: call ``mj_forward``
            or ``mj_kinematics`` + ``mj_camlight`` yourself after changing qpos).
        spec: camera model name or :class:`CameraSpec`. Default: read from the model
            (``custom/text wrist_camera_model``), or ``"c920"``.
        width, height: image size. Default: the spec's default resolution (1280x720). Must not
            exceed the model's offscreen buffer (``<visual><global offwidth offheight>``).
        scene_option: ``MjvOption``. The default hides the IK target (group 1), the collision
            pads (group 3) and all sites.
    """

    def __init__(
        self,
        source: Any,
        spec: Union[str, CameraSpec, None] = None,
        width: Optional[int] = None,
        height: Optional[int] = None,
        data: Optional[mujoco.MjData] = None,
        scene_option: Optional[mujoco.MjvOption] = None,
        camera_name: str = CAMERA_NAME,
    ):
        if isinstance(source, mujoco.MjModel):
            self.robot = None
            self.model = source
            self._own_data = data is None
            self.data = data if data is not None else mujoco.MjData(source)
            if self._own_data:
                mujoco.mj_forward(self.model, self.data)
        else:
            self.robot = source
            self.model = source.model
            self._own_data = True
            self.data = mujoco.MjData(self.model)
        m = self.model
        self.cam_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
        if self.cam_id < 0:
            raise ValueError(f"model has no camera {camera_name!r}; build the scene with camera='c920' or 'c270'")
        if spec is None:
            spec = model_camera_key(m) or "c920"
        self.spec = camera_spec(spec)
        w, h = (width, height) if width else self.spec.default_resolution
        self.width, self.height = int(w), int(h)
        if self.width > m.vis.global_.offwidth or self.height > m.vis.global_.offheight:
            raise ValueError(
                f"{self.width}x{self.height} exceeds the offscreen buffer {m.vis.global_.offwidth}x{m.vis.global_.offheight}; "
                "raise <visual><global offwidth/offheight> in the scene"
            )
        self.renderer = mujoco.Renderer(m, height=self.height, width=self.width)
        if scene_option is None:
            scene_option = mujoco.MjvOption()
            scene_option.geomgroup[:] = 0
            scene_option.geomgroup[0] = scene_option.geomgroup[2] = 1
            scene_option.sitegroup[:] = 0
        self.scene_option = scene_option
        self._intr = self.spec.intrinsics(self.width, self.height)

    # ------------------------------------------------------------------ state
    def sync(self) -> None:
        """Copy the robot's state (if any) and recompute positions. Called by every render."""
        if self.robot is None:
            return
        with self.robot.lock:
            src = self.robot.data
            self.data.qpos[:] = src.qpos
            self.data.qvel[:] = src.qvel
            self.data.mocap_pos[:] = src.mocap_pos
            self.data.mocap_quat[:] = src.mocap_quat
            self.data.time = src.time
        mujoco.mj_kinematics(self.model, self.data)
        mujoco.mj_comPos(self.model, self.data)
        mujoco.mj_camlight(self.model, self.data)

    def _render(self, mode: str) -> np.ndarray:
        self.sync()
        r = self.renderer
        if mode == "depth":
            r.enable_depth_rendering()
        elif mode == "seg":
            r.enable_segmentation_rendering()
        try:
            r.update_scene(self.data, camera=self.cam_id, scene_option=self.scene_option)
            return r.render()
        finally:
            r.disable_depth_rendering()
            r.disable_segmentation_rendering()

    def render(self) -> np.ndarray:
        """RGB image, ``(H, W, 3) uint8``."""
        return self._render("rgb").copy()

    def depth(self) -> np.ndarray:
        """Depth along the optical axis in metres, ``(H, W) float32``."""
        return self._render("depth").copy()

    def segmentation(self, kind: str = "geom") -> np.ndarray:
        """Per-pixel ids, ``(H, W) int32``, -1 for background.

        ``kind="geom"`` gives MuJoCo geom ids, ``kind="body"`` the id of the body owning the geom.
        """
        seg = self._render("seg")
        ids = seg[:, :, 0].astype(np.int32)
        geom = (seg[:, :, 1] == int(mujoco.mjtObj.mjOBJ_GEOM)) & (ids >= 0)
        out = np.where(geom, ids, -1).astype(np.int32)
        if kind == "body":
            lut = np.append(self.model.geom_bodyid, -1).astype(np.int32)
            out = lut[out]  # -1 indexes the appended -1
        elif kind != "geom":
            raise ValueError("kind must be 'geom' or 'body'")
        return out

    # ------------------------------------------------------------------ geometry
    def intrinsics(self) -> Dict[str, float]:
        """``{"fx", "fy", "cx", "cy", "width", "height"}`` for the rendered image size."""
        return dict(self._intr)

    def K(self) -> np.ndarray:
        i = self._intr
        return np.array([[i["fx"], 0.0, i["cx"]], [0.0, i["fy"], i["cy"]], [0.0, 0.0, 1.0]])

    def pose(self, synced: bool = False) -> np.ndarray:
        """4x4 optical-frame-to-world transform (ROS optical frame: +Z forward, +Y down)."""
        if not synced:
            self.sync()
        T = np.eye(4)
        T[:3, :3] = self.data.cam_xmat[self.cam_id].reshape(3, 3) @ _OPT_TO_MJCAM_R
        T[:3, 3] = self.data.cam_xpos[self.cam_id]
        return T

    def project(self, points_world: np.ndarray, return_depth: bool = False, synced: bool = False):
        """Project world points to pixel coordinates ``(N, 2)`` (u right, v down; OpenCV convention).

        Points behind the camera get NaN. With ``return_depth`` also returns the optical-axis
        depth of each point.
        """
        P = np.atleast_2d(np.asarray(points_world, dtype=float))
        T = self.pose(synced=synced)
        pc = (P - T[:3, 3]) @ T[:3, :3]
        z = pc[:, 2]
        i = self._intr
        with np.errstate(divide="ignore", invalid="ignore"):
            u = i["fx"] * pc[:, 0] / z + i["cx"]
            v = i["fy"] * pc[:, 1] / z + i["cy"]
        uv = np.stack([u, v], axis=1)
        uv[z <= 1e-6] = np.nan
        if np.asarray(points_world).ndim == 1:
            uv = uv[0]
            z = z[0]
        return (uv, z) if return_depth else uv

    def camera_info(self) -> Dict[str, Any]:
        """ROS ``sensor_msgs/CameraInfo``-style dict."""
        K = self.K()
        P = np.hstack([K, np.zeros((3, 1))])
        return {
            "frame_id": self.spec.frame_name,
            "width": self.width,
            "height": self.height,
            "distortion_model": "plumb_bob",
            "D": list(self.spec.distortion),
            "K": K.reshape(-1).tolist(),
            "R": np.eye(3).reshape(-1).tolist(),
            "P": P.reshape(-1).tolist(),
            "camera_model": self.spec.name,
        }

    def close(self) -> None:
        if getattr(self, "renderer", None) is not None:
            self.renderer.close()
            self.renderer = None

    def __enter__(self) -> "WristCamera":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def __del__(self):  # pragma: no cover - best effort
        try:
            self.close()
        except Exception:
            pass


def model_camera_key(model: mujoco.MjModel) -> Optional[str]:
    """The camera model a scene was built with (stored as custom text), or None."""
    tid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TEXT, "wrist_camera_model")
    if tid < 0:
        return None
    adr, n = model.text_adr[tid], model.text_size[tid]
    return bytes(model.text_data[adr: adr + n - 1]).decode()


# ================================================================================================
# Real USB webcam
# ================================================================================================
OPENCV_HINT = (
    "RealWebcam needs OpenCV, which is not in the default pixi environment. Use one of:\n"
    "    pixi run -e webcam python ...      # opencv-python-headless only\n"
    "    pixi run -e train  python ...      # Ultralytics + PyTorch + OpenCV\n"
    "or `pip install opencv-python-headless` in your own environment."
)


class RealWebcam:
    """A real USB webcam with the same ``render()`` signature as :class:`WristCamera`.

    ``spec`` selects the nominal intrinsics (from the same YAML as the sim). Calibrate the real
    camera for anything metric; the nominal numbers ignore lens distortion and autofocus.
    """

    def __init__(self, index: Union[int, str] = 0, spec: Union[str, CameraSpec] = "c920",
                 width: Optional[int] = None, height: Optional[int] = None):
        try:
            import cv2  # noqa: F401
        except ImportError as e:
            raise ImportError(OPENCV_HINT) from e
        import cv2

        self._cv2 = cv2
        self.spec = camera_spec(spec)
        w, h = (width, height) if width else self.spec.default_resolution
        self.cap = cv2.VideoCapture(index)
        if not self.cap.isOpened():
            raise RuntimeError(f"could not open webcam {index!r} (on macOS, grant camera access to your terminal)")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(w))
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(h))
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or int(w)
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or int(h)

    def render(self) -> np.ndarray:
        """Grab one frame as RGB ``(H, W, 3) uint8``."""
        ok, frame = self.cap.read()
        if not ok or frame is None:
            raise RuntimeError("webcam returned no frame")
        return self._cv2.cvtColor(frame, self._cv2.COLOR_BGR2RGB)

    def intrinsics(self) -> Dict[str, float]:
        return self.spec.intrinsics(self.width, self.height)

    def camera_info(self) -> Dict[str, Any]:
        K = self.spec.K(self.width, self.height)
        return {"frame_id": self.spec.frame_name, "width": self.width, "height": self.height,
                "distortion_model": "plumb_bob", "D": list(self.spec.distortion), "K": K.reshape(-1).tolist(),
                "camera_model": self.spec.name}

    def close(self) -> None:
        if self.cap is not None:
            self.cap.release()
            self.cap = None


def ee_mass_properties(model: mujoco.MjModel, mount: str = "gripper") -> Dict[str, Any]:
    """Combined inertial of the gripper mount body plus everything welded to it (webcam, stylus).

    Returns ``ee_mass`` (kg) and ``ee_inertia`` (10 values: COM position (3), principal-axes quat
    wxyz (4), principal moments (3), all in the mount frame) in the format of i2rt's
    ``get_yam_robot(..., ee_mass=, ee_inertia=)``, which overrides the mount body's inertial in the
    model the real driver uses for gravity compensation. The finger bodies are not included (they
    stay separate bodies in i2rt's model too).
    """
    mid = model.body(mount).id
    bodies = [mid] + [b for b in range(model.nbody) if model.body_parentid[b] == mid and model.body_jntnum[b] == 0]
    d = mujoco.MjData(model)
    mujoco.mj_kinematics(model, d)
    Rm, pm = d.xmat[mid].reshape(3, 3), d.xpos[mid]
    masses, coms, inertias = [], [], []
    for b in bodies:
        m = float(model.body_mass[b])
        if m <= 0:
            continue
        Rb = d.ximat[b].reshape(3, 3)
        com = Rm.T @ (d.xipos[b] - pm)
        Rl = Rm.T @ Rb
        I = Rl @ np.diag(model.body_inertia[b]) @ Rl.T
        masses.append(m)
        coms.append(com)
        inertias.append(I)
    M = sum(masses)
    c = sum(m * x for m, x in zip(masses, coms)) / M
    I = np.zeros((3, 3))
    for m, x, Ib in zip(masses, coms, inertias):
        r = x - c
        I += Ib + m * (np.dot(r, r) * np.eye(3) - np.outer(r, r))
    w, V = np.linalg.eigh(I)
    if np.linalg.det(V) < 0:
        V[:, 0] *= -1
    q = np.empty(4)
    mujoco.mju_mat2Quat(q, V.reshape(-1))
    return {"ee_mass": float(M), "ee_inertia": np.concatenate([c, q, w]), "bodies": [model.body(b).name for b in bodies]}


def save_image(path: str, rgb: np.ndarray, quality: int = 92) -> None:
    """Write an RGB uint8 image (PNG/JPEG by extension) with Pillow."""
    from PIL import Image

    img = Image.fromarray(np.ascontiguousarray(rgb))
    if path.lower().endswith((".jpg", ".jpeg")):
        img.save(path, quality=quality)
    else:
        img.save(path, optimize=True)


def main(argv: Optional[List[str]] = None) -> int:
    p = argparse.ArgumentParser(description="Grab one wrist-camera frame (sim, or a real webcam with --webcam).")
    p.add_argument("--camera", default="c920", help="c920 | c270")
    p.add_argument("--webcam", default=None, help="real webcam index (needs OpenCV: pixi run -e webcam ...)")
    p.add_argument("--width", type=int, default=None)
    p.add_argument("--height", type=int, default=None)
    p.add_argument("--q", type=float, nargs=6, default=[0.0, 1.2, 1.0, -0.9, 0.0, 0.0], help="arm joints for the sim shot")
    p.add_argument("--out", default="wrist_camera.png")
    p.add_argument("--tool", default="stylus", help="tool in the sim shot / mass report: stylus | none")
    p.add_argument("--ee-mass", action="store_true",
                   help="print the ee_mass / ee_inertia to pass to i2rt get_yam_robot for this camera (+tool) and exit")
    args = p.parse_args(argv)
    tool = None if args.tool == "none" else args.tool
    if args.ee_mass:
        from yam_sim.assembly import load_scene

        model, _ = load_scene(camera=args.camera, tool=tool)
        props = ee_mass_properties(model)
        base, _ = load_scene(camera="none")
        print(f"gripper mount alone: {base.body_mass[base.body('gripper').id]:.4f} kg; with {args.camera} + {tool}: "
              f"{props['bodies']}")
        print(f"ee_mass={props['ee_mass']:.6f}")
        print("ee_inertia=np.array([" + ", ".join(f"{v:.6g}" for v in props["ee_inertia"]) + "])")
        return 0
    if args.webcam is not None:
        idx = int(args.webcam) if str(args.webcam).isdigit() else args.webcam
        cam = RealWebcam(idx, spec=args.camera, width=args.width, height=args.height)
        rgb = cam.render()
        cam.close()
    else:
        from yam_sim.robot import YamSimRobot

        robot = YamSimRobot(start_thread=False, zero_gravity_mode=False, objects="keyboard", camera=args.camera, tool=tool,
                            initial_qpos=list(args.q) + [0.0])
        cam = WristCamera(robot, width=args.width, height=args.height)
        rgb = cam.render()
        print(cam.camera_info())
        cam.close()
    save_image(args.out, rgb)
    print(f"wrote {args.out} {rgb.shape}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
