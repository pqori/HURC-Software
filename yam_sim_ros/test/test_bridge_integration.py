"""Headless integration test: sim bridge + robot_state_publisher, driven over ROS.

Starts ``ros2 launch yam_sim_ros sim_bridge.launch.py`` (MuJoCo sim backend, no RViz) in its own
process group, then from this process:

* sends a FollowJointTrajectory goal to [0.5, 1.0, 0.8, -0.3, 0.4, 0.6] and expects SUCCESSFUL,
* checks /joint_states converges to it,
* checks TF yam_base -> yam_grasp (robot_state_publisher, from the URDF) against yam_sim's MuJoCo
  FK of ``grasp_site`` at the measured joints (position to 1e-3 m, rotation to 1e-3),
* checks out-of-limit / wrong-joint goals are aborted with the right error codes,
* checks the GripperCommand action moves both finger joints,
* checks the wrist camera (default camera:=c920): /yam_wrist_camera/image_raw size, encoding, step
  and rate; /yam_wrist_camera/camera_info K = yam_sim.camera.camera_spec("c920").K(); and TF
  yam_gripper -> yam_wrist_camera_optical_frame (URDF) = the MuJoCo camera pose.

The keyboard topics are tested in test_keyboard_integration.py (separate launch, objects:=keyboard).

Run with the workspace sourced:  pixi run test   (or: python -m pytest -v yam_sim_ros/test)
Uses ROS_DOMAIN_ID from the environment, or 87 if unset.
"""

import os
import signal
import subprocess
import time

import numpy as np
import pytest

os.environ.setdefault("ROS_DOMAIN_ID", "87")

import rclpy  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from rclpy.time import Time  # noqa: E402

from builtin_interfaces.msg import Duration  # noqa: E402
from control_msgs.action import FollowJointTrajectory, GripperCommand  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from trajectory_msgs.msg import JointTrajectoryPoint  # noqa: E402

ARM = [f"yam_joint{i}" for i in range(1, 7)]
TARGET = [0.5, 1.0, 0.8, -0.3, 0.4, 0.6]


def _quat_to_mat(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


@pytest.fixture(scope="module")
def bridge():
    proc = subprocess.Popen(
        ["ros2", "launch", "yam_sim_ros", "sim_bridge.launch.py", "rviz:=false"],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, start_new_session=True, env=os.environ.copy(),
    )
    rclpy.init()
    node = rclpy.create_node("yam_bridge_test")
    state = {}
    node.create_subscription(JointState, "/joint_states", lambda m: state.__setitem__("js", m), 10)
    import tf2_ros

    buf = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buf, node, spin_thread=False)
    try:
        yield node, state, buf
    finally:
        del listener
        node.destroy_node()
        rclpy.shutdown()
        try:
            proc.send_signal(signal.SIGINT)  # ros2 launch forwards it to its children
            proc.wait(timeout=15)
        except Exception:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait(timeout=5)
        out = proc.stdout.read().decode(errors="replace") if proc.stdout else ""
        print("---- launch output ----\n" + "\n".join(l for l in out.splitlines() if "got segment" not in l))


def _spin_until(node, pred, timeout):
    t0 = time.time()
    while time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.05)
        if pred():
            return True
    return False


def _send(node, client, goal, timeout=30.0):
    fut = client.send_goal_async(goal)
    assert _spin_until(node, fut.done, 10.0), "goal was not acknowledged"
    gh = fut.result()
    assert gh.accepted
    rf = gh.get_result_async()
    assert _spin_until(node, rf.done, timeout), "no result"
    return rf.result()


def _fjt_goal(positions, names=ARM, sec=2):
    g = FollowJointTrajectory.Goal()
    g.trajectory.joint_names = list(names)
    g.trajectory.points = [JointTrajectoryPoint(positions=list(positions), time_from_start=Duration(sec=sec))]
    return g


def test_follow_joint_trajectory_and_tf(bridge):
    node, state, buf = bridge
    client = ActionClient(node, FollowJointTrajectory, "/yam_arm_controller/follow_joint_trajectory")
    assert client.wait_for_server(timeout_sec=60.0), "bridge action server did not come up"
    assert _spin_until(node, lambda: "js" in state, 10.0), "no /joint_states"

    res = _send(node, client, _fjt_goal(TARGET, sec=3))
    assert res.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL, res.result.error_string

    # /joint_states converges
    def converged():
        js = state.get("js")
        if js is None:
            return False
        q = dict(zip(js.name, js.position))
        return all(abs(q[j] - t) < 0.01 for j, t in zip(ARM, TARGET))

    assert _spin_until(node, converged, 5.0), f"joint_states did not converge: {state['js'].position}"
    js = state["js"]
    q = np.array([dict(zip(js.name, js.position))[j] for j in ARM])
    print("measured q:", q.round(5))

    # TF from robot_state_publisher vs MuJoCo FK of grasp_site, at the same measured joints
    _spin_until(node, lambda: False, 1.0)  # let TF catch up to the latest joint_states
    js = state["js"]
    q = np.array([dict(zip(js.name, js.position))[j] for j in ARM])
    assert _spin_until(node, lambda: buf.can_transform("yam_base", "yam_grasp", Time()), 10.0)
    tf = buf.lookup_transform("yam_base", "yam_grasp", Time()).transform
    p_tf = np.array([tf.translation.x, tf.translation.y, tf.translation.z])
    R_tf = _quat_to_mat(tf.rotation)

    from yam_sim import Kinematics

    T_mj = Kinematics("yam", "linear_4310").fk(q, "grasp_site")
    dp = np.abs(p_tf - T_mj[:3, 3]).max()
    dR = np.abs(R_tf - T_mj[:3, :3]).max()
    print(f"TF yam_base->yam_grasp p={p_tf.round(5)}  MuJoCo grasp_site p={T_mj[:3, 3].round(5)}  "
          f"max|dp|={dp:.2e} m max|dR|={dR:.2e}")
    assert dp < 1e-3
    assert dR < 1e-3


def test_invalid_goals_are_aborted(bridge):
    node, _, _ = bridge
    client = ActionClient(node, FollowJointTrajectory, "/yam_arm_controller/follow_joint_trajectory")
    assert client.wait_for_server(timeout_sec=60.0)
    bad = list(TARGET)
    bad[1] = 4.0  # joint2 upper limit is 3.66519
    res = _send(node, client, _fjt_goal(bad))
    assert res.result.error_code == FollowJointTrajectory.Result.INVALID_GOAL
    res = _send(node, client, _fjt_goal(TARGET[:5], names=ARM[:5]))
    assert res.result.error_code == FollowJointTrajectory.Result.INVALID_JOINTS


def test_joint_reordering(bridge):
    node, state, _ = bridge
    client = ActionClient(node, FollowJointTrajectory, "/yam_arm_controller/follow_joint_trajectory")
    assert client.wait_for_server(timeout_sec=60.0)
    target = [0.2, 0.6, 0.5, 0.1, -0.2, 0.3]
    names = list(reversed(ARM))
    res = _send(node, client, _fjt_goal(list(reversed(target)), names=names))
    assert res.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL, res.result.error_string
    _spin_until(node, lambda: False, 0.5)
    q = dict(zip(state["js"].name, state["js"].position))
    assert np.allclose([q[j] for j in ARM], target, atol=0.02)


def test_gripper_command(bridge):
    node, state, _ = bridge
    client = ActionClient(node, GripperCommand, "/yam_gripper_controller/gripper_cmd")
    assert client.wait_for_server(timeout_sec=60.0)
    g = GripperCommand.Goal()
    g.command.position = 0.03
    res = _send(node, client, g, timeout=15.0)
    assert res.result.reached_goal, res.result
    _spin_until(node, lambda: False, 1.0)
    q = dict(zip(state["js"].name, state["js"].position))
    assert abs(q["yam_joint7"] - 0.03) < 0.003
    assert abs(q["yam_joint8"] - q["yam_joint7"]) < 1e-9


def test_preempt_aborts_previous_goal(bridge):
    node, state, _ = bridge
    client = ActionClient(node, FollowJointTrajectory, "/yam_arm_controller/follow_joint_trajectory")
    assert client.wait_for_server(timeout_sec=60.0)
    f1 = client.send_goal_async(_fjt_goal([0.0, 0.5, 0.5, 0.0, 0.0, 0.0], sec=5))
    assert _spin_until(node, f1.done, 10.0) and f1.result().accepted
    r1 = f1.result().get_result_async()
    _spin_until(node, lambda: False, 0.5)
    res2 = _send(node, client, _fjt_goal(TARGET, sec=2))
    assert _spin_until(node, r1.done, 5.0)
    from action_msgs.msg import GoalStatus

    assert r1.result().status == GoalStatus.STATUS_ABORTED
    assert "preempted" in r1.result().result.error_string
    assert res2.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL


def test_topic_interfaces(bridge):
    """JTC-style /joint_trajectory streaming and the Float64MultiArray gripper topic."""
    node, state, _ = bridge
    from std_msgs.msg import Float64MultiArray
    from trajectory_msgs.msg import JointTrajectory

    traj_pub = node.create_publisher(JointTrajectory, "/yam_arm_controller/joint_trajectory", 10)
    grip_pub = node.create_publisher(Float64MultiArray, "/yam_gripper_controller/commands", 10)
    _spin_until(node, lambda: traj_pub.get_subscription_count() > 0 and grip_pub.get_subscription_count() > 0, 10.0)
    target = [-0.4, 0.8, 0.6, 0.2, 0.0, -0.5]
    msg = JointTrajectory(joint_names=ARM)
    msg.points = [JointTrajectoryPoint(positions=target, time_from_start=Duration(sec=1))]
    traj_pub.publish(msg)
    grip_pub.publish(Float64MultiArray(data=[0.01]))

    def reached():
        q = dict(zip(state["js"].name, state["js"].position))
        return np.allclose([q[j] for j in ARM], target, atol=0.02) and abs(q["yam_joint7"] - 0.01) < 0.003

    assert _spin_until(node, reached, 6.0), state["js"].position


# ---------------------------------------------------------------------------------------------
# Wrist camera (launched with the defaults: camera:=c920, camera_rate:=15, 1280x720)
# ---------------------------------------------------------------------------------------------
CAMERA_RATE = 15.0


def _collect_camera(node, seconds):
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CameraInfo, Image

    images, infos = [], []
    qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
    # keep only the metadata of each image (the 2.7 MB payload is checked on the first one)
    first = {}

    def on_image(m):
        if "img" not in first:
            first["img"] = m
        images.append((time.monotonic(), Time.from_msg(m.header.stamp).nanoseconds,
                       m.width, m.height, m.encoding, m.step, len(m.data), m.header.frame_id))

    s1 = node.create_subscription(Image, "/yam_wrist_camera/image_raw", on_image, qos)
    s2 = node.create_subscription(CameraInfo, "/yam_wrist_camera/camera_info", infos.append, qos)
    assert _spin_until(node, lambda: len(images) > 0, 30.0), "no /yam_wrist_camera/image_raw"
    images.clear()
    infos.clear()
    _spin_until(node, lambda: False, seconds)
    node.destroy_subscription(s1)
    node.destroy_subscription(s2)
    return images, infos, first["img"]


def test_wrist_camera_image_and_info(bridge):
    node, _, _ = bridge
    from yam_sim.camera import camera_spec

    images, infos, img = _collect_camera(node, 3.0)
    assert len(images) >= 10, f"only {len(images)} images in 3 s"
    for _, _, w, h, enc, step, n, frame in images:
        assert (w, h, enc, step, n, frame) == (1280, 720, "rgb8", 1280 * 3, 1280 * 720 * 3,
                                               "yam_wrist_camera_optical_frame")
    stamps_ns = [i[1] for i in images]
    rate = (len(stamps_ns) - 1) / ((stamps_ns[-1] - stamps_ns[0]) * 1e-9)
    print(f"wrist camera: {len(images)} frames, stamp rate {rate:.2f} Hz (requested {CAMERA_RATE})")
    assert abs(rate - CAMERA_RATE) < 0.15 * CAMERA_RATE
    # the image is not blank (the arm at TARGET sees the floor / skybox)
    px = np.frombuffer(bytes(img.data), np.uint8)
    assert px.std() > 5.0

    # (b) camera_info: K from the same spec, same stamps as the images
    assert infos, "no /yam_wrist_camera/camera_info"
    K = camera_spec("c920").K(1280, 720)
    info = infos[-1]
    assert info.header.frame_id == "yam_wrist_camera_optical_frame"
    assert (info.width, info.height) == (1280, 720)
    np.testing.assert_allclose(np.array(info.k).reshape(3, 3), K, rtol=0, atol=1e-9)
    np.testing.assert_allclose(np.array(info.p).reshape(3, 4), np.hstack([K, np.zeros((3, 1))]), rtol=0, atol=1e-9)
    np.testing.assert_allclose(info.r, np.eye(3).reshape(-1))
    assert info.distortion_model == "plumb_bob" and list(info.d) == [0.0] * 5
    info_stamps = {Time.from_msg(i.header.stamp).nanoseconds for i in infos}
    img_stamps = set(stamps_ns)
    shared = len(img_stamps & info_stamps)
    assert shared >= len(img_stamps) - 2, f"only {shared}/{len(img_stamps)} images have a camera_info with the same stamp"


def test_wrist_camera_tf_matches_sim(bridge):
    """URDF gripper -> optical frame (robot_state_publisher) vs the MuJoCo camera at a random pose."""
    node, _, buf = bridge
    import mujoco
    from yam_sim.assembly import load_scene
    from yam_sim.camera import WristCamera

    assert _spin_until(node, lambda: buf.can_transform("yam_gripper", "yam_wrist_camera_optical_frame", Time()), 10.0)
    tf = buf.lookup_transform("yam_gripper", "yam_wrist_camera_optical_frame", Time()).transform
    T_tf = np.eye(4)
    T_tf[:3, :3] = _quat_to_mat(tf.rotation)
    T_tf[:3, 3] = [tf.translation.x, tf.translation.y, tf.translation.z]

    model, _ = load_scene(camera="c920")
    data = mujoco.MjData(model)
    data.qpos[:6] = [0.3, 1.1, 0.7, -0.4, 0.5, -0.8]
    mujoco.mj_forward(model, data)
    cam = WristCamera(model, data=data, width=64, height=36)  # pose() = optical frame (ROS convention)
    T_cam = cam.pose(synced=True)
    cam.close()
    g = model.body("gripper").id
    T_g = np.eye(4)
    T_g[:3, :3] = data.xmat[g].reshape(3, 3)
    T_g[:3, 3] = data.xpos[g]
    T_sim = np.linalg.inv(T_g) @ T_cam
    dp = np.abs(T_tf[:3, 3] - T_sim[:3, 3]).max()
    dR = np.abs(T_tf[:3, :3] - T_sim[:3, :3]).max()
    print(f"gripper->optical: TF p={T_tf[:3, 3].round(6)} sim p={T_sim[:3, 3].round(6)} |dp|={dp:.1e} |dR|={dR:.1e}")
    assert dp <= 1e-4
    assert dR <= 1e-4
    # sanity: the optical axis (+Z) points mostly along the approach axis, tilted towards +Y (down)
    z = T_tf[:3, 2]
    assert z[2] > 0.85 and z[1] > 0.35
