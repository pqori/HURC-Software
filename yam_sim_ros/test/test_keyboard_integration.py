"""Headless integration test of the keyboard topics: ``sim_bridge.launch.py objects:=keyboard tool:=stylus``.

Checks, against the MuJoCo model built with the same options in this process:

* /yam_keyboard/pressed_keys publishes (empty at rest),
* the static TF yam_base -> yam_keyboard equals the keyboard body pose in the sim,
* /yam_keyboard/markers (latched) has one CUBE per key in namespace "keys", at the key centres,
* TF yam_gripper -> yam_stylus_tip (URDF tool:=stylus) equals the MuJoCo ``stylus_tip`` site,
* pressing "g" with the stylus through FollowJointTrajectory shows up on /yam_keyboard/typed and
  /yam_keyboard/pressed_keys.

The wrist camera is off here (camera:=none) to keep the run short; test_bridge_integration.py
covers it. Uses ROS_DOMAIN_ID from the environment, or 87 if unset.
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
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from rclpy.time import Time  # noqa: E402

from builtin_interfaces.msg import Duration  # noqa: E402
from control_msgs.action import FollowJointTrajectory  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from trajectory_msgs.msg import JointTrajectoryPoint  # noqa: E402
from visualization_msgs.msg import MarkerArray  # noqa: E402

ARM = [f"yam_joint{i}" for i in range(1, 7)]


def _quat_to_mat(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _tf_to_T(tf):
    T = np.eye(4)
    T[:3, :3] = _quat_to_mat(tf.rotation)
    T[:3, 3] = [tf.translation.x, tf.translation.y, tf.translation.z]
    return T


def _body_T(model, data, name):
    b = model.body(name).id
    T = np.eye(4)
    T[:3, :3] = data.xmat[b].reshape(3, 3)
    T[:3, 3] = data.xpos[b]
    return T


def _spin_until(node, pred, timeout):
    t0 = time.time()
    while time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.05)
        if pred():
            return True
    return False


@pytest.fixture(scope="module")
def sim():
    """The MuJoCo scene the bridge builds (same options), at qpos0."""
    import mujoco
    from yam_sim.assembly import load_scene

    model, _ = load_scene(camera="none", objects="keyboard", tool="stylus")
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data


@pytest.fixture(scope="module")
def bridge():
    proc = subprocess.Popen(
        ["ros2", "launch", "yam_sim_ros", "sim_bridge.launch.py", "rviz:=false",
         "objects:=keyboard", "tool:=stylus", "camera:=none"],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, start_new_session=True, env=os.environ.copy(),
    )
    rclpy.init()
    node = rclpy.create_node("yam_keyboard_test")
    state = {"pressed": [], "typed": []}
    node.create_subscription(String, "/yam_keyboard/pressed_keys", lambda m: state["pressed"].append(m.data), 50)
    node.create_subscription(String, "/yam_keyboard/typed", lambda m: state["typed"].append(m.data), 50)
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(MarkerArray, "/yam_keyboard/markers", lambda m: state.__setitem__("markers", m), latched)
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
            proc.send_signal(signal.SIGINT)
            proc.wait(timeout=15)
        except Exception:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait(timeout=5)
        out = proc.stdout.read().decode(errors="replace") if proc.stdout else ""
        print("---- launch output ----\n" + "\n".join(l for l in out.splitlines() if "got segment" not in l))


def test_pressed_keys_publishes(bridge):
    node, state, _ = bridge
    assert _spin_until(node, lambda: len(state["pressed"]) >= 10, 60.0), "no /yam_keyboard/pressed_keys"
    assert state["pressed"][-1] == "", f"keys down at rest: {state['pressed'][-1]!r}"
    assert state["typed"] == []


def test_keyboard_static_tf(bridge, sim):
    node, _, buf = bridge
    model, data = sim
    assert _spin_until(node, lambda: buf.can_transform("yam_base", "yam_keyboard", Time()), 30.0)
    T_tf = _tf_to_T(buf.lookup_transform("yam_base", "yam_keyboard", Time()).transform)
    T_sim = np.linalg.inv(_body_T(model, data, "base")) @ _body_T(model, data, "keyboard")
    print("yam_base->yam_keyboard TF:\n", T_tf.round(5), "\nsim:\n", T_sim.round(5))
    assert np.abs(T_tf[:3, 3] - T_sim[:3, 3]).max() < 1e-6
    assert np.abs(T_tf[:3, :3] - T_sim[:3, :3]).max() < 1e-6


def test_key_markers(bridge, sim):
    node, state, _ = bridge
    model, data = sim
    from yam_sim.keyboard import CAP_HALF_HEIGHT, Keyboard

    assert _spin_until(node, lambda: "markers" in state, 30.0), "no latched /yam_keyboard/markers"
    kb = Keyboard.from_model(model, data)
    keys = [m for m in state["markers"].markers if m.ns == "keys"]
    assert len(keys) == len(kb.key_names) == 104
    T_bw = np.linalg.inv(_body_T(model, data, "base"))
    for m in keys:
        name = kb.key_names[m.id]
        T = T_bw @ kb.key_pose(name)
        centre = T[:3, 3] - T[:3, 2] * CAP_HALF_HEIGHT
        p = np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z])
        assert m.header.frame_id == "yam_base" and m.type == m.CUBE
        assert np.abs(p - centre).max() < 1e-6, name
        assert np.allclose([m.scale.x, m.scale.y, m.scale.z], 2 * kb.key_half_size(name))


def test_stylus_tip_tf(bridge, sim):
    node, _, buf = bridge
    model, data = sim
    assert _spin_until(node, lambda: buf.can_transform("yam_gripper", "yam_stylus_tip", Time()), 10.0)
    T_tf = _tf_to_T(buf.lookup_transform("yam_gripper", "yam_stylus_tip", Time()).transform)
    s = model.site("stylus_tip").id
    T_site = np.eye(4)
    T_site[:3, :3] = data.site_xmat[s].reshape(3, 3)
    T_site[:3, 3] = data.site_xpos[s]
    T_sim = np.linalg.inv(_body_T(model, data, "gripper")) @ T_site
    assert np.abs(T_tf - T_sim).max() < 1e-6


def _move(node, client, q, sec):
    g = FollowJointTrajectory.Goal()
    g.trajectory.joint_names = ARM
    g.trajectory.points = [JointTrajectoryPoint(positions=[float(v) for v in q],
                                                time_from_start=Duration(sec=int(sec), nanosec=int((sec % 1) * 1e9)))]
    fut = client.send_goal_async(g)
    assert _spin_until(node, fut.done, 10.0) and fut.result().accepted
    rf = fut.result().get_result_async()
    assert _spin_until(node, rf.done, sec + 10.0)
    return rf.result().result


def test_press_a_key_with_the_stylus(bridge, sim):
    """Hover above "g", descend 5 mm below the keycap top, lift: one "g" on /typed."""
    node, state, _ = bridge
    model, _ = sim
    from yam_sim.kinematics import Kinematics
    from yam_sim.keyboard import Keyboard
    from yam_sim.scripts.press_keys import press_rotation

    kin = Kinematics(model=model)
    kb = Keyboard.from_model(model)
    top = kb.key_pose("g")[:3, 3]
    T = np.eye(4)
    T[:3, :3] = press_rotation(top[:2])
    seed = [np.arctan2(top[1], top[0]), 1.0, 1.0, -0.6, 0.0, 0.0]
    qs = []
    for dz in (0.03, -0.005):
        T[:3, 3] = top + [0, 0, dz]
        ok, q = kin.ik(T, "stylus_tip", init_q=seed, restarts=4, max_iters=300, seed=0)
        assert ok, f"IK failed for dz={dz}"
        qs.append(q)
        seed = q
    client = ActionClient(node, FollowJointTrajectory, "/yam_arm_controller/follow_joint_trajectory")
    assert client.wait_for_server(timeout_sec=30.0)
    res = _move(node, client, qs[0], 3.0)
    assert res.error_code == FollowJointTrajectory.Result.SUCCESSFUL, res.error_string
    _spin_until(node, lambda: False, 0.3)
    state["typed"].clear()
    state["pressed"].clear()
    _move(node, client, qs[1], 1.5)  # may end with a goal-tolerance error: the key pushes back
    assert _spin_until(node, lambda: "g" in state["typed"], 3.0), f"typed={state['typed']} pressed={state['pressed'][-3:]}"
    assert any("g" in p.split() for p in state["pressed"])
    res = _move(node, client, qs[0], 1.0)
    _spin_until(node, lambda: False, 0.5)
    assert state["typed"] == ["g"], state["typed"]
    assert state["pressed"][-1] == ""
