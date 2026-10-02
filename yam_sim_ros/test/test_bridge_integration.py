"""Headless integration test: sim bridge + robot_state_publisher, driven over ROS.

Starts ``ros2 launch yam_sim_ros sim_bridge.launch.py`` (MuJoCo sim backend, no RViz) in its own
process group, then from this process:

* sends a FollowJointTrajectory goal to [0.5, 1.0, 0.8, -0.3, 0.4, 0.6] and expects SUCCESSFUL,
* checks /joint_states converges to it,
* checks TF yam_base -> yam_grasp (robot_state_publisher, from the URDF) against yam_sim's MuJoCo
  FK of ``grasp_site`` at the measured joints (position to 1e-3 m, rotation to 1e-3),
* checks out-of-limit / wrong-joint goals are aborted with the right error codes,
* checks the GripperCommand action moves both finger joints.

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
