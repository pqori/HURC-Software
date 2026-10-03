#!/usr/bin/env python3
"""Send one NavigateToPose goal to Nav2 and wait for the result.

    ros2 run rover_navigation go_to_pose.py --x 8.0 --y 0.0 --yaw 0.0

Exit code 0 on SUCCEEDED, 1 on abort, cancel, rejection or timeout.
The timeout is measured in sim time once the goal is accepted (wall-clock
time is used as a fallback guard at 3x the timeout).
"""

import argparse
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from std_srvs.srv import Trigger
import tf2_ros

STATUS_NAMES = {
    GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
    GoalStatus.STATUS_ABORTED: "ABORTED",
    GoalStatus.STATUS_CANCELED: "CANCELED",
    GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
}


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class GoToPose(Node):
    def __init__(self, args):
        super().__init__(
            "go_to_pose",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, args.use_sim_time)])
        self.args = args
        self.odom = None
        self.feedback = None
        self.create_subscription(Odometry, args.odom_topic, self._on_odom, 10)
        self.client = ActionClient(self, NavigateToPose, args.action)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def _on_odom(self, msg):
        self.odom = msg

    def _on_feedback(self, msg):
        self.feedback = msg.feedback

    def log(self, text):
        print(f"[go_to_pose] {text}", flush=True)

    def spin_until(self, predicate, wall_timeout, period=0.1):
        deadline = time.monotonic() + wall_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if predicate():
                return True
            rclpy.spin_once(self, timeout_sec=period)
        return predicate()

    def robot_pose(self):
        """(x, y, yaw, source) of base_link in the goal frame, or None."""
        try:
            t = self.tf_buffer.lookup_transform(self.args.frame, "base_link", Time())
            tr = t.transform
            return tr.translation.x, tr.translation.y, yaw_from_quat(tr.rotation), "tf"
        except Exception:  # noqa: BLE001 - fall back to odometry
            pass
        if self.odom is not None:
            p = self.odom.pose.pose
            return p.position.x, p.position.y, yaw_from_quat(p.orientation), "odom"
        return None

    def wait_nav2_active(self, wall_timeout):
        cli = self.create_client(Trigger, "/lifecycle_manager_navigation/is_active")
        deadline = time.monotonic() + wall_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if cli.wait_for_service(timeout_sec=1.0):
                fut = cli.call_async(Trigger.Request())
                self.spin_until(fut.done, 5.0)
                if fut.done() and fut.result() is not None and fut.result().success:
                    return True
            rclpy.spin_once(self, timeout_sec=1.0)
        return False

    def run(self):
        a = self.args
        self.log(f"waiting for Nav2 lifecycle manager to report active (<= {a.wait_timeout:.0f} s)")
        if not self.wait_nav2_active(a.wait_timeout):
            self.log("Nav2 lifecycle manager not active (continuing to wait for the action server)")
        self.log(f"waiting for action server {a.action}")
        if not self.client.wait_for_server(timeout_sec=a.wait_timeout):
            self.log("ERROR: NavigateToPose action server not available")
            return 1
        self.log(f"waiting for {a.odom_topic}")
        if not self.spin_until(lambda: self.odom is not None, a.wait_timeout):
            self.log(f"ERROR: no message on {a.odom_topic}")
            return 1
        # Give TF a moment to fill.
        self.spin_until(lambda: False, 1.0)

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = a.frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = a.x
        goal.pose.pose.position.y = a.y
        goal.pose.pose.orientation.z = math.sin(a.yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(a.yaw / 2.0)

        start = self.robot_pose()
        if start:
            self.log(f"start pose ({start[3]}): x={start[0]:.3f} y={start[1]:.3f} yaw={start[2]:.3f}")
        self.log(f"sending goal x={a.x:.3f} y={a.y:.3f} yaw={a.yaw:.3f} frame={a.frame}")
        send_fut = self.client.send_goal_async(goal, feedback_callback=self._on_feedback)
        if not self.spin_until(send_fut.done, 30.0) or send_fut.result() is None:
            self.log("ERROR: goal send timed out")
            return 1
        handle = send_fut.result()
        if not handle.accepted:
            self.log("ERROR: goal rejected")
            return 1

        t0_sim = self.get_clock().now()
        t0_wall = time.monotonic()
        result_fut = handle.get_result_async()
        next_print = 0.0
        while rclpy.ok() and not result_fut.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            sim_elapsed = (self.get_clock().now() - t0_sim).nanoseconds / 1e9
            wall_elapsed = time.monotonic() - t0_wall
            if sim_elapsed >= next_print:
                next_print = sim_elapsed + 2.0
                pose = self.robot_pose()
                line = f"t_sim={sim_elapsed:6.1f}s"
                if pose:
                    d = math.hypot(a.x - pose[0], a.y - pose[1])
                    line += f" pose=({pose[0]:.2f},{pose[1]:.2f},{pose[2]:.2f}) dist_to_goal={d:.2f} m"
                if self.feedback is not None:
                    line += f" nav2_remaining={self.feedback.distance_remaining:.2f} m"
                    line += f" recoveries={self.feedback.number_of_recoveries}"
                self.log(line)
            if sim_elapsed > a.timeout or wall_elapsed > 3.0 * a.timeout:
                self.log(f"TIMEOUT after {sim_elapsed:.1f} s sim ({wall_elapsed:.1f} s wall); canceling goal")
                cancel = handle.cancel_goal_async()
                self.spin_until(cancel.done, 5.0)
                return 1

        sim_elapsed = (self.get_clock().now() - t0_sim).nanoseconds / 1e9
        wall_elapsed = time.monotonic() - t0_wall
        res = result_fut.result()
        status = res.status if res is not None else GoalStatus.STATUS_UNKNOWN
        name = STATUS_NAMES.get(status, str(status))
        # Let odometry settle into the latest sample.
        self.spin_until(lambda: False, 0.5)
        pose = self.robot_pose()
        o = self.odom.pose.pose
        self.log(f"RESULT {name}")
        if res is not None and hasattr(res.result, "error_code") and res.result.error_code:
            self.log(f"error_code={res.result.error_code} {getattr(res.result, 'error_msg', '')}")
        self.log(f"elapsed sim time {sim_elapsed:.1f} s (wall {wall_elapsed:.1f} s)")
        if pose:
            d = math.hypot(a.x - pose[0], a.y - pose[1])
            self.log(f"final pose in {a.frame} ({pose[3]}): x={pose[0]:.3f} y={pose[1]:.3f} "
                     f"yaw={pose[2]:.3f} dist_to_goal={d:.3f} m")
        self.log(f"final {a.odom_topic}: x={o.position.x:.3f} y={o.position.y:.3f} "
                 f"yaw={yaw_from_quat(o.orientation):.3f}")
        return 0 if status == GoalStatus.STATUS_SUCCEEDED else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument("--x", type=float, required=True)
    parser.add_argument("--y", type=float, required=True)
    parser.add_argument("--yaw", type=float, default=0.0, help="radians")
    parser.add_argument("--frame", default="map")
    parser.add_argument("--timeout", type=float, default=180.0, help="seconds of sim time after the goal is accepted")
    parser.add_argument("--wait-timeout", type=float, default=120.0,
                        help="wall seconds to wait for Nav2 and odometry")
    parser.add_argument("--action", default="/navigate_to_pose")
    parser.add_argument("--odom-topic", default="/odometry/filtered")
    parser.add_argument("--no-sim-time", dest="use_sim_time", action="store_false")
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=[sys.argv[0]] + ros_args)
    node = GoToPose(args)
    try:
        code = node.run()
    except KeyboardInterrupt:
        code = 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
