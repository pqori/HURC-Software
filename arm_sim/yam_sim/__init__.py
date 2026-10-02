"""yam_sim: a MuJoCo physics simulation of the I2RT YAM arm with i2rt's ``Robot`` API.

    from yam_sim import make_robot
    robot = make_robot("sim")                   # or make_robot("real", channel="can0") on the arm's Linux host
    robot.command_joint_pos([0, 1.0, 1.0, 0, 0, 0, 1.0])
"""

from yam_sim.assembly import build_scene, build_scene_file, load_scene
from yam_sim.factory import RealBackendUnavailable, make_robot
from yam_sim.kinematics import Kinematics
from yam_sim.robot import YamSimRobot

HOME = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
# A comfortable "ready" pose: elbow up, gripper facing forward and down.
READY = (0.0, 1.0, 1.0, -0.3, 0.0, 0.0)

__all__ = [
    "YamSimRobot",
    "make_robot",
    "RealBackendUnavailable",
    "Kinematics",
    "build_scene",
    "build_scene_file",
    "load_scene",
    "HOME",
    "READY",
]
