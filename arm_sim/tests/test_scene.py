import mujoco
import numpy as np
import pytest

from yam_sim.assembly import build_scene, build_scene_file, load_scene
from yam_sim.config import GRIPPERS
from yam_sim.kinematics import Kinematics

# i2rt robot_models/arm/yam/v1/README.md, "Home configuration M": the `gripper` mount frame at q = 0.
HOME_POS = np.array([0.110598, 0.0, 0.173501])
HOME_ROT = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])


@pytest.mark.parametrize("gripper", GRIPPERS)
def test_scene_builds(gripper):
    model, info = load_scene("yam", gripper)
    data = mujoco.MjData(model)
    mujoco.mj_step(model, data)
    names = [model.joint(i).name for i in range(model.njnt)]
    assert names[:6] == [f"joint{i}" for i in range(1, 7)]
    assert model.body("target").mocapid[0] >= 0
    assert model.site("grasp_site").id >= 0 and model.site("tcp_site").id >= 0
    n_motors = 6 + (gripper != "no_gripper")
    assert model.nu == n_motors and len(info.torque_limits) == n_motors
    # Per-motor limits: DM4340 on joints 1-3, DM4310 on 4-6 and the gripper.
    assert np.allclose(info.torque_limits[:3], 27.0) and np.allclose(info.torque_limits[3:], 7.0)
    assert np.allclose(model.actuator_forcerange[:, 1], info.torque_limits)


def test_scene_xml_string_and_file(tmp_path):
    xml = build_scene("yam", "linear_4310")
    assert "<mujoco" in xml and 'name="target"' in xml
    path = build_scene_file("yam", "linear_4310", out_dir=str(tmp_path))
    mujoco.MjModel.from_xml_path(path)


def test_unvendored_arm_gives_clear_error():
    with pytest.raises(NotImplementedError, match="not vendored"):
        load_scene("big_yam", "linear_4310")


@pytest.mark.parametrize("gripper", GRIPPERS)
def test_fk_home_matches_i2rt_oracle(gripper):
    k = Kinematics("yam", gripper)
    T = k.fk(np.zeros(6), "gripper")
    np.testing.assert_allclose(T[:3, 3], HOME_POS, atol=1e-4)
    np.testing.assert_allclose(T[:3, :3], HOME_ROT, atol=1e-4)
    # tcp_site sits on the mount frame
    np.testing.assert_allclose(k.fk(np.zeros(6), "tcp_site")[:3, 3], HOME_POS, atol=1e-4)


def test_gripper_mass_is_merged():
    m_no, _ = load_scene("yam", "no_gripper")
    m_lin, _ = load_scene("yam", "linear_4310")
    # linear_4310 body 0.553 kg + two 0.071 kg tips
    assert m_lin.body_subtreemass[1] - m_no.body_subtreemass[1] == pytest.approx(0.553219 + 2 * 0.0710042, abs=1e-4)
