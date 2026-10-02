import numpy as np
import pytest

from yam_sim import YamSimRobot
from yam_sim.kinematics import Kinematics
from yam_sim.motion import move_to
from yam_sim.scripts import go_to, play_trajectory as pt


def sync_robot(**kw):
    return YamSimRobot(start_thread=False, zero_gravity_mode=False, **kw)


def test_go_to_joint_target_with_deg_and_grip():
    r = sync_robot()
    k = Kinematics(model=r.model)
    tgt = go_to.resolve_target(r.get_joint_pos(), k, joints=[0, 60, 60, -20, 0, 0], deg=True, gripper=1.0, gripper_index=6)
    np.testing.assert_allclose(tgt[:6], np.deg2rad([0, 60, 60, -20, 0, 0]))
    assert tgt[6] == 1.0
    final = move_to(r, tgt, duration=1.5)
    r.step_for(0.5)
    np.testing.assert_allclose(r.get_joint_pos(), tgt, atol=0.01)


def test_go_to_pose_via_ik():
    r = sync_robot()
    k = Kinematics(model=r.model)
    tgt = go_to.resolve_target(r.get_joint_pos(), k, pos=[0.35, 0.0, 0.25], rpy=[0, 1.57, 0], gripper_index=6)
    move_to(r, tgt, duration=2.0)
    r.step_for(0.5)
    p = r.get_ee_pose("grasp_site")[:3, 3]
    assert np.linalg.norm(p - [0.35, 0.0, 0.25]) < 0.005
    with pytest.raises(ValueError, match="IK"):
        go_to.resolve_target(r.get_joint_pos(), k, pos=[2.0, 0.0, 0.0], gripper_index=6)


@pytest.mark.parametrize("ext", ["csv", "json"])
def test_record_replay_round_trip(tmp_path, ext):
    # record a demo replay on one robot, save, load, replay on a fresh robot, compare
    t, q = pt.demo_trajectory()
    path = str(tmp_path / f"demo.{ext}")
    pt.save_trajectory(path, t, q)
    t2, q2 = pt.load_trajectory(path)
    np.testing.assert_allclose(t2, t)
    np.testing.assert_allclose(q2, q, atol=1e-6)

    r = sync_robot()
    _, cmd, meas = pt.replay(r, t2, q2, speed=1.0, rate=100.0)
    rec_path = str(tmp_path / f"rec.{ext}")
    rt, rq = pt.record(r, duration=0.5, rate=50.0)
    assert rq.shape == (25, 7)
    pt.save_trajectory(rec_path, rt, rq)
    lt, lq = pt.load_trajectory(rec_path)
    np.testing.assert_allclose(lq, rq, atol=1e-5)
    # replay tracked the authored trajectory and ended on its last waypoint
    np.testing.assert_allclose(cmd[-1], q[-1], atol=1e-9)
    np.testing.assert_allclose(lq[-1, :6], q[-1, :6], atol=0.02)
    assert np.sqrt(np.mean((cmd[:, :6] - meas[:, :6]) ** 2)) < 0.05


def test_load_trajectory_rejects_bad_time(tmp_path):
    p = tmp_path / "bad.csv"
    p.write_text("t,j1,j2,j3,j4,j5,j6\n0,0,0,0,0,0,0\n0,0,0,0,0,0,0\n")
    with pytest.raises(ValueError):
        pt.load_trajectory(str(p))
