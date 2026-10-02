import warnings

import numpy as np
import pytest

from yam_sim import YamSimRobot, make_robot
from yam_sim.factory import RealBackendUnavailable
from yam_sim.kinematics import Kinematics, pose_error

POSE_A = np.array([0.3, 1.0, 1.2, -0.4, 0.2, 0.5])


def sync_robot(**kw):
    kw.setdefault("start_thread", False)
    return YamSimRobot(**kw)


def test_observation_keys_and_shapes_match_i2rt_simrobot():
    r = sync_robot()
    assert r.num_dofs() == 7
    obs = r.get_observations()
    assert set(obs) == {"joint_pos", "joint_vel", "joint_eff", "gripper_pos", "gripper_vel", "gripper_eff"}
    for k in ("joint_pos", "joint_vel", "joint_eff"):
        assert obs[k].shape == (6,)
    for k in ("gripper_pos", "gripper_vel", "gripper_eff"):
        assert obs[k].shape == (1,)
    assert r.get_joint_pos().shape == (7,)
    st = r.get_joint_state()
    assert set(st) == {"pos", "vel"} and st["pos"].shape == (7,)
    info = r.get_robot_info()
    for key in ("joint_limits", "gripper_limits", "gripper_index", "sim", "gravity_comp_factor", "kp", "kd"):
        assert key in info
    assert info["sim"] is True and info["gripper_index"] == 6
    assert info["joint_limits"].shape == (6, 2)
    np.testing.assert_allclose(info["kp"], [80, 80, 80, 10, 10, 10, 20])
    np.testing.assert_allclose(info["kd"], [5, 5, 5, 1.5, 1.5, 1.5, 0.5])


def test_no_gripper_observations():
    r = sync_robot(gripper="no_gripper")
    assert r.num_dofs() == 6
    assert set(r.get_observations()) == {"joint_pos", "joint_vel", "joint_eff"}
    assert r.get_robot_info()["gripper_index"] is None


def test_joint_limits_are_clipped():
    r = sync_robot(zero_gravity_mode=False)
    lim = r.get_robot_info()["joint_limits"]
    cmd = np.array([10.0, -10.0, 10.0, -10.0, 10.0, -10.0, 5.0])
    r.command_joint_pos(cmd)
    target = r.get_commanded_pos()
    np.testing.assert_allclose(target[:6], [lim[0, 1], lim[1, 0], lim[2, 1], lim[3, 0], lim[4, 1], lim[5, 0]])
    assert target[6] == 1.0
    r.command_joint_pos(np.array([0, 0, 0, 0, 0, 0, -3.0]))
    assert r.get_commanded_pos()[6] == 0.0
    # the caller's array is not modified
    assert cmd[0] == 10.0
    # the limits are the MJCF ranges widened by 0.15 rad, as in get_yam_robot
    np.testing.assert_allclose(lim[1], [0 - 0.15, 3.66519 + 0.15])


def test_pd_tracks_step_and_settles():
    q0 = np.append(POSE_A, 0.5)
    r = sync_robot(zero_gravity_mode=False, initial_qpos=q0)
    r.step_for(0.5)
    q1 = q0 + np.array([0.3, 0.3, -0.3, 0.3, 0.3, 0.3, 0.4])
    r.command_joint_pos(q1)
    r.step_for(1.5)
    err = np.abs(r.get_joint_pos() - q1)
    # Coulomb friction / kp gives a steady-state band: 0.3/80 rad on joints 1-3, 0.06/10 on 4-6.
    assert np.all(err[:3] < 0.01), err
    assert np.all(err[3:6] < 0.015), err
    assert err[6] < 0.02, err
    assert np.all(np.abs(r.get_joint_state()["vel"]) < 0.02)


def test_gravity_is_held_when_position_holding():
    q0 = np.append(POSE_A, 0.0)
    r = sync_robot(zero_gravity_mode=False, initial_qpos=q0)
    r.step_for(3.0)
    sag = np.abs(r.get_joint_pos()[:6] - POSE_A)
    assert np.all(sag < 5e-3), sag
    # with gravity comp the motors carry the arm's weight: joint3 needs several N m here
    assert abs(r.get_observations()["joint_eff"][2]) > 2.0


def test_gravity_hold_without_gravity_comp_sags_but_holds():
    q0 = np.append(POSE_A, 0.0)
    r = sync_robot(zero_gravity_mode=False, initial_qpos=q0, use_gravity_comp=False, friction=False)
    r.step_for(2.0)
    sag = np.abs(r.get_joint_pos()[:6] - POSE_A)
    assert sag.max() > 0.01  # PD alone needs an error to make torque
    assert sag.max() < 0.2


def test_zero_gravity_mode_keeps_arm_still():
    q0 = np.append(POSE_A, 0.0)
    r = sync_robot(zero_gravity_mode=True, initial_qpos=q0)
    assert np.all(r.get_command_gains()["kp"] == 0)
    r.step_for(3.0)
    np.testing.assert_allclose(r.get_joint_pos()[:6], POSE_A, atol=0.02)
    # without friction, gravity comp alone still holds it (the sim model is exact)
    r2 = sync_robot(zero_gravity_mode=True, initial_qpos=q0, friction=False)
    r2.step_for(3.0)
    np.testing.assert_allclose(r2.get_joint_pos()[:6], POSE_A, atol=0.02)
    # and without gravity comp it falls
    r3 = sync_robot(zero_gravity_mode=True, initial_qpos=q0, use_gravity_comp=False)
    r3.step_for(1.0)
    assert np.abs(r3.get_joint_pos()[:6] - POSE_A).max() > 0.2


def test_watchdog_bus_stall_drops_to_damping():
    q0 = np.append(POSE_A, 0.0)
    r = sync_robot(zero_gravity_mode=False, initial_qpos=q0)
    r.step_for(0.5)
    r.stall_bus(10.0)
    r.step_for(0.3)  # < 400 ms: the motors keep the last frame and the arm holds
    assert not r.watchdog_tripped
    assert np.abs(r.get_joint_pos()[:6] - POSE_A).max() < 0.01
    r.step_for(0.2)  # > 400 ms since the last frame
    assert r.watchdog_tripped and r.get_robot_info()["watchdog"]["tripped"]
    assert np.all(r.model.actuator_biasprm[r._act_ids, 1] == 0)  # kp = 0 in damping mode
    r.step_for(1.0)
    assert np.abs(r.get_joint_pos()[:6] - POSE_A).max() > 0.05  # sinking under damping only
    r.reinit()
    assert not r.watchdog_tripped


def test_watchdog_command_feed_and_disable():
    q0 = np.append(POSE_A, 0.0)
    r = sync_robot(zero_gravity_mode=False, initial_qpos=q0, watchdog_feed="command")
    r.command_joint_pos(q0)
    r.step_for(0.3)
    assert not r.watchdog_tripped
    r.step_for(0.2)
    assert r.watchdog_tripped
    r2 = sync_robot(zero_gravity_mode=False, initial_qpos=q0, watchdog_feed="command", watchdog=False)
    r2.command_joint_pos(q0)
    r2.step_for(1.0)
    assert not r2.watchdog_tripped


def test_torque_saturates_at_motor_limit():
    r = sync_robot(zero_gravity_mode=False, kp=[500] * 6)
    r.command_joint_pos(np.array([0, 2.0, 2.0, 1.0, 1.0, 1.0, 1.0]))
    r.step(1)
    tau = np.abs(r.get_applied_torques())
    lim = r.get_robot_info()["torque_limits"]
    assert np.all(tau <= lim + 1e-9)
    assert np.isclose(tau[1], 27.0) and np.isclose(tau[4], 7.0)


def test_gripper_opens_and_closes():
    r = sync_robot(zero_gravity_mode=False)
    r.command_joint_pos(np.array([0, 0, 0, 0, 0, 0, 1.0]))
    r.step_for(1.0)
    assert r.get_observations()["gripper_pos"][0] > 0.97
    r.command_joint_pos(np.array([0, 0, 0, 0, 0, 0, 0.0]))
    r.step_for(1.0)
    assert r.get_observations()["gripper_pos"][0] < 0.03


def test_gripper_force_limiter_engages_on_grasp():
    r = sync_robot(zero_gravity_mode=False, objects=True)
    k = Kinematics(model=r.model)
    R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1.0]])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [0.42, 0, 0.12]
    ok, qa = k.ik(T, init_q=[0, 1.2, 1.0, 0, 0, 0])
    T[2, 3] = 0.022
    ok2, qb = k.ik(T, init_q=qa)
    assert ok and ok2
    for q, g, t in [(qa, 1.0, 2.0), (qb, 1.0, 1.5), (qb, 0.0, 1.5), (qa, 0.0, 1.5)]:
        r.command_joint_pos(np.append(q, g))
        r.step_for(t)
    info = r.get_robot_info()
    assert info["gripper_clogged"]
    g = r.get_observations()["gripper_pos"][0]
    assert 0.3 < g < 0.5  # 4 cm cube in a 9.5 cm stroke
    assert r.data.body("cube").xpos[2] > 0.08  # lifted
    assert abs(r.get_observations()["gripper_eff"][0]) < 2.0  # limited, far below the 7 N m peak


def test_command_joint_state_and_target_vel():
    r = sync_robot(zero_gravity_mode=False)
    q = np.array([0.2, 0.5, 0.5, 0, 0, 0, 0.5])
    r.command_joint_state({"pos": q, "vel": np.zeros(7), "kp": np.full(7, 40.0), "kd": np.full(7, 2.0)})
    np.testing.assert_allclose(r.get_command_gains()["kp"], 40.0)
    r.step_for(1.5)
    np.testing.assert_allclose(r.get_joint_pos()[:3], q[:3], atol=0.02)
    with warnings.catch_warnings(record=True) as w:
        warnings.simplefilter("always")
        r.command_target_vel(np.ones(7))
        assert any("no-op" in str(x.message) for x in w)


def test_move_joints_and_reset():
    r = sync_robot(zero_gravity_mode=False)
    target = np.array([0.0, 0.8, 0.8, 0.0, 0.0, 0.0, 1.0])
    r.move_joints(target, time_interval_s=1.0)
    r.step_for(1.0)
    np.testing.assert_allclose(r.get_joint_pos()[:6], target[:6], atol=0.02)
    r.reset()
    np.testing.assert_allclose(r.get_joint_pos(), 0.0, atol=1e-9)


def test_threaded_mode_runs_in_real_time():
    import time

    with YamSimRobot(zero_gravity_mode=False) as r:
        t0 = r.sim_time
        time.sleep(0.5)
        r.command_joint_pos(np.array([0.0, 0.6, 0.6, 0.0, 0.0, 0.0, 1.0]))
        time.sleep(1.5)
        elapsed = r.sim_time - t0
        q = r.get_joint_pos()
    assert 1.5 < elapsed < 2.6
    np.testing.assert_allclose(q[1:3], [0.6, 0.6], atol=0.03)


def test_ik_round_trip():
    k = Kinematics()
    rng = np.random.default_rng(1)
    lo, hi = k.joint_limits[:, 0], k.joint_limits[:, 1]
    for _ in range(5):
        q_true = rng.uniform(lo + 0.3 * (hi - lo), hi - 0.3 * (hi - lo))
        T = k.fk(q_true)
        ok, q = k.ik(T, init_q=np.clip(q_true + 0.2, lo, hi))
        assert ok
        dp, dr = pose_error(k.fk(q), T)
        assert dp < 1e-3 and dr < 1e-2
        assert np.all(q >= lo - 1e-9) and np.all(q <= hi + 1e-9)


def test_ik_position_only():
    k = Kinematics()
    ok, q = k.ik(np.array([0.35, 0.1, 0.25]), init_q=[0, 1, 1, 0, 0, 0])
    assert ok
    assert np.linalg.norm(k.fk(q)[:3, 3] - [0.35, 0.1, 0.25]) < 1e-3


def test_make_robot_sim_and_real_error(monkeypatch):
    import builtins

    r = make_robot("sim", start_thread=False)
    assert isinstance(r, YamSimRobot)
    real_import = builtins.__import__

    def fake_import(name, *a, **kw):
        if name.startswith("i2rt"):
            raise ImportError("No module named 'i2rt'")
        return real_import(name, *a, **kw)

    monkeypatch.setattr(builtins, "__import__", fake_import)
    with pytest.raises(RealBackendUnavailable, match="pip install -e"):
        make_robot("real", channel="can0")
    with pytest.raises(ValueError):
        make_robot("banana")
