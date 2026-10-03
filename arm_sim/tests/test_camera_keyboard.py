"""Wrist camera, keyboard, stylus, dataset generator and key-pressing demo (all headless)."""

import json
import os
import time

import mujoco
import numpy as np
import pytest
import yaml

from yam_sim import YamSimRobot
from yam_sim.assembly import load_scene
from yam_sim.camera import CAMERA_NAME, WristCamera, camera_spec
from yam_sim.keyboard import TRAVEL, U, Keyboard, char_to_key, keys_to_text

LOOK_DOWN_Q = [0.0, 1.2, 1.0, -0.9, 0.0, 0.0]


def sync_robot(**kw):
    kw.setdefault("start_thread", False)
    kw.setdefault("zero_gravity_mode", False)
    return YamSimRobot(**kw)


# ------------------------------------------------------------------------------------ camera
@pytest.mark.parametrize("model_name", ["c920", "c270"])
def test_camera_renders_with_spec_resolution(model_name):
    spec = camera_spec(model_name)
    r = sync_robot(camera=model_name, objects="keyboard", initial_qpos=LOOK_DOWN_Q + [0.0])
    cid = r.model.camera(CAMERA_NAME).id
    assert r.model.cam_fovy[cid] == pytest.approx(spec.vfov_deg, abs=1e-4)
    assert tuple(r.model.cam_resolution[cid]) == spec.default_resolution == (1280, 720)
    with WristCamera(r) as cam:
        img = cam.render()
        assert img.shape == (720, 1280, 3) and img.dtype == np.uint8
        assert img.std() > 10  # not a blank frame
    with WristCamera(r, width=640, height=360) as cam:
        assert cam.render().shape == (360, 640, 3)
        assert cam.depth().shape == (360, 640)
        seg = cam.segmentation("body")
        assert seg.shape == (360, 640) and seg.max() > 0
    if model_name == "c920":
        with WristCamera(r, width=1920, height=1080) as cam:  # the offscreen buffer is big enough for 1080p
            assert cam.render().shape == (1080, 1920, 3)


@pytest.mark.parametrize("model_name", ["c920", "c270"])
def test_intrinsics_consistent_with_fov_and_yaml(model_name):
    spec = camera_spec(model_name)
    with open(spec.path) as f:
        y = yaml.safe_load(f)
    # the YAML's diagonal / horizontal / vertical FOVs agree for a 16:9 sensor
    t = np.tan(np.deg2rad(spec.diagonal_fov_deg) / 2)
    d = np.hypot(16, 9)
    assert np.rad2deg(2 * np.arctan(t * 9 / d)) == pytest.approx(spec.vfov_deg, abs=0.01)
    assert np.rad2deg(2 * np.arctan(t * 16 / d)) == pytest.approx(spec.hfov_deg, abs=0.01)
    for mode in y["modes"]:
        i = spec.intrinsics(mode["width"], mode["height"])
        for k in ("fx", "fy", "cx", "cy"):
            assert i[k] == pytest.approx(mode[k], abs=0.05), (mode, k)
        # fx also reproduces the horizontal FOV (square pixels)
        assert np.rad2deg(2 * np.arctan(mode["width"] / 2 / i["fx"])) == pytest.approx(spec.hfov_deg, abs=0.05)


def test_camera_mount_pose_and_camera_info():
    spec = camera_spec("c920")
    r = sync_robot(initial_qpos=[0.3, 1.0, 0.8, -0.5, 0.4, 0.2, 0.0])
    with WristCamera(r, width=640, height=360) as cam:
        T_cam = cam.pose()
        info = cam.camera_info()
    T_grip = np.eye(4)
    bid = r.model.body("gripper").id
    T_grip[:3, :3] = r.data.xmat[bid].reshape(3, 3)
    T_grip[:3, 3] = r.data.xpos[bid]
    rel = np.linalg.inv(T_grip) @ T_cam
    np.testing.assert_allclose(rel[:3, 3], spec.mount_pos, atol=1e-6)
    R = np.empty(9)
    mujoco.mju_quat2Mat(R, spec.mount_quat)
    np.testing.assert_allclose(rel[:3, :3], R.reshape(3, 3), atol=1e-6)
    # the optical axis is tilted from the gripper axis towards the gripper's +Y by the yaml tilt
    assert np.rad2deg(np.arctan2(rel[1, 2], rel[2, 2])) == pytest.approx(spec.tilt_deg, abs=1e-3)
    assert info["frame_id"] == "wrist_camera_optical_frame" and info["width"] == 640 and info["height"] == 360
    assert len(info["K"]) == 9 and info["D"] == [0.0] * 5
    # the camera mass is part of the gripper subtree
    m_none, _ = load_scene(camera="none")
    assert r.model.body_subtreemass[bid] - m_none.body_subtreemass[m_none.body("gripper").id] == pytest.approx(spec.mass, abs=1e-6)


def test_project_grasp_site_hits_marker_centroid():
    # Open fingers so the space at grasp_site is free, park a 4 mm marker sphere (the IK target
    # mocap) exactly on grasp_site, and compare its segmentation centroid with project(grasp_site).
    r = sync_robot(initial_qpos=LOOK_DOWN_Q + [1.0])
    m = r.model
    gid = m.geom("target_geom").id
    m.geom_size[gid, 0] = 0.004
    m.geom_rgba[gid] = [1, 0, 0, 1]
    p = r.get_ee_pose("grasp_site")[:3, 3]
    r.data.mocap_pos[m.body("target").mocapid[0]] = p
    opt = mujoco.MjvOption()
    opt.geomgroup[:] = 0
    opt.geomgroup[0] = opt.geomgroup[1] = opt.geomgroup[2] = 1
    opt.sitegroup[:] = 0  # hide the target's axis sites
    with WristCamera(r, width=1280, height=720, scene_option=opt) as cam:
        seg = cam.segmentation("geom")
        uv, z = cam.project(p, return_depth=True)
        depth = cam.depth()
    ys, xs = np.nonzero(seg == gid)
    assert len(xs) > 50, "marker not visible"
    centroid = np.array([xs.mean(), ys.mean()])
    assert np.linalg.norm(centroid - uv) < 2.0, (centroid, uv)
    # grasp_site sits below the image centre (the camera looks past it), but well inside the frame
    assert 360 < uv[1] < 650 and abs(uv[0] - 639.5) < 3
    # depth at the marker centre = distance to the near side of the sphere
    assert depth[int(round(uv[1])), int(round(uv[0]))] == pytest.approx(z - 0.004, abs=0.002)


def test_no_camera_option():
    m, _ = load_scene(camera="none")
    assert mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, CAMERA_NAME) < 0
    with pytest.raises(ValueError):
        camera_spec("c999")


# ------------------------------------------------------------------------------------ keyboard
def test_keyboard_layout_counts_and_pitch():
    full, tkl = Keyboard("full"), Keyboard("tkl")
    assert len(full.key_names) == 104 and len(set(full.key_names)) == 104
    assert len(tkl.key_names) == 87
    c = full.key_local_center
    assert np.linalg.norm(c("w") - c("q")) == pytest.approx(U, abs=1e-9)
    assert np.linalg.norm(c("2") - c("1")) == pytest.approx(U, abs=1e-9)
    assert c("q")[1] - c("a")[1] == pytest.approx(U, abs=1e-9)  # rows are one pitch apart
    assert U == pytest.approx(0.01905)
    widths = {"space": 6.25, "lshift": 2.25, "rshift": 2.75, "enter": 2.25, "backspace": 2.0, "tab": 1.5, "caps": 1.75}
    for k, w in widths.items():
        assert full.key_half_size(k)[0] * 2 == pytest.approx(w * U - 0.0015, abs=1e-9), k
    assert full.key_half_size("kp_plus")[1] * 2 == pytest.approx(2 * U - 0.0015, abs=1e-9)
    # the default placement is in reach, in front of the arm, keys up
    m, _ = load_scene(objects="keyboard")
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    kb = Keyboard.from_model(m, d)
    assert kb.layout == "full" and len(kb.key_names) == 104
    for k in kb.key_names:
        assert m.geom(f"key_{k}").id >= 0
    tops = np.array([kb.key_pose(k)[:3, 3] for k in kb.key_names])
    assert 0.27 < tops[:, 0].min() and tops[:, 0].max() < 0.42
    assert abs(tops[:, 1].mean()) < 0.03
    np.testing.assert_allclose(kb.key_pose("a")[:3, 2], [0, 0, 1], atol=1e-9)
    assert kb.key_pose("esc")[1, 3] > kb.key_pose("enter")[1, 3]  # Esc on the robot's left (+Y)
    assert kb.key_pose("space")[0, 3] < kb.key_pose("f5")[0, 3]  # space bar nearest the robot
    # nominal (unbound) geometry agrees with the compiled model
    nominal = Keyboard("full")
    np.testing.assert_allclose(nominal.key_pose("j")[:3, 3], kb.key_pose("j")[:3, 3], atol=1e-7)
    np.testing.assert_allclose(nominal.key_bbox_world("space"), kb.key_bbox_world("key_space"), atol=1e-7)
    assert kb.key_bbox_world("space").shape == (8, 3)


def test_pushing_a_key_registers_and_springs_back():
    r = sync_robot(objects="keyboard")
    kb = Keyboard.from_model(r.model, r.data)
    r.step_for(0.3)
    assert kb.pressed_keys() == [] and np.abs(kb.travel()).max() < 1e-4  # keys rest still, no jitter
    base_pos = r.data.xpos[r.model.body("keyboard").id].copy()
    f = np.zeros((r.model.nbody, 6))
    f[r.model.body("key_g_body").id, 2] = -1.5  # N, more than the 0.8 N bottom-out spring force
    r.set_external_forces(f)
    r.step_for(0.3)
    assert kb.pressed_keys() == ["g"]
    assert TRAVEL - 0.0002 < kb.travel()[kb.key_names.index("g")] < TRAVEL + 0.0008  # stops at bottom-out
    r.set_external_forces(None)
    r.step_for(0.3)
    assert kb.pressed_keys() == [] and kb.travel()[kb.key_names.index("g")] < 2e-4
    np.testing.assert_allclose(r.data.xpos[r.model.body("keyboard").id], base_pos)


def test_char_mapping_round_trip():
    assert char_to_key("A") == ("a", True) and char_to_key(" ") == ("space", False) and char_to_key("!") == ("1", True)
    assert keys_to_text(["h", "i", "space", "lshift", "1"]) == "hi !"
    assert keys_to_text(["lshift", "a", "b"]) == "Ab"


def test_cube_and_keyboard_together():
    m, _ = load_scene(objects="cube,keyboard")
    assert m.body("cube").id >= 0 and m.body("keyboard").id >= 0
    m2, _ = load_scene(objects=True)
    assert m2.body("cube").id >= 0 and mujoco.mj_name2id(m2, mujoco.mjtObj.mjOBJ_BODY, "keyboard") < 0


# ------------------------------------------------------------------------------------ stylus
def test_stylus_tip_site_position():
    from yam_sim.kinematics import Kinematics

    m, _ = load_scene(tool="stylus")
    k = Kinematics(model=m)
    for q in (np.zeros(6), np.array(LOOK_DOWN_Q)):
        Tg = k.fk(q, "gripper")
        Tt = k.fk(q, "stylus_tip")
        Ts = k.fk(q, "grasp_site")
        rel = np.linalg.inv(Tg) @ Tt
        np.testing.assert_allclose(rel[:3, 3], [0, 0, 0.185], atol=1e-9)  # on the gripper +Z axis
        np.testing.assert_allclose(Tt[:3, :3], Ts[:3, :3], atol=1e-9)  # same axes as grasp_site
        d = np.linalg.inv(Ts) @ Tt
        assert d[2, 3] == pytest.approx(0.185 - 0.14465, abs=2e-4)  # 4 cm past the fingertips
    np.testing.assert_allclose(k.fk(np.zeros(6), "stylus_tip")[:3, 3], [0.1106 + 0.185, 0, 0.1735], atol=2e-4)


# ------------------------------------------------------------------------------------ dataset
def test_dataset_generation_writes_valid_yolo_and_coco(tmp_path):
    from yam_sim.scripts.generate_key_dataset import generate

    out = str(tmp_path / "ds")
    stats = generate(out, n=3, seed=7, width=640, height=360, n_viz=1, val_frac=0.34, verbose=False)
    assert stats["images"] == 3 and stats["boxes"] > 10
    with open(os.path.join(out, "data.yaml")) as f:
        dy = yaml.safe_load(f)
    nc = dy["nc"]
    assert nc == 104 and len(dy["names"]) == 104 and dy["names"][0] == "esc"
    n_lines = 0
    for split in ("train", "val"):
        imgs = sorted(os.listdir(os.path.join(out, "images", split)))
        for img in imgs:
            lab = os.path.join(out, "labels", split, img.replace(".jpg", ".txt"))
            assert os.path.exists(lab)
            for line in open(lab).read().split("\n"):
                if not line:
                    continue
                parts = line.split()
                assert len(parts) == 5
                c = int(parts[0])
                vals = np.array([float(v) for v in parts[1:]])
                assert 0 <= c < nc
                assert np.all(vals >= 0) and np.all(vals <= 1) and np.all(vals[2:] > 0)
                n_lines += 1
    assert n_lines == stats["boxes"]
    with open(os.path.join(out, "annotations.json")) as f:
        coco = json.load(f)
    assert len(coco["images"]) == 3 and len(coco["annotations"]) == n_lines and len(coco["categories"]) == 104
    for a in coco["annotations"]:
        x, y, w, h = a["bbox"]
        assert x >= 0 and y >= 0 and x + w <= 640 + 1e-6 and y + h <= 360 + 1e-6 and w >= 4 and h >= 4
    assert os.path.exists(os.path.join(out, "viz", "000000_viz.jpg"))
    # deterministic for a seed
    stats2 = generate(str(tmp_path / "ds2"), n=3, seed=7, width=640, height=360, n_viz=0, val_frac=0.34, verbose=False)
    assert stats2["boxes"] == stats["boxes"]
    a = open(os.path.join(out, "meta.jsonl")).read().splitlines()[0]
    b = open(os.path.join(str(tmp_path / "ds2"), "meta.jsonl")).read().splitlines()[0]
    assert json.loads(a)["q"] == json.loads(b)["q"]


# ------------------------------------------------------------------------------------ pressing
def test_press_keys_types_in_sim(capsys):
    from yam_sim.scripts import press_keys

    t0 = time.perf_counter()
    rc = press_keys.main(["ab", "--quiet"])
    out = capsys.readouterr().out
    assert rc == 0, out
    assert "typed:    'ab'" in out
    assert time.perf_counter() - t0 < 30.0


def test_press_with_fingertips_reaches_key():
    # No stylus: press with the closed fingertips (grasp_site). The fingertip pads are wider than a
    # key, so neighbours may go down too; the target key must register.
    from yam_sim.kinematics import Kinematics
    from yam_sim.motion import move_to
    from yam_sim.scripts.press_keys import KeyPresser

    r = sync_robot(objects="keyboard", tool=None)
    kb = Keyboard.from_model(r.model, r.data)
    move_to(r, np.array([0.0, 1.0, 1.0, -0.6, 0.0, 0.0, 0.0]), duration=1.0)
    p = KeyPresser(r, Kinematics(model=r.model), kb, press_site="grasp_site", verbose=False)
    res = p.press("h")
    assert res.ok and "h" in res.events


def test_ee_mass_properties_for_i2rt():
    from yam_sim.camera import ee_mass_properties

    m, _ = load_scene(camera="c920", tool="stylus")
    props = ee_mass_properties(m)
    assert props["ee_mass"] == pytest.approx(0.553219 + 0.162 + 0.014, abs=1e-6)
    assert props["ee_inertia"].shape == (10,)
    base = ee_mass_properties(load_scene(camera="none")[0])
    assert base["ee_mass"] == pytest.approx(0.553219, abs=1e-6)
    assert props["ee_inertia"][1] < base["ee_inertia"][1]  # COM moves towards the camera side (-Y)
