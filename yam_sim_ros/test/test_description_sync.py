"""Fast, ROS-graph-free check that yam_description agrees with the MuJoCo sim (arm_sim/yam_sim).

The xacro copies numbers from arm_sim/yam_sim/models/camera/logitech_*.yml and assembly.py
(STYLUS); config/camera_info_*.yaml copies the intrinsics. This fails if either side changes
without the other.
"""

import os
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
DESC = os.path.normpath(os.path.join(HERE, "..", "..", "yam_description"))


def _rpy_to_mat(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def _urdf(**args):
    import xacro

    doc = xacro.process_file(os.path.join(DESC, "urdf", "yam.urdf.xacro"), mappings={k: str(v) for k, v in args.items()})
    return ET.fromstring(doc.toxml())


def _chain(root, parent, child):
    """Transform parent -> child through fixed/any joints at zero position."""
    joints = {j.find("child").get("link"): j for j in root.findall("joint")}
    T = np.eye(4)
    link = child
    while link != parent:
        j = joints[link]
        o = j.find("origin")
        Tj = np.eye(4)
        Tj[:3, :3] = _rpy_to_mat(*[float(v) for v in o.get("rpy", "0 0 0").split()])
        Tj[:3, 3] = [float(v) for v in o.get("xyz", "0 0 0").split()]
        T = Tj @ T
        link = j.find("parent").get("link")
    return T


@pytest.mark.parametrize("camera", ["c920", "c270"])
def test_camera_mount_matches_yml(camera):
    import mujoco
    from yam_sim.camera import camera_spec

    spec = camera_spec(camera)
    root = _urdf(camera=camera)
    T = _chain(root, "yam_gripper", "yam_wrist_camera_optical_frame")
    R = np.empty(9)
    mujoco.mju_quat2Mat(R, spec.mount_quat)  # yml quat = optical frame in the gripper frame
    np.testing.assert_allclose(T[:3, 3], spec.mount_pos, atol=1e-9)
    np.testing.assert_allclose(T[:3, :3], R.reshape(3, 3), atol=1e-7)
    link = next(l for l in root.findall("link") if l.get("name") == "yam_wrist_camera_link")
    assert float(link.find("inertial/mass").get("value")) == pytest.approx(spec.mass)
    box = [float(v) for v in link.find("visual/geometry/box").get("size").split()]
    w, h, d = spec.body["size"]
    assert box == pytest.approx([d, w, h])


def test_no_camera_and_stylus_frames():
    links = {l.get("name") for l in _urdf(camera="none", tool="none").findall("link")}
    assert "yam_wrist_camera_optical_frame" not in links and "yam_stylus_tip" not in links
    root = _urdf(camera="none", tool="stylus")
    from yam_sim.assembly import STYLUS

    T = _chain(root, "yam_gripper", "yam_stylus_tip")
    np.testing.assert_allclose(T[:3, 3], [0, 0, STYLUS["tip_z"]], atol=1e-12)
    np.testing.assert_allclose(T[:3, :3], [[0, -1, 0], [1, 0, 0], [0, 0, 1]], atol=1e-12)  # Rz(+90 deg)


@pytest.mark.parametrize("camera", ["c920", "c270"])
def test_camera_info_yaml_matches_spec(camera):
    from yam_sim.camera import camera_spec

    spec = camera_spec(camera)
    with open(os.path.join(DESC, "config", f"camera_info_{camera}.yaml")) as f:
        y = yaml.safe_load(f)
    w, h = spec.default_resolution
    assert (y["image_width"], y["image_height"]) == (w, h)
    K = spec.K(w, h)
    np.testing.assert_allclose(np.reshape(y["camera_matrix"]["data"], (3, 3)), K, atol=1e-9)
    np.testing.assert_allclose(np.reshape(y["projection_matrix"]["data"], (3, 4)), np.hstack([K, np.zeros((3, 1))]), atol=1e-9)
    assert y["distortion_model"] == "plumb_bob" and y["distortion_coefficients"]["data"] == [0.0] * 5
    assert y["rectification_matrix"]["data"] == [1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0]
