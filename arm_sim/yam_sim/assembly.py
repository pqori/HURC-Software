"""Build the MuJoCo scene: the YAM arm, the chosen gripper, motors, a floor, lights and an IK target.

The arm and gripper are merged exactly the way i2rt's ``combine_arm_and_gripper_xml`` merges them
(adapted from i2rt @ 120c3c8, MIT). The gripper config's ``last_joint_mount`` sets the arm's
``gripper`` mount body pose and joint6 axis. The gripper's own ``<body name="gripper">`` is then
folded into that mount under a ``<frame>``, and its inertial is re-expressed in the mount frame.
So the kinematics and mass properties are the same as the model i2rt uses for gravity
compensation on the real robot.

Added on top of the i2rt model:

* one ``<general>`` actuator per motor, implementing the Damiao MIT-mode law
  ``tau = kp*(q_des - q) + kd*(dq_des - dq) + tau_ff`` with ``ctrl = kp*q_des + kd*dq_des +
  tau_ff``, ``biasprm = (0, -kp, -kd)`` and ``forcerange = +/- motor torque limit``. MuJoCo
  evaluates this law on every physics substep and integrates the kd term implicitly, the way
  the motor's on-board loop runs much faster than the host's 200 Hz command rate. The gripper
  actuator drives a fixed tendon (the mean of the two finger slides) with ``gear = 1/r``, where r
  is metres of finger travel per motor radian, so its length, velocity, gains and force limit
  are all in motor units (rad, rad/s, N m), the same as the real DM4310.
* joint ``armature`` (reflected rotor inertia) and ``frictionloss`` (Coulomb friction)
* a floor, lights, an optional graspable cube, and a ``target`` mocap body for IK dragging
"""

from __future__ import annotations

import hashlib
import os
import tempfile
import xml.etree.ElementTree as ET
from copy import deepcopy
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple, Union

import mujoco
import numpy as np

from yam_sim.config import (
    MOTOR_SPECS,
    load_arm_config,
    load_gripper_config,
)

ARM_JOINTS = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")
GRIPPER_JOINT = "joint7"  # driven finger; joint8 follows through the equality constraint
TARGET_BODY = "target"
TCP_SITE = "tcp_site"

# Box collision pads for the fingertips, in each tip body's frame: (center, half-sizes). MuJoCo
# collides meshes through their convex hulls, and the hull of an L-shaped fingertip fills the
# space between the fingers, so nothing could be grasped. The boxes were fitted (once, offline)
# to the fingertip mesh vertices within 35 mm of grasp_site. The tip meshes stay visible but no
# longer collide.
FINGER_PADS = {
    "linear_4310": {
        "tip_left": ("-0.07312 0.02399 -0.04954", "0.01704 0.02004 0.005"),
        "tip_right": ("-0.02399 -0.07312 -0.04954", "0.02004 0.01704 0.005"),
    },
    "crank_4310": {
        "tip_left": ("0.03934 -0.06881 -0.07063", "0.00439 0.01753 0.01927"),
        "tip_right": ("-0.03934 -0.06881 0.00897", "0.00439 0.01753 0.02047"),
    },
}
GRASP_SITE = "grasp_site"


@dataclass(frozen=True)
class SceneInfo:
    """Metadata describing a built scene (returned alongside the XML)."""

    arm: str
    gripper: str
    arm_joints: Tuple[str, ...]
    gripper_joint: Optional[str]
    motor_types: Tuple[str, ...]  # one per actuated DOF (arm + gripper)
    torque_limits: np.ndarray  # one per actuated DOF, N m (motor side)
    gripper_motor_stroke: Optional[float]  # rad, closed->open
    gripper_slide_range: Optional[Tuple[float, float]]  # metres, MuJoCo joint range
    xml_joint_ranges: np.ndarray  # (6, 2) arm joint ranges from the vendored MJCF


def _floats(text: str) -> np.ndarray:
    return np.array([float(v) for v in text.split()])


def _fmt(values: Sequence[float]) -> str:
    return " ".join(f"{float(v):.17g}" for v in values)


def _compose_pose(outer_pos, outer_quat, inner_pos, inner_quat):
    rotated = np.zeros(3)
    mujoco.mju_rotVecQuat(rotated, inner_pos, outer_quat)
    quat = np.zeros(4)
    mujoco.mju_mulQuat(quat, outer_quat, inner_quat)
    return outer_pos + rotated, quat


def _find_deepest_body(element: ET.Element) -> ET.Element:
    current = element
    while True:
        child_bodies = [c for c in current if c.tag == "body"]
        if not child_bodies:
            return current
        current = child_bodies[0]


def _absolutize_assets(root: ET.Element, xml_path: str) -> None:
    base_dir = os.path.dirname(os.path.abspath(xml_path))
    compiler = root.find("compiler")
    meshdir = compiler.get("meshdir", "") if compiler is not None else ""
    asset = root.find("asset")
    if asset is not None:
        for child in asset:
            f = child.get("file")
            if f and not os.path.isabs(f):
                child.set("file", os.path.abspath(os.path.join(base_dir, meshdir, f)))
    if compiler is not None and "meshdir" in compiler.attrib:
        del compiler.attrib["meshdir"]


def combine_arm_and_gripper(arm: str = "yam", gripper: str = "linear_4310") -> ET.ElementTree:
    """Return the merged arm+gripper MJCF tree (the same result as i2rt's helper), with absolute mesh paths."""
    arm_cfg = load_arm_config(arm)
    grip_cfg = load_gripper_config(gripper, arm)

    arm_tree = ET.parse(arm_cfg.xml_path)
    arm_root = arm_tree.getroot()
    _absolutize_assets(arm_root, arm_cfg.xml_path)

    worldbody = arm_root.find("worldbody")
    mount_body = _find_deepest_body(worldbody)
    mount_body.set("pos", grip_cfg.mount_pos)
    mount_body.set("quat", grip_cfg.mount_quat)
    last_joint = mount_body.find("joint")
    if last_joint is not None:
        last_joint.set("axis", grip_cfg.mount_axis)

    if grip_cfg.xml_path is None:
        return arm_tree

    grip_tree = ET.parse(grip_cfg.xml_path)
    grip_root = grip_tree.getroot()
    _absolutize_assets(grip_root, grip_cfg.xml_path)
    grip_body = grip_root.find(".//body[@name='gripper']")

    arm_asset = arm_root.find("asset")
    grip_asset = grip_root.find("asset")
    if grip_asset is not None:
        if arm_asset is None:
            arm_asset = ET.Element("asset")
            arm_root.insert(list(arm_root).index(worldbody), arm_asset)
        existing = {(c.tag, c.get("name")) for c in arm_asset}
        for child in grip_asset:
            key = (child.tag, child.get("name"))
            if key not in existing:
                arm_asset.append(deepcopy(child))
                existing.add(key)

    if grip_body is not None:
        grip_pos = grip_body.get("pos", "0 0 0")
        grip_quat = grip_body.get("quat", "1 0 0 0")
        frame = ET.Element("frame", {"pos": grip_pos, "quat": grip_quat})
        frame.extend(deepcopy(c) for c in grip_body if c.tag != "inertial")
        mount_body.append(frame)
        grip_inertial = grip_body.find("inertial")
        if grip_inertial is not None:
            placeholder = mount_body.find("inertial")
            if placeholder is not None:
                mount_body.remove(placeholder)
            merged = deepcopy(grip_inertial)
            pos, quat = _compose_pose(
                _floats(grip_pos),
                _floats(grip_quat),
                _floats(grip_inertial.get("pos", "0 0 0")),
                _floats(grip_inertial.get("quat", "1 0 0 0")),
            )
            merged.set("pos", _fmt(pos))
            merged.set("quat", _fmt(quat))
            mount_body.insert(0, merged)

    for section_tag in ("equality", "contact"):
        grip_section = grip_root.find(section_tag)
        if grip_section is None:
            continue
        arm_section = arm_root.find(section_tag)
        if arm_section is None:
            arm_section = ET.SubElement(arm_root, section_tag)
        for child in grip_section:
            arm_section.append(deepcopy(child))
    return arm_tree


def _resolve_per_motor(value, defaults: np.ndarray, name: str) -> np.ndarray:
    if value is None:
        return defaults.astype(float).copy()
    arr = np.broadcast_to(np.asarray(value, dtype=float), defaults.shape).copy()
    if arr.shape != defaults.shape:
        raise ValueError(f"{name} must have {defaults.shape[0]} entries, got {arr.shape}")
    return arr


def build_scene_tree(
    arm: str = "yam",
    gripper: str = "linear_4310",
    *,
    torque_limit: Union[str, float, Sequence[float]] = "peak",
    armature: Optional[Sequence[float]] = None,
    friction: Union[bool, Sequence[float]] = True,
    gripper_friction: float = 0.3,
    timestep: float = 0.001,
    objects: bool = False,
    floor: bool = True,
    self_collision: bool = False,
) -> Tuple[ET.ElementTree, SceneInfo]:
    """Build the scene MJCF tree.

    Args:
        torque_limit: ``"peak"`` (default) or ``"rated"`` for the Damiao datasheet torque, a single
            number for every motor, or one number per motor (arm joints first, then the gripper).
        armature: reflected rotor inertia per arm joint (kg m^2). Defaults to the MotorSpec estimates.
        friction: ``True`` uses the yam_v1.yml ``coulomb_friction`` values as joint ``frictionloss``,
            ``False`` turns friction off, or pass a 6-vector (N m).
        gripper_friction: Coulomb friction of the gripper drive, at the motor (N m).
        timestep: physics timestep (s).
        objects: add a 4 cm cube in front of the robot for grasping tests.
        floor: add a ground plane.
        self_collision: allow contacts between the arm's own links. Off by default because the
            upstream collision meshes are the visual meshes, which are not convex-friendly. Contact with the floor
            and the cube, and between the fingertips and the cube, is always on. When it is on,
            base/link1 contact is excluded because those meshes overlap by about 5 mm at every pose.
    """
    arm_cfg = load_arm_config(arm)
    grip_cfg = load_gripper_config(gripper, arm)
    tree = combine_arm_and_gripper(arm, gripper)
    root = tree.getroot()
    root.set("model", f"{arm}_{gripper}_scene")

    # Arm and gripper geoms default to group 0. Put them in group 2 so the viewer's number keys
    # used for joint selection don't hide the robot. contype/conaffinity bits: 1 = robot,
    # 2 = world. Robot geoms collide only with the world unless self_collision is on.
    for body in root.iter("body"):
        for g in body.findall("geom"):
            g.set("group", "2")
            if g.get("contype") == "0" and g.get("conaffinity") == "0":
                continue
            g.set("contype", "1")
            g.set("conaffinity", "2" if not self_collision else "3")
        for fr in body.findall("frame"):
            for g in fr.iter("geom"):
                g.set("group", "2")
                if g.get("contype") == "0" and g.get("conaffinity") == "0":
                    continue
                g.set("contype", "1")
                g.set("conaffinity", "2" if not self_collision else "3")

    # --- compiler / option / visual -------------------------------------------------------
    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.Element("compiler")
        root.insert(0, compiler)
    compiler.set("angle", "radian")
    compiler.set("autolimits", "true")

    option = ET.Element("option", {"timestep": f"{timestep:g}", "integrator": "implicitfast", "cone": "elliptic"})
    root.insert(1, option)
    visual = ET.fromstring(
        """<visual>
            <global offwidth="1280" offheight="960" azimuth="150" elevation="-20"/>
            <headlight ambient="0.35 0.35 0.35" diffuse="0.5 0.5 0.5" specular="0.1 0.1 0.1"/>
            <quality shadowsize="4096"/>
            <map znear="0.01"/>
            <rgba haze="0.15 0.25 0.35 1"/>
        </visual>"""
    )
    root.insert(2, visual)

    asset = root.find("asset")
    if asset is None:
        asset = ET.Element("asset")
        root.insert(3, asset)
    asset.append(ET.fromstring('<texture name="skybox" type="skybox" builtin="gradient" rgb1="0.35 0.45 0.6" rgb2="0.05 0.07 0.1" width="512" height="512"/>'))
    asset.append(ET.fromstring('<texture name="grid" type="2d" builtin="checker" rgb1="0.22 0.24 0.27" rgb2="0.17 0.19 0.21" width="512" height="512" mark="edge" markrgb="0.3 0.32 0.35"/>'))
    asset.append(ET.fromstring('<material name="grid" texture="grid" texrepeat="8 8" reflectance="0.05"/>'))

    worldbody = root.find("worldbody")
    worldbody.insert(0, ET.fromstring('<light name="key" pos="0.6 -0.6 1.6" dir="-0.4 0.4 -1" directional="false" diffuse="0.7 0.7 0.7" castshadow="true"/>'))
    worldbody.insert(1, ET.fromstring('<light name="fill" pos="-0.8 0.6 1.2" dir="0.5 -0.3 -1" diffuse="0.3 0.3 0.3" castshadow="false"/>'))
    if floor:
        worldbody.insert(2, ET.fromstring('<geom name="floor" type="plane" size="2 2 0.05" material="grid" contype="2" conaffinity="1" group="0"/>'))
    if objects:
        worldbody.append(
            ET.fromstring(
                """<body name="cube" pos="0.42 0 0.02">
                    <freejoint name="cube_free"/>
                    <geom name="cube" type="box" size="0.02 0.02 0.02" mass="0.05" rgba="0.9 0.5 0.1 1"
                          contype="2" conaffinity="3" friction="1.5 0.02 0.001" condim="4" group="0"/>
                </body>"""
            )
        )
    worldbody.append(
        ET.fromstring(
            f"""<body name="{TARGET_BODY}" mocap="true" pos="0.3 0 0.3">
                <geom name="target_geom" type="sphere" size="0.018" rgba="0.2 0.8 0.2 0.35" contype="0" conaffinity="0" group="1"/>
                <site name="target_x" type="cylinder" size="0.003 0.03" pos="0.03 0 0" quat="0.70710678 0 0.70710678 0" rgba="1 0 0 0.8" group="1"/>
                <site name="target_y" type="cylinder" size="0.003 0.03" pos="0 0.03 0" quat="0.70710678 -0.70710678 0 0" rgba="0 1 0 0.8" group="1"/>
                <site name="target_z" type="cylinder" size="0.003 0.03" pos="0 0 0.03" rgba="0 0 1 0.8" group="1"/>
            </body>"""
        )
    )

    pads = FINGER_PADS.get(gripper, {})
    for body in root.iter("body"):
        name = body.get("name")
        if name in pads:
            for g in body.findall("geom"):
                g.set("contype", "0")
                g.set("conaffinity", "0")
            pos, size = pads[name]
            body.append(
                ET.Element(
                    "geom",
                    {
                        "name": f"{name}_pad",
                        "type": "box",
                        "pos": pos,
                        "size": size,
                        "rgba": "0.9 0.2 0.2 0.4",
                        "group": "3",
                        "contype": "1",
                        "conaffinity": "2" if not self_collision else "3",
                        "friction": "1.5 0.02 0.001",
                        "condim": "4",
                        "solref": "0.004 1",
                        "solimp": "0.95 0.99 0.001",
                        "mass": "0",
                    },
                )
            )

    if self_collision:
        contact = root.find("contact")
        if contact is None:
            contact = ET.SubElement(root, "contact")
        contact.append(ET.Element("exclude", {"body1": "base", "body2": "link1"}))

    # --- motor parameters -------------------------------------------------------------------
    n_arm = arm_cfg.n_joints
    motor_types: List[str] = list(arm_cfg.motor_types)
    if grip_cfg.has_motor:
        motor_types.append(grip_cfg.motor_type)
    specs = [MOTOR_SPECS[m] for m in motor_types]
    if isinstance(torque_limit, str):
        if torque_limit not in ("peak", "rated"):
            raise ValueError("torque_limit must be 'peak', 'rated', a number, or a per-motor list")
        limits = np.array([s.peak_torque if torque_limit == "peak" else s.rated_torque for s in specs])
    else:
        limits = _resolve_per_motor(torque_limit, np.zeros(len(specs)), "torque_limit")
    arm_armature = _resolve_per_motor(armature, np.array([s.armature for s in specs[:n_arm]]), "armature")
    if friction is True:
        fric = arm_cfg.coulomb_friction.copy()
    elif friction is False:
        fric = np.zeros(n_arm)
    else:
        fric = _resolve_per_motor(friction, np.zeros(n_arm), "friction")

    joints: Dict[str, ET.Element] = {j.get("name"): j for j in root.iter("joint")}
    xml_ranges = []
    for i, jn in enumerate(ARM_JOINTS[:n_arm]):
        j = joints[jn]
        xml_ranges.append(_floats(j.get("range")))
        j.set("armature", f"{arm_armature[i]:g}")
        j.set("frictionloss", f"{fric[i]:g}")
        # The MJCF's actuatorfrcrange="-10 10" placeholder is replaced by the actuator forcerange.
        j.attrib.pop("actuatorfrcrange", None)

    actuator = root.find("actuator")
    if actuator is None:
        actuator = ET.SubElement(root, "actuator")
    for i, jn in enumerate(ARM_JOINTS[:n_arm]):
        kp, kd = arm_cfg.kp[i], arm_cfg.kd[i]
        actuator.append(
            ET.Element(
                "general",
                {
                    "name": f"{jn}_motor",
                    "joint": jn,
                    "gainprm": "1",
                    "biasprm": f"0 {-kp:g} {-kd:g}",
                    "biastype": "affine",
                    "forcerange": f"{-limits[i]:g} {limits[i]:g}",
                    "ctrllimited": "false",
                },
            )
        )

    stroke = None
    slide_range = None
    if grip_cfg.has_motor:
        gj = joints[GRIPPER_JOINT]
        lo, hi = _floats(gj.get("range"))
        slide_range = (float(lo), float(hi))
        stroke = grip_cfg.motor_stroke_rad()
        r = (hi - lo) / stroke  # metres of finger travel per motor radian
        gear = 1.0 / r
        g_spec = specs[n_arm]
        # The motor drives both fingers through a fixed tendon whose length is their mean travel,
        # so its force splits evenly between them. The equality constraint only has to absorb
        # asymmetric loads. Rotor inertia and drive friction are given at the motor and reflected
        # into slide units, half on each finger joint.
        finger_joints = [GRIPPER_JOINT] + [
            j for j in ("joint8",) if j in joints and joints[j].get("type") == "slide"
        ]
        share = 1.0 / len(finger_joints)
        for jn in finger_joints:
            joints[jn].set("armature", f"{share * g_spec.armature * gear * gear:g}")
            joints[jn].set("frictionloss", f"{share * gripper_friction * gear:g}")
        tendon = root.find("tendon")
        if tendon is None:
            tendon = ET.SubElement(root, "tendon")
        fixed = ET.SubElement(tendon, "fixed", {"name": "gripper_drive"})
        for jn in finger_joints:
            ET.SubElement(fixed, "joint", {"joint": jn, "coef": f"{share:g}"})
        eq = root.find("equality")
        if eq is not None:
            for e in eq:
                e.set("solref", "0.002 1")
                e.set("solimp", "0.99 0.999 0.001")
        actuator.append(
            ET.Element(
                "general",
                {
                    "name": "gripper_motor",
                    "tendon": "gripper_drive",
                    "gear": f"{gear:.10g}",
                    "gainprm": "1",
                    "biasprm": f"0 {-grip_cfg.motor_kp:g} {-grip_cfg.motor_kd:g}",
                    "biastype": "affine",
                    "forcerange": f"{-limits[n_arm]:g} {limits[n_arm]:g}",
                    "ctrllimited": "false",
                },
            )
        )

    info = SceneInfo(
        arm=arm,
        gripper=gripper,
        arm_joints=ARM_JOINTS[:n_arm],
        gripper_joint=GRIPPER_JOINT if grip_cfg.has_motor else None,
        motor_types=tuple(motor_types),
        torque_limits=limits,
        gripper_motor_stroke=stroke,
        gripper_slide_range=slide_range,
        xml_joint_ranges=np.array(xml_ranges),
    )
    return tree, info


def build_scene(arm: str = "yam", gripper: str = "linear_4310", **kwargs) -> str:
    """Return the complete scene MJCF as an XML string. Mesh paths are absolute."""
    tree, _ = build_scene_tree(arm, gripper, **kwargs)
    ET.indent(tree, space="  ")
    return ET.tostring(tree.getroot(), encoding="unicode")


def build_scene_file(arm: str = "yam", gripper: str = "linear_4310", out_dir: Optional[str] = None, **kwargs) -> str:
    """Write the scene MJCF to a file and return its path (a temp dir unless ``out_dir`` is given)."""
    xml = build_scene(arm, gripper, **kwargs)
    digest = hashlib.sha1(xml.encode()).hexdigest()[:10]
    out_dir = out_dir or os.path.join(tempfile.gettempdir(), "yam_sim")
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, f"{arm}_{gripper}_{digest}.xml")
    with open(path, "w") as f:
        f.write(xml)
    return path


def load_scene(arm: str = "yam", gripper: str = "linear_4310", **kwargs) -> Tuple[mujoco.MjModel, SceneInfo]:
    """Build and compile the scene. Returns ``(MjModel, SceneInfo)``."""
    tree, info = build_scene_tree(arm, gripper, **kwargs)
    model = mujoco.MjModel.from_xml_string(ET.tostring(tree.getroot(), encoding="unicode"))
    return model, info
