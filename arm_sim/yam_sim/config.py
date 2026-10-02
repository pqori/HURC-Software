"""Hardware configuration for the YAM arm, grippers and Damiao motors.

The arm and gripper values come from the vendored i2rt YAML files in ``models/config``, so they
match what i2rt's real driver loads. The motor torque limits and rotor inertias are not in i2rt.
They are our own estimates, and every one of them can be overridden.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from functools import lru_cache
from typing import Dict, Optional, Tuple

import numpy as np
import yaml

PACKAGE_DIR = os.path.dirname(os.path.abspath(__file__))
MODELS_DIR = os.path.join(PACKAGE_DIR, "models")
CONFIG_DIR = os.path.join(MODELS_DIR, "config")

# Arm variant -> (model family, hardware revision), same table as i2rt's _ARM_VARIANTS.
# Only "yam" is vendored. To add another variant, copy i2rt/robot_models/arm/<family>/v<N>/ into
# models/arm/<family>/v<N>/, copy i2rt/robots/config/<family>_v<N>.yml into models/config/, and
# make sure each gripper YAML has a last_joint_mount.<family>.v<N> block (the upstream ones do).
ARM_VARIANTS: Dict[str, Tuple[str, int]] = {
    "yam": ("yam", 1),
    "yam_pro": ("yam_pro", 1),
    "yam_ultra": ("yam_ultra", 1),
    "yam_ultra_2": ("yam_ultra", 2),
    "big_yam": ("big_yam", 1),
}

GRIPPERS = ("linear_4310", "crank_4310", "no_gripper")


@dataclass(frozen=True)
class MotorSpec:
    """Damiao DM-series actuator, values at the output shaft.

    ``rated_torque`` and ``peak_torque`` are approximate datasheet figures. ``armature`` is
    our estimate of the rotor inertia reflected through the gearbox (kg m^2). We have not
    measured it, so it is only there to give the simulated joints a believable effective
    inertia. ``pmax``, ``vmax`` and ``tmax`` are the MIT-mode encoding ranges (DM factory
    defaults) and are used only when feedback quantisation is turned on. ``no_load_speed``
    (rad/s at the output) is an approximate figure for the motor's torque-speed curve: the torque
    available in the direction of motion falls linearly to zero at this speed.
    """

    name: str
    rated_torque: float
    peak_torque: float
    armature: float
    no_load_speed: float = 20.0
    pmax: float = 12.5
    vmax: float = 30.0
    tmax: float = 10.0
    kp_max: float = 500.0
    kd_max: float = 5.0


MOTOR_SPECS: Dict[str, MotorSpec] = {
    "DM4310": MotorSpec("DM4310", rated_torque=3.0, peak_torque=7.0, armature=0.002, no_load_speed=20.0, vmax=30.0, tmax=10.0),
    "DM4340": MotorSpec("DM4340", rated_torque=9.0, peak_torque=27.0, armature=0.03, no_load_speed=8.0, vmax=10.0, tmax=28.0),
}


@dataclass(frozen=True)
class ArmConfig:
    family: str
    version: int
    xml_path: str
    motor_types: Tuple[str, ...]
    kp: np.ndarray
    kd: np.ndarray
    gravity_comp_factor: np.ndarray
    grav_comp_kd: np.ndarray
    coulomb_friction: np.ndarray

    @property
    def n_joints(self) -> int:
        return len(self.motor_types)


@dataclass(frozen=True)
class GripperConfig:
    name: str
    xml_path: Optional[str]
    mount_pos: str
    mount_quat: str
    mount_axis: str
    motor_type: Optional[str]
    motor_kp: float
    motor_kd: float
    needs_calibration: bool
    limiter: Optional[dict] = field(default=None)

    @property
    def has_motor(self) -> bool:
        return bool(self.motor_type)

    def motor_stroke_rad(self) -> float:
        """Motor rotation from fully closed to fully open (rad).

        linear_4310 uses the ``motor_stroke`` from its limiter config (6.57 rad). crank_4310 uses
        the crank's close-to-open angle (8 deg to 170 deg). The sim treats both transmissions as
        linear, so the crank gripper's force and speed profile is only approximate.
        """
        lim = self.limiter or {}
        if "motor_stroke" in lim:
            return float(lim["motor_stroke"])
        if "gripper_open_angle" in lim:
            return float(lim["gripper_open_angle"]) - float(lim["gripper_close_angle"])
        return 1.0


def arm_family_version(arm: str) -> Tuple[str, int]:
    if arm not in ARM_VARIANTS:
        raise ValueError(f"Unknown arm {arm!r}. Known variants: {sorted(ARM_VARIANTS)}")
    return ARM_VARIANTS[arm]


@lru_cache(maxsize=None)
def load_arm_config(arm: str = "yam") -> ArmConfig:
    family, version = arm_family_version(arm)
    xml_path = os.path.join(MODELS_DIR, "arm", family, f"v{version}", f"{family}.xml")
    cfg_path = os.path.join(CONFIG_DIR, f"{family}_v{version}.yml")
    if not (os.path.isfile(xml_path) and os.path.isfile(cfg_path)):
        raise NotImplementedError(
            f"Arm {arm!r} is a known i2rt variant but its model is not vendored in yam_sim "
            f"(expected {xml_path} and {cfg_path}). Copy them from the i2rt repo to enable it."
        )
    with open(cfg_path) as f:
        raw = yaml.safe_load(f)
    return ArmConfig(
        family=family,
        version=version,
        xml_path=xml_path,
        motor_types=tuple(m[1] for m in raw["motor_list"]),
        kp=np.array(raw["kp"], dtype=float),
        kd=np.array(raw["kd"], dtype=float),
        gravity_comp_factor=np.array(raw["gravity_comp_factor"], dtype=float),
        grav_comp_kd=np.array(raw["grav_comp_kd"], dtype=float),
        coulomb_friction=np.array(raw["coulomb_friction"], dtype=float),
    )


@lru_cache(maxsize=None)
def load_gripper_config(gripper: str = "linear_4310", arm: str = "yam") -> GripperConfig:
    if gripper not in GRIPPERS:
        raise ValueError(f"Unknown or unvendored gripper {gripper!r}. Available: {list(GRIPPERS)}")
    family, version = arm_family_version(arm)
    with open(os.path.join(CONFIG_DIR, f"{gripper}.yml")) as f:
        raw = yaml.safe_load(f)
    mounts = raw.get("last_joint_mount") or raw["joint6_mount"]
    if "pos" not in mounts:
        mounts = mounts[family]
        if "pos" not in mounts:
            mounts = mounts[f"v{version}"]
    xml_path = os.path.join(MODELS_DIR, "gripper", gripper, f"{gripper}.xml")
    return GripperConfig(
        name=gripper,
        xml_path=xml_path if os.path.isfile(xml_path) else None,
        mount_pos=mounts["pos"],
        mount_quat=mounts["quat"],
        mount_axis=mounts["axis"],
        motor_type=raw.get("motor_type") or None,
        motor_kp=float(raw.get("motor_kp", -1.0)),
        motor_kd=float(raw.get("motor_kd", -1.0)),
        needs_calibration=bool(raw.get("needs_calibration", False)),
        limiter=raw.get("limiter"),
    )
