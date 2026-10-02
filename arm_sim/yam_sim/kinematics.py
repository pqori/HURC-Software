"""Forward kinematics, Jacobians and damped-least-squares IK on the YAM scene model.

Uses only MuJoCo and NumPy. i2rt's ``Kinematics`` uses ``mink`` instead; this module gives the same
results without the QP-solver dependency. Joint limits are enforced by clamping after each step.
All poses are 4x4 homogeneous matrices in the robot base frame, which is the world frame of the
scene.
"""

from __future__ import annotations

from typing import Optional, Sequence, Tuple, Union

import mujoco
import numpy as np

from yam_sim.assembly import ARM_JOINTS, load_scene


def _rot_log(R: np.ndarray) -> np.ndarray:
    """Axis-angle vector of a rotation matrix."""
    q = np.empty(4)
    mujoco.mju_mat2Quat(q, R.reshape(-1))
    v = np.empty(3)
    mujoco.mju_quat2Vel(v, q, 1.0)
    return v


def pose_from_pos_quat(pos: Sequence[float], quat_wxyz: Sequence[float]) -> np.ndarray:
    T = np.eye(4)
    R = np.empty(9)
    mujoco.mju_quat2Mat(R, np.asarray(quat_wxyz, dtype=float))
    T[:3, :3] = R.reshape(3, 3)
    T[:3, 3] = pos
    return T


def pose_from_pos_rpy(pos: Sequence[float], rpy: Sequence[float]) -> np.ndarray:
    """Pose from position and roll/pitch/yaw (URDF convention ``R = Rz(yaw) Ry(pitch) Rx(roll)``)."""
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    R = np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = pos
    return T


def rpy_from_matrix(R: np.ndarray) -> np.ndarray:
    pitch = np.arcsin(-np.clip(R[2, 0], -1.0, 1.0))
    roll = np.arctan2(R[2, 1], R[2, 2])
    yaw = np.arctan2(R[1, 0], R[0, 0])
    return np.array([roll, pitch, yaw])


def pose_error(T_current: np.ndarray, T_target: np.ndarray) -> Tuple[float, float]:
    """(position error in m, orientation error in rad)."""
    dp = np.linalg.norm(T_target[:3, 3] - T_current[:3, 3])
    dr = np.linalg.norm(_rot_log(T_target[:3, :3] @ T_current[:3, :3].T))
    return float(dp), float(dr)


class Kinematics:
    """FK/IK for the six arm joints. q vectors are the 6 arm joint angles (extra entries are ignored)."""

    def __init__(self, arm: str = "yam", gripper: str = "linear_4310", model: Optional[mujoco.MjModel] = None):
        if model is None:
            model, _ = load_scene(arm, gripper)
        self.model = model
        self.data = mujoco.MjData(model)
        m = model
        self.n = 6
        self._qadr = np.array([m.jnt_qposadr[m.joint(j).id] for j in ARM_JOINTS])
        self._dadr = np.array([m.jnt_dofadr[m.joint(j).id] for j in ARM_JOINTS])
        self.joint_limits = np.array([m.jnt_range[m.joint(j).id] for j in ARM_JOINTS])

    def _frame(self, name: str) -> Tuple[str, int]:
        sid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, name)
        if sid >= 0:
            return "site", sid
        bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
        if bid >= 0:
            return "body", bid
        raise ValueError(f"No site or body named {name!r}")

    def _set_q(self, q: Sequence[float]) -> None:
        q = np.asarray(q, dtype=float)
        self.data.qpos[self._qadr] = q[: self.n]
        mujoco.mj_kinematics(self.model, self.data)
        mujoco.mj_comPos(self.model, self.data)

    def _pose(self, kind: str, idx: int) -> np.ndarray:
        T = np.eye(4)
        if kind == "site":
            T[:3, :3] = self.data.site_xmat[idx].reshape(3, 3)
            T[:3, 3] = self.data.site_xpos[idx]
        else:
            T[:3, :3] = self.data.xmat[idx].reshape(3, 3)
            T[:3, 3] = self.data.xpos[idx]
        return T

    def fk(self, q: Sequence[float], frame: str = "grasp_site") -> np.ndarray:
        """4x4 pose of a site or body (e.g. ``grasp_site``, ``tcp_site``, ``gripper``) at joint angles ``q``."""
        kind, idx = self._frame(frame)
        self._set_q(q)
        return self._pose(kind, idx)

    def jacobian(self, q: Sequence[float], frame: str = "grasp_site") -> np.ndarray:
        """6x6 world-frame Jacobian [linear; angular] of ``frame`` w.r.t. the six arm joints."""
        kind, idx = self._frame(frame)
        self._set_q(q)
        return self._jac(kind, idx)

    def _jac(self, kind: str, idx: int) -> np.ndarray:
        jp = np.zeros((3, self.model.nv))
        jr = np.zeros((3, self.model.nv))
        if kind == "site":
            mujoco.mj_jacSite(self.model, self.data, jp, jr, idx)
        else:
            mujoco.mj_jacBody(self.model, self.data, jp, jr, idx)
        return np.vstack([jp[:, self._dadr], jr[:, self._dadr]])

    def ik(
        self,
        target: np.ndarray,
        frame: str = "grasp_site",
        init_q: Optional[Sequence[float]] = None,
        position_only: bool = False,
        rot_weight: float = 0.3,
        pos_tol: float = 1e-4,
        rot_tol: float = 1e-3,
        max_iters: int = 300,
        damping: float = 1e-3,
        max_step: float = 0.3,
        restarts: int = 4,
        seed: Optional[int] = 0,
    ) -> Tuple[bool, np.ndarray]:
        """Damped least-squares IK.

        Args:
            target: 4x4 target pose, or a 3-vector position (which implies ``position_only``).
            init_q: initial guess (6 arm joints). Defaults to zeros.
            rot_weight: weight of the orientation error (rad) relative to position error (m).
            restarts: extra random restarts if the first attempt does not converge.

        Returns:
            ``(success, q)``. ``q`` is the best 6-joint solution found, within joint limits.
        """
        target = np.asarray(target, dtype=float)
        if target.shape == (3,):
            T = np.eye(4)
            T[:3, 3] = target
            target, position_only = T, True
        kind, idx = self._frame(frame)
        lo, hi = self.joint_limits[:, 0], self.joint_limits[:, 1]
        rng = np.random.default_rng(seed)
        q0 = np.zeros(self.n) if init_q is None else np.asarray(init_q, dtype=float)[: self.n].copy()
        best_q, best_cost = np.clip(q0, lo, hi), np.inf
        for attempt in range(restarts + 1):
            q = np.clip(q0 if attempt == 0 else rng.uniform(lo, hi), lo, hi)
            for _ in range(max_iters):
                self._set_q(q)
                T = self._pose(kind, idx)
                e_p = target[:3, 3] - T[:3, 3]
                e_r = np.zeros(3) if position_only else _rot_log(target[:3, :3] @ T[:3, :3].T)
                if np.linalg.norm(e_p) < pos_tol and (position_only or np.linalg.norm(e_r) < rot_tol):
                    return True, q
                J = self._jac(kind, idx)
                if position_only:
                    J, e = J[:3], e_p
                else:
                    J = np.vstack([J[:3], rot_weight * J[3:]])
                    e = np.concatenate([e_p, rot_weight * e_r])
                dq = J.T @ np.linalg.solve(J @ J.T + damping * np.eye(J.shape[0]), e)
                nrm = np.linalg.norm(dq)
                if nrm > max_step:
                    dq *= max_step / nrm
                q = np.clip(q + dq, lo, hi)
            self._set_q(q)
            T = self._pose(kind, idx)
            dp, dr = pose_error(T, target)
            cost = dp + (0 if position_only else rot_weight * dr)
            if cost < best_cost:
                best_cost, best_q = cost, q.copy()
        return False, best_q


def make_kinematics(robot_or_arm: Union[str, object] = "yam", gripper: str = "linear_4310") -> Kinematics:
    """Kinematics for a robot (uses its model when it is a YamSimRobot) or for an arm/gripper name."""
    model = getattr(robot_or_arm, "model", None)
    if isinstance(model, mujoco.MjModel):
        return Kinematics(model=model)
    info = robot_or_arm.get_robot_info() if hasattr(robot_or_arm, "get_robot_info") else {}
    arm = str(info.get("arm_type", robot_or_arm if isinstance(robot_or_arm, str) else "yam"))
    arm = arm.split(".")[-1].lower() if "ArmType" in arm else arm
    g = info.get("gripper_type", gripper)
    g = str(getattr(g, "value", g))
    return Kinematics(arm, g)
