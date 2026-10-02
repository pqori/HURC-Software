"""Render a few arm poses offscreen to PNG (no window needed).

    pixi run render                      # writes arm_sim/docs/img/*.png
    pixi run render --out /tmp/imgs --width 800 --height 600

Each pose is reached by PD control in the physics sim (not by teleporting), so the pictures show
the settled state, including any gravity sag.
"""

from __future__ import annotations

import argparse
import os
import struct
import sys
import zlib

import mujoco
import numpy as np

from yam_sim.kinematics import Kinematics
from yam_sim.robot import YamSimRobot

DEFAULT_OUT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "docs", "img"))


def write_png(path: str, rgb: np.ndarray) -> None:
    """Minimal RGB8 PNG encoder (no Pillow dependency)."""
    h, w, _ = rgb.shape
    raw = b"".join(b"\x00" + rgb[y].tobytes() for y in range(h))

    def chunk(tag: bytes, data: bytes) -> bytes:
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)

    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n")
        f.write(chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0)))
        f.write(chunk(b"IDAT", zlib.compress(raw, 9)))
        f.write(chunk(b"IEND", b""))


def poses(k: Kinematics):
    R_down = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1.0]])
    T = np.eye(4)
    T[:3, :3] = R_down
    T[:3, 3] = [0.42, 0.0, 0.03]
    _, q_grasp = k.ik(T, init_q=[0, 2.2, 1.7, -1.1, 0, 1.57])
    return [
        ("home", [0, 0, 0, 0, 0, 0], 0.0, (150, -20, 0.9, [0.1, 0, 0.15])),
        ("ready", [0.0, 1.0, 1.0, -0.3, 0.0, 0.0], 1.0, (150, -20, 1.1, [0.2, 0, 0.2])),
        ("reach_left", [0.9, 1.6, 1.0, 0.2, 0.6, 0.0], 0.6, (120, -25, 1.3, [0.2, 0.15, 0.25])),
        ("grasp_cube", list(q_grasp), 0.55, (125, -18, 1.15, [0.28, 0, 0.18])),
    ]


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--out", default=DEFAULT_OUT)
    p.add_argument("--gripper", default="linear_4310")
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    args = p.parse_args(argv)
    os.makedirs(args.out, exist_ok=True)

    robot = YamSimRobot(gripper=args.gripper, start_thread=False, zero_gravity_mode=False, objects=True)
    model = robot.model
    k = Kinematics(model=model)
    renderer = mujoco.Renderer(model, height=args.height, width=args.width)
    cam = mujoco.MjvCamera()
    opt = mujoco.MjvOption()
    opt.geomgroup[:] = 0
    opt.geomgroup[0] = opt.geomgroup[2] = 1  # floor/objects and robot; hide the IK target and collision pads
    opt.sitegroup[:] = 0
    written = []
    for name, q, g, (az, el, dist, look) in poses(k):
        robot.reset(np.append(q, g) if robot.num_dofs() == 7 else q, hold=True)
        robot.step_for(1.0)
        cam.azimuth, cam.elevation, cam.distance, cam.lookat[:] = az, el, dist, look
        with robot.lock:
            renderer.update_scene(robot.data, camera=cam, scene_option=opt)
            img = renderer.render()
        path = os.path.join(args.out, f"yam_{name}.png")
        write_png(path, img)
        written.append((path, os.path.getsize(path)))
    renderer.close()
    for path, size in written:
        print(f"{path}  {size / 1024:.0f} KB")
    return 0


if __name__ == "__main__":
    sys.exit(main())
