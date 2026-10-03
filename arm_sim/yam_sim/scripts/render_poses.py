"""Render a few arm poses offscreen to PNG (no window needed).

    pixi run render                      # writes arm_sim/docs/img/*.png
    pixi run render --out /tmp/imgs --width 800 --height 600
    pixi run render --keyboard           # also the keyboard/stylus scene, a wrist-camera view and a labelled frame

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


def render_keyboard(out: str, width: int, height: int) -> list:
    """Keyboard + stylus scene shot, a wrist-camera view and a labelled dataset frame."""
    from yam_sim.camera import OPTICAL_SITE, WristCamera, save_image
    from yam_sim.keyboard import Keyboard
    from yam_sim.scripts.generate_key_dataset import draw_boxes, label_keys, look_at_pose
    from yam_sim.scripts.press_keys import press_rotation

    robot = YamSimRobot(start_thread=False, zero_gravity_mode=False, objects="keyboard", tool="stylus",
                        keyboard={"textured": False})
    model = robot.model
    k = Kinematics(model=model)
    kb = Keyboard.from_model(model, robot.data)
    opt = mujoco.MjvOption()
    opt.geomgroup[:] = 0
    opt.geomgroup[0] = opt.geomgroup[2] = 1
    opt.sitegroup[:] = 0
    written = []
    # 1) stylus pressing "g"
    top = kb.key_pose("g")[:3, 3]
    T = np.eye(4)
    T[:3, :3] = press_rotation(top[:2])
    T[:3, 3] = top + [0, 0, 0.004]
    _, q = k.ik(T, "stylus_tip", init_q=[0, 1, 1, -0.6, 0, 0], restarts=4)
    robot.reset(np.append(q, 0.0), hold=True)
    robot.step_for(1.0)
    renderer = mujoco.Renderer(model, height=height, width=width)
    cam = mujoco.MjvCamera()
    cam.azimuth, cam.elevation, cam.distance, cam.lookat[:] = 140, -28, 0.95, [0.28, 0.02, 0.12]
    renderer.update_scene(robot.data, camera=cam, scene_option=opt)
    path = os.path.join(out, "yam_keyboard_stylus.png")
    write_png(path, renderer.render())
    written.append(path)
    renderer.close()
    # 2) wrist-camera view from 0.28 m above the keyboard, looking slightly forward
    target = kb.key_pose("t")[:3, 3]
    Tc = look_at_pose(target + [-0.08, 0.0, 0.28], target, np.array([-1.0, 0, 0]), 0.0)
    _, q = k.ik(Tc, OPTICAL_SITE, init_q=[0, 1, 1, -0.6, 0, 0], restarts=4)
    robot.reset(np.append(q, 0.0), hold=True)
    with WristCamera(robot) as wc:
        rgb = wc.render()
        path = os.path.join(out, "yam_wrist_camera.jpg")
        save_image(path, rgb, quality=85)
        written.append(path)
        # 3) the same frame with dataset labels
        seg = wc.segmentation()
        labels = label_keys(wc, kb, seg, kb.geom_to_key_index())
        path = os.path.join(out, "yam_key_labels.jpg")
        save_image(path, draw_boxes(rgb, labels), quality=80)
        written.append(path)
    robot.close()
    return written


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--out", default=DEFAULT_OUT)
    p.add_argument("--gripper", default="linear_4310")
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--keyboard", action="store_true", help="also render the keyboard / wrist-camera images")
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
    if args.keyboard:
        written += [(pth, os.path.getsize(pth)) for pth in render_keyboard(args.out, args.width, args.height)]
    for path, size in written:
        print(f"{path}  {size / 1024:.0f} KB")
    return 0


if __name__ == "__main__":
    sys.exit(main())
