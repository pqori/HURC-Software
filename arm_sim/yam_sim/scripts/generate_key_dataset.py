"""Generate a labelled wrist-camera dataset of keyboard keys (Ultralytics YOLO + COCO formats).

    pixi run dataset -- --n 200 --out /tmp/key_dataset              # 1280x720, C920, stylus
    pixi run dataset -- --n 50 --camera c270 --width 640 --height 360 --seed 3

For each image the scene is domain-randomised and the arm is posed (by IK on the camera's optical
frame, no physics) so that the wrist camera looks at a random point on the keyboard:

* keyboard position on the table (x 0.28-0.40 m, y +-0.10 m) and yaw (+-30 deg around the default
  "space bar towards the robot" orientation)
* camera height 0.15-0.45 m above the keycaps, tilt 0-35 deg from straight down, coming from
  a random direction around the robot side, plus +-20 deg roll
* light positions, directions and intensities, headlight ambient/diffuse
* keycap colour scheme (dark caps/light tops, all white, all black, grey, ...) with per-key jitter,
  keyboard base colour, a subtle noise texture on the key tops
* floor material (checker, noise, flat, grid) and tint

Labels: every keycap is a class (``data.yaml`` lists the key names, e.g. ``a``, ``space``,
``f1``, ``kp_7``). For each key, the 8 corners of its keycap box are projected through the
camera intrinsics; the convex hull of the projections is clipped to the image, and the box is the
bounding rectangle of the clipped hull. The key is kept only if

* all corners are in front of the camera,
* at least ``--min-visible`` (default 0.3) of the clipped hull area is actually covered by that
  key's geoms in the segmentation render (so keys hidden behind the gripper, the stylus or other
  keys are dropped), and
* the box is at least ``--min-box`` (default 4) pixels on each side.

Outputs in ``--out``::

    images/{train,val}/000000.jpg  labels/{train,val}/000000.txt  (class cx cy w h, normalised)
    data.yaml  annotations.json (COCO, bbox = [x, y, w, h] in pixels)  meta.jsonl (poses per image)
    viz/000000_viz.jpg ...  (the first --viz images with the boxes drawn)

The run is deterministic for a given ``--seed`` and set of options.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from typing import Dict, List, Optional, Sequence, Tuple

import mujoco
import numpy as np

from yam_sim.assembly import load_scene
from yam_sim.camera import OPTICAL_SITE, WristCamera, camera_spec, save_image
from yam_sim.keyboard import DEFAULT_YAW, Keyboard
from yam_sim.kinematics import Kinematics, pose_error

PALETTES = [
    # (cap rgb, top rgb, base rgb)
    ((0.10, 0.10, 0.11), (0.80, 0.80, 0.78), (0.15, 0.15, 0.16)),  # dark caps, light tops (default)
    ((0.92, 0.92, 0.90), (0.97, 0.97, 0.95), (0.85, 0.85, 0.85)),  # all white
    ((0.07, 0.07, 0.08), (0.12, 0.12, 0.13), (0.08, 0.08, 0.09)),  # all black
    ((0.45, 0.46, 0.48), (0.62, 0.63, 0.65), (0.30, 0.30, 0.32)),  # grey
    ((0.80, 0.78, 0.72), (0.88, 0.86, 0.80), (0.55, 0.53, 0.50)),  # beige retro
    ((0.15, 0.17, 0.25), (0.55, 0.60, 0.75), (0.12, 0.13, 0.18)),  # blue-ish
]
FLOORS = ["grid", "floor_noise", "floor_check", "floor_flat"]


# ------------------------------------------------------------------ 2-D geometry helpers
def convex_hull(pts: np.ndarray) -> np.ndarray:
    """Andrew's monotone chain. ``pts`` (N, 2) -> hull vertices counter-clockwise."""
    p = pts[np.lexsort((pts[:, 1], pts[:, 0]))]
    if len(p) <= 2:
        return p

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for q in p:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], q) <= 0:
            lower.pop()
        lower.append(q)
    for q in p[::-1]:
        while len(upper) >= 2 and cross(upper[-2], upper[-1], q) <= 0:
            upper.pop()
        upper.append(q)
    return np.array(lower[:-1] + upper[:-1])


def clip_polygon(poly: np.ndarray, w: float, h: float) -> np.ndarray:
    """Sutherland-Hodgman clip of a convex polygon to the rectangle [0, w] x [0, h]."""
    out = [tuple(p) for p in poly]
    for axis, val, keep_less in ((0, 0.0, False), (0, w, True), (1, 0.0, False), (1, h, True)):
        if not out:
            break
        inp, out = out, []
        for i, cur in enumerate(inp):
            prev = inp[i - 1]
            cin = cur[axis] <= val if keep_less else cur[axis] >= val
            pin = prev[axis] <= val if keep_less else prev[axis] >= val
            if cin:
                if not pin:
                    out.append(_intersect(prev, cur, axis, val))
                out.append(cur)
            elif pin:
                out.append(_intersect(prev, cur, axis, val))
    return np.array(out) if out else np.zeros((0, 2))


def _intersect(a, b, axis, val):
    t = (val - a[axis]) / (b[axis] - a[axis])
    return (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))


def polygon_area(poly: np.ndarray) -> float:
    if len(poly) < 3:
        return 0.0
    x, y = poly[:, 0], poly[:, 1]
    return 0.5 * abs(float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))))


# ------------------------------------------------------------------ labelling
def label_keys(cam: WristCamera, kb: Keyboard, seg: np.ndarray, lut: np.ndarray, min_visible: float = 0.3,
               min_box: float = 4.0) -> List[Dict]:
    """Boxes for every visible key. ``seg`` is a geom-id segmentation, ``lut`` maps geom -> key index."""
    W, H = cam.width, cam.height
    corners = kb.all_key_corners(cam.data)  # (K, 8, 3)
    K = corners.shape[0]
    uv, z = cam.project(corners.reshape(-1, 3), return_depth=True, synced=True)
    uv = uv.reshape(K, 8, 2) + 0.5  # OpenCV pixel-centre coords -> pixel-edge coords ([0, W] spans the image)
    z = z.reshape(K, 8)
    counts = np.bincount(lut[seg.reshape(-1)] + 1, minlength=K + 1)[1:]  # pixels per key (index 0 = background)
    out = []
    for k in np.flatnonzero(counts > 0):
        if np.any(z[k] < 0.02):
            continue
        poly = clip_polygon(convex_hull(uv[k]), W, H)
        area = polygon_area(poly)
        if area < 1.0:
            continue
        frac = counts[k] / area
        x0, y0 = poly.min(0)
        x1, y1 = poly.max(0)
        if frac < min_visible or (x1 - x0) < min_box or (y1 - y0) < min_box:
            continue
        out.append({"cls": int(k), "name": kb.key_names[k], "bbox": [float(x0), float(y0), float(x1), float(y1)],
                    "visible": float(min(frac, 1.0)), "pixels": int(counts[k])})
    return out


# ------------------------------------------------------------------ randomisation
class Randomizer:
    def __init__(self, model: mujoco.MjModel, kb: Keyboard, rng: np.random.Generator):
        self.m, self.kb, self.rng = model, kb, rng
        self.light_ids = [model.light(n).id for n in ("key", "fill")]
        self.light0 = {i: (model.light_pos[i].copy(), model.light_diffuse[i].copy()) for i in self.light_ids}
        self.floor = model.geom("floor").id
        self.floor_mats = [model.material(n).id for n in FLOORS]
        self.n_keys = len(kb.key_names)

    def apply(self) -> Dict:
        m, r = self.m, self.rng
        # keyboard pose
        pos = np.array([r.uniform(0.28, 0.40), r.uniform(-0.10, 0.10), 0.0])
        yaw = DEFAULT_YAW + np.deg2rad(r.uniform(-30, 30))
        self.kb.set_pose(pos, yaw, m)
        # lights
        for i in self.light_ids:
            p0, d0 = self.light0[i]
            p = p0 + r.uniform(-0.6, 0.6, 3) * [1, 1, 0.3]
            m.light_pos[i] = p
            tgt = np.array([pos[0], pos[1], 0.0]) + r.uniform(-0.3, 0.3, 3) * [1, 1, 0]
            d = tgt - p
            m.light_dir[i] = d / np.linalg.norm(d)
            m.light_diffuse[i] = np.clip(d0 * r.uniform(0.4, 1.6), 0, 1) * r.uniform(0.85, 1.0, 3)
        m.vis.headlight.ambient[:] = r.uniform(0.1, 0.45)
        m.vis.headlight.diffuse[:] = r.uniform(0.15, 0.6)
        # keycaps
        cap, top, base = (np.array(c) for c in PALETTES[r.integers(len(PALETTES))])
        cap = np.clip(cap * r.uniform(0.85, 1.15), 0, 1)
        top = np.clip(top * r.uniform(0.85, 1.1), 0, 1)
        jit = r.uniform(0.0, 0.05)
        caps = np.ones((self.n_keys, 4))
        tops = np.ones((self.n_keys, 4))
        noise = r.normal(0, jit, (self.n_keys, 1)) + r.normal(0, jit * 0.3, (self.n_keys, 3))
        caps[:, :3] = np.clip(cap + noise * 0.5, 0, 1)
        tops[:, :3] = np.clip(top + noise, 0, 1)
        if r.random() < 0.3:  # accent colours on a few keys (esc, enter, space ...)
            accent = r.uniform(0.2, 1.0, 3)
            for name in ("esc", "enter", "space", "kp_enter"):
                if name in self.kb.key_names:
                    tops[self.kb.key_names.index(name), :3] = accent
        self.kb.set_colors(caps, tops, np.append(np.clip(base * r.uniform(0.8, 1.2), 0, 1), 1.0))
        # floor
        mat = int(r.integers(len(self.floor_mats)))
        m.geom_matid[self.floor] = self.floor_mats[mat]
        m.geom_rgba[self.floor] = np.append(r.uniform(0.5, 1.0, 3), 1.0)
        return {"keyboard_pos": pos.tolist(), "keyboard_yaw_deg": float(np.rad2deg(yaw)), "floor": FLOORS[mat]}


def look_at_pose(cam_pos: np.ndarray, target: np.ndarray, down_hint: np.ndarray, roll: float) -> np.ndarray:
    """Optical-frame pose at ``cam_pos`` looking at ``target`` (+Z), image-down (+Y) close to ``down_hint``."""
    z = target - cam_pos
    z /= np.linalg.norm(z)
    y = down_hint - np.dot(down_hint, z) * z
    if np.linalg.norm(y) < 1e-6:
        y = np.array([1.0, 0, 0]) - z[0] * z
    y /= np.linalg.norm(y)
    x = np.cross(y, z)
    c, s = np.cos(roll), np.sin(roll)
    x, y = c * x + s * y, -s * x + c * y
    T = np.eye(4)
    T[:3, :3] = np.stack([x, y, z], axis=1)
    T[:3, 3] = cam_pos
    return T


def sample_view(kb: Keyboard, kin: Kinematics, rng: np.random.Generator, robot_geoms: np.ndarray,
                model: mujoco.MjModel, data: mujoco.MjData, arm_qadr: np.ndarray, finger_qadr: List[int],
                finger_range: Optional[Tuple[float, float]], open_gripper: bool, max_tries: int = 25):
    """Pick a camera pose that looks at the keyboard, solve IK, and apply it to ``data``."""
    F = kb.frame()
    half = np.array([kb.width_u, kb.depth_u]) * 0.01905 / 2
    for attempt in range(max_tries):
        local = np.array([rng.uniform(-0.85, 0.85) * half[0], rng.uniform(-0.9, 0.9) * half[1], kb.top_height])
        target = F[:3, :3] @ local + F[:3, 3]
        h = rng.uniform(0.15, 0.45)
        tilt = np.deg2rad(rng.uniform(0, 35))
        towards_robot = -target[:2] / np.linalg.norm(target[:2])
        phi = np.arctan2(towards_robot[1], towards_robot[0]) + np.deg2rad(rng.uniform(-60, 60))
        cam_pos = target + np.array([h * np.tan(tilt) * np.cos(phi), h * np.tan(tilt) * np.sin(phi), h])
        down_hint = np.array([towards_robot[0], towards_robot[1], 0.0])
        T = look_at_pose(cam_pos, target, down_hint, np.deg2rad(rng.uniform(-20, 20)))
        seed = np.array([np.arctan2(target[1], target[0]), 1.0, 1.0, -0.6, 0.0, 0.0])
        ok, q = kin.ik(T, OPTICAL_SITE, init_q=seed, restarts=2, rot_weight=0.5, max_iters=150, seed=int(rng.integers(1 << 30)))
        dp, dr = pose_error(kin.fk(q, OPTICAL_SITE), T)
        if dp > 0.003 or dr > 0.03:
            continue
        data.qpos[:] = model.qpos0
        data.qpos[arm_qadr] = q
        if finger_qadr and finger_range is not None:
            g = rng.uniform(0, 1) if open_gripper else 0.0
            for a in finger_qadr:
                data.qpos[a] = finger_range[0] + g * (finger_range[1] - finger_range[0])
        mujoco.mj_forward(model, data)
        # reject poses where the arm touches the table or the keyboard
        touching = False
        for i in range(data.ncon):
            c = data.contact[i]
            if c.dist < 0.002 and (robot_geoms[c.geom1] or robot_geoms[c.geom2]):
                touching = True
                break
        if touching:
            continue
        return {"q": q.tolist(), "camera_target": target.tolist(), "camera_height": float(h),
                "camera_tilt_deg": float(np.rad2deg(tilt)), "tries": attempt + 1}
    return None


# ------------------------------------------------------------------ output
def draw_boxes(rgb: np.ndarray, labels: List[Dict]) -> np.ndarray:
    from PIL import Image, ImageDraw

    img = Image.fromarray(rgb)
    d = ImageDraw.Draw(img)
    for lab in labels:
        x0, y0, x1, y1 = lab["bbox"]
        col = (0, 255, 0) if lab["visible"] > 0.7 else (255, 200, 0)
        d.rectangle([x0, y0, x1 - 1, y1 - 1], outline=col, width=2)
        d.text((x0 + 3, y0 + 2), lab["name"], fill=(255, 0, 255))
    return np.asarray(img)


def write_data_yaml(out: str, names: Sequence[str]) -> str:
    path = os.path.join(out, "data.yaml")
    with open(path, "w") as f:
        f.write("# Generated by yam_sim.scripts.generate_key_dataset (Ultralytics YOLO format)\n")
        f.write(f"path: {os.path.abspath(out)}\ntrain: images/train\nval: images/val\nnc: {len(names)}\nnames:\n")
        for i, n in enumerate(names):
            f.write(f"  {i}: '{n}'\n")
    return path


def generate(out: str, n: int = 200, seed: int = 0, camera: str = "c920", width: Optional[int] = None,
             height: Optional[int] = None, tool: Optional[str] = "stylus", layout: str = "full", val_frac: float = 0.1,
             min_visible: float = 0.3, min_box: float = 4.0, n_viz: int = 6, jpeg_quality: int = 90,
             verbose: bool = True) -> Dict:
    t_start = time.perf_counter()
    rng = np.random.default_rng(seed)
    spec = camera_spec(camera)
    model, _ = load_scene("yam", "linear_4310", objects="keyboard", camera=spec.key, tool=tool,
                          keyboard={"layout": layout, "textured": True})
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    kb = Keyboard.from_model(model, data)
    kin = Kinematics(model=model)
    cam = WristCamera(model, spec=spec, width=width, height=height, data=data)
    lut = kb.geom_to_key_index()
    dr = Randomizer(model, kb, rng)
    arm_qadr = np.array([model.jnt_qposadr[model.joint(f"joint{i}").id] for i in range(1, 7)])
    finger_qadr = [int(model.jnt_qposadr[model.joint(j).id]) for j in ("joint7", "joint8")]
    finger_range = tuple(model.jnt_range[model.joint("joint7").id])
    # robot geoms: everything that collides as "robot" (contype bit 1)
    robot_geoms = (model.geom_contype & 1).astype(bool)

    for sub in ("images/train", "images/val", "labels/train", "labels/val", "viz"):
        os.makedirs(os.path.join(out, sub), exist_ok=True)
    names = list(kb.key_names)
    data_yaml = write_data_yaml(out, names)
    coco = {"info": {"description": "YAM wrist-camera keyboard keys (synthetic)", "camera": spec.name,
                     "seed": seed, "intrinsics": cam.intrinsics()},
            "images": [], "annotations": [], "categories": [{"id": i + 1, "name": nm, "yolo_id": i} for i, nm in enumerate(names)]}
    meta_f = open(os.path.join(out, "meta.jsonl"), "w")
    n_boxes, ann_id, skipped, t_render = 0, 1, 0, 0.0
    sample_lines = []
    i = 0
    while i < n:
        info = dr.apply()
        mujoco.mj_forward(model, data)
        view = sample_view(kb, kin, rng, robot_geoms, model, data, arm_qadr, finger_qadr, finger_range,
                           open_gripper=(tool is None))
        if view is None:
            skipped += 1
            continue
        t0 = time.perf_counter()
        rgb = cam.render()
        seg = cam.segmentation("geom")
        t_render += time.perf_counter() - t0
        labels = label_keys(cam, kb, seg, lut, min_visible=min_visible, min_box=min_box)
        split = "val" if rng.random() < val_frac else "train"
        stem = f"{i:06d}"
        save_image(os.path.join(out, "images", split, stem + ".jpg"), rgb, quality=jpeg_quality)
        W, H = cam.width, cam.height
        lines = []
        for lab in labels:
            x0, y0, x1, y1 = lab["bbox"]
            lines.append(f"{lab['cls']} {(x0 + x1) / 2 / W:.6f} {(y0 + y1) / 2 / H:.6f} {(x1 - x0) / W:.6f} {(y1 - y0) / H:.6f}")
            coco["annotations"].append({"id": ann_id, "image_id": i, "category_id": lab["cls"] + 1,
                                        "bbox": [x0, y0, x1 - x0, y1 - y0], "area": (x1 - x0) * (y1 - y0), "iscrowd": 0,
                                        "visible_fraction": round(lab["visible"], 3)})
            ann_id += 1
        with open(os.path.join(out, "labels", split, stem + ".txt"), "w") as f:
            f.write("\n".join(lines) + ("\n" if lines else ""))
        if lines and len(sample_lines) < 3:
            sample_lines.append(f"labels/{split}/{stem}.txt: {lines[0]}")
        coco["images"].append({"id": i, "file_name": f"images/{split}/{stem}.jpg", "width": W, "height": H})
        T_cam = cam.pose(synced=True)
        meta_f.write(json.dumps({"image": f"images/{split}/{stem}.jpg", **info, **view, "n_boxes": len(labels),
                                 "camera_pose": np.round(T_cam, 6).tolist()}) + "\n")
        if i < n_viz:
            save_image(os.path.join(out, "viz", f"{stem}_viz.jpg"), draw_boxes(rgb, labels), quality=85)
        n_boxes += len(labels)
        i += 1
        if verbose and (i % 25 == 0 or i == n):
            el = time.perf_counter() - t_start
            print(f"[dataset] {i}/{n} images, {n_boxes / i:.1f} boxes/image, {i / el:.2f} images/s", flush=True)
    meta_f.close()
    with open(os.path.join(out, "annotations.json"), "w") as f:
        json.dump(coco, f)
    cam.close()
    elapsed = time.perf_counter() - t_start
    stats = {"images": n, "boxes": n_boxes, "boxes_per_image": n_boxes / max(1, n), "seconds": elapsed,
             "images_per_s": n / elapsed, "render_s_per_image": t_render / max(1, n), "rejected_views": skipped,
             "data_yaml": data_yaml, "sample_labels": sample_lines, "classes": len(names)}
    return stats


def main(argv: Optional[Sequence[str]] = None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--n", type=int, default=200, help="number of images")
    p.add_argument("--out", default="key_dataset", help="output directory")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--camera", default="c920", help="c920 | c270")
    p.add_argument("--width", type=int, default=None, help="image width (default: the camera's 1280)")
    p.add_argument("--height", type=int, default=None, help="image height (default: 720)")
    p.add_argument("--tool", default="stylus", help="stylus | none (none: random gripper opening)")
    p.add_argument("--layout", default="full", help="full | tkl")
    p.add_argument("--val-frac", type=float, default=0.1)
    p.add_argument("--min-visible", type=float, default=0.3, help="min fraction of the projected key area that must be visible")
    p.add_argument("--min-box", type=float, default=4.0, help="min box width/height in pixels")
    p.add_argument("--viz", type=int, default=6, help="save this many *_viz.jpg with boxes drawn")
    args = p.parse_args(argv)
    if (args.width is None) != (args.height is None):
        p.error("pass both --width and --height")
    tool = None if args.tool.lower() == "none" else args.tool
    stats = generate(args.out, n=args.n, seed=args.seed, camera=args.camera, width=args.width, height=args.height,
                     tool=tool, layout=args.layout, val_frac=args.val_frac, min_visible=args.min_visible,
                     min_box=args.min_box, n_viz=args.viz)
    print(f"[dataset] wrote {stats['images']} images, {stats['boxes']} boxes ({stats['boxes_per_image']:.1f}/image, "
          f"{stats['classes']} classes) to {os.path.abspath(args.out)}")
    print(f"[dataset] {stats['seconds']:.1f} s total, {stats['images_per_s']:.2f} images/s "
          f"(render+segmentation {stats['render_s_per_image'] * 1000:.0f} ms/image), {stats['rejected_views']} rejected view samples")
    for line in stats["sample_labels"]:
        print(f"[dataset] sample {line}")
    print(f"[dataset] train with: pixi run -e train train -- --data {stats['data_yaml']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
