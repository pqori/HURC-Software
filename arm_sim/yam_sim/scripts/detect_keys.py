"""Run a trained key detector on wrist-camera frames (sim or a real webcam) and draw the detections.

Needs the optional ``train`` environment (PyTorch + Ultralytics + OpenCV)::

    pixi run -e train detect -- --weights runs/keys/train/weights/best.pt --sim            # sim frames
    pixi run -e train detect -- --weights best.pt --images data/keys/images/val --out det   # a folder
    pixi run -e train detect -- --weights best.pt --webcam 0 --show                        # real webcam

``--sim`` renders the wrist camera of the simulated arm in a few poses above the keyboard (the
same randomisation as the dataset generator, different seed) and, because the sim knows the truth,
also prints how many ground-truth keys were matched (IoU >= 0.5, same class).
With ``--webcam`` frames come from :class:`yam_sim.camera.RealWebcam` (OpenCV). ``--show`` opens
a live OpenCV window (needs a desktop session; q quits); otherwise annotated frames are written to
``--out``.
"""

from __future__ import annotations

import argparse
import os
import sys
from typing import Dict, List, Optional, Sequence

# Some torchvision ops (e.g. NMS in the conda-forge build) have no Apple-GPU (MPS) kernel; let
# PyTorch run those on the CPU instead of aborting. Must be set before torch is imported.
os.environ.setdefault("PYTORCH_ENABLE_MPS_FALLBACK", "1")

import numpy as np


def detections_from_result(res, names) -> List[Dict]:
    out = []
    boxes = res.boxes
    if boxes is None:
        return out
    xyxy = boxes.xyxy.cpu().numpy()
    cls = boxes.cls.cpu().numpy().astype(int)
    conf = boxes.conf.cpu().numpy()
    for b, c, s in zip(xyxy, cls, conf):
        out.append({"bbox": b.tolist(), "cls": int(c), "name": names[int(c)], "conf": float(s)})
    return out


def draw(rgb: np.ndarray, dets: List[Dict]) -> np.ndarray:
    from PIL import Image, ImageDraw

    img = Image.fromarray(rgb)
    d = ImageDraw.Draw(img)
    for det in dets:
        x0, y0, x1, y1 = det["bbox"]
        d.rectangle([x0, y0, x1, y1], outline=(0, 255, 255), width=2)
        d.text((x0 + 2, y0 + 1), f"{det['name']} {det['conf']:.2f}", fill=(255, 255, 0))
    return np.asarray(img)


def iou(a, b) -> float:
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    u = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / u if u > 0 else 0.0


def sim_frames(n: int, seed: int, camera: str, width: Optional[int], height: Optional[int]):
    """Yield (rgb, ground-truth labels) from randomised sim views (as in the dataset generator)."""
    import mujoco

    from yam_sim.assembly import load_scene
    from yam_sim.camera import WristCamera
    from yam_sim.keyboard import Keyboard
    from yam_sim.kinematics import Kinematics
    from yam_sim.scripts.generate_key_dataset import Randomizer, label_keys, sample_view

    rng = np.random.default_rng(seed)
    model, _ = load_scene(objects="keyboard", camera=camera, tool="stylus", keyboard={"textured": True})
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    kb = Keyboard.from_model(model, data)
    kin = Kinematics(model=model)
    cam = WristCamera(model, data=data, width=width, height=height)
    lut = kb.geom_to_key_index()
    dr = Randomizer(model, kb, rng)
    arm_qadr = np.array([model.jnt_qposadr[model.joint(f"joint{i}").id] for i in range(1, 7)])
    fq = [int(model.jnt_qposadr[model.joint(j).id]) for j in ("joint7", "joint8")]
    fr = tuple(model.jnt_range[model.joint("joint7").id])
    robot_geoms = (model.geom_contype & 1).astype(bool)
    made = 0
    while made < n:
        dr.apply()
        mujoco.mj_forward(model, data)
        if sample_view(kb, kin, rng, robot_geoms, model, data, arm_qadr, fq, fr, open_gripper=False) is None:
            continue
        rgb = cam.render()
        gt = label_keys(cam, kb, cam.segmentation(), lut)
        made += 1
        yield rgb, gt
    cam.close()


def main(argv: Optional[Sequence[str]] = None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--weights", required=True, help="trained weights (best.pt)")
    src = p.add_mutually_exclusive_group()
    src.add_argument("--sim", action="store_true", help="render sim wrist-camera frames (default)")
    src.add_argument("--webcam", default=None, help="real webcam index, e.g. 0")
    src.add_argument("--images", default=None, help="folder of images")
    p.add_argument("--n", type=int, default=8, help="number of frames (sim/webcam)")
    p.add_argument("--camera", default="c920")
    p.add_argument("--width", type=int, default=None)
    p.add_argument("--height", type=int, default=None)
    p.add_argument("--imgsz", type=int, default=960)
    p.add_argument("--conf", type=float, default=0.25)
    p.add_argument("--seed", type=int, default=12345)
    p.add_argument("--device", default=None)
    p.add_argument("--out", default="runs/keys/detect")
    p.add_argument("--show", action="store_true", help="(webcam) live OpenCV window")
    args = p.parse_args(argv)
    try:
        from ultralytics import YOLO
    except ImportError as e:
        from yam_sim.scripts.train_key_detector import TRAIN_HINT

        raise SystemExit(TRAIN_HINT) from e
    from yam_sim.camera import save_image
    from yam_sim.scripts.train_key_detector import pick_device

    model = YOLO(args.weights)
    names = model.names
    device = pick_device(args.device)
    os.makedirs(args.out, exist_ok=True)

    def run(rgb):
        # Ultralytics treats numpy input as BGR (OpenCV); flip the channels of our RGB frames.
        res = model.predict(rgb[:, :, ::-1], imgsz=args.imgsz, conf=args.conf, device=device, verbose=False)[0]
        return detections_from_result(res, names)

    if args.webcam is not None:
        from yam_sim.camera import RealWebcam

        idx = int(args.webcam) if str(args.webcam).isdigit() else args.webcam
        cam = RealWebcam(idx, spec=args.camera, width=args.width, height=args.height)
        try:
            i = 0
            while args.show or i < args.n:
                rgb = cam.render()
                vis = draw(rgb, run(rgb))
                if args.show:
                    import cv2

                    cv2.imshow("keys", vis[:, :, ::-1])
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break
                else:
                    save_image(os.path.join(args.out, f"webcam_{i:04d}.jpg"), vis)
                i += 1
        finally:
            cam.close()
        print(f"[detect] wrote annotated webcam frames to {args.out}")
        return 0

    if args.images:
        from PIL import Image

        files = sorted(f for f in os.listdir(args.images) if f.lower().endswith((".jpg", ".jpeg", ".png")))
        for f in files[: args.n]:
            rgb = np.asarray(Image.open(os.path.join(args.images, f)).convert("RGB"))
            dets = run(rgb)
            save_image(os.path.join(args.out, f"det_{f.rsplit('.', 1)[0]}.jpg"), draw(rgb, dets))
            print(f"[detect] {f}: {len(dets)} keys")
        return 0

    tot_gt = tot_match = tot_det = 0
    for i, (rgb, gt) in enumerate(sim_frames(args.n, args.seed, args.camera, args.width, args.height)):
        dets = run(rgb)
        used = set()
        match = 0
        for g in gt:
            best, bj = 0.0, -1
            for j, d in enumerate(dets):
                if j in used or d["cls"] != g["cls"]:
                    continue
                v = iou(g["bbox"], d["bbox"])
                if v > best:
                    best, bj = v, j
            if best >= 0.5:
                used.add(bj)
                match += 1
        tot_gt += len(gt)
        tot_match += match
        tot_det += len(dets)
        save_image(os.path.join(args.out, f"sim_{i:03d}.jpg"), draw(rgb, dets))
        print(f"[detect] sim frame {i}: {len(dets)} detections, {match}/{len(gt)} ground-truth keys matched")
    print(f"[detect] total: {tot_match}/{tot_gt} keys found (recall {tot_match / max(1, tot_gt):.2f}), "
          f"{tot_det} detections (precision {tot_match / max(1, tot_det):.2f}); frames in {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
