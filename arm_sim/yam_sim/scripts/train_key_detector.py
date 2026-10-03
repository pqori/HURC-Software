"""Train a YOLO keyboard-key detector on a dataset from ``generate_key_dataset``.

Needs the optional ``train`` pixi environment (PyTorch + Ultralytics)::

    pixi run dataset -- --n 2000 --out data/keys
    pixi run -e train train -- --data data/keys/data.yaml --epochs 50 --imgsz 960
    pixi run -e train train -- --data data/keys/data.yaml --epochs 1 --imgsz 640 --fraction 0.15   # smoke test

The default model is ``yolov8n.pt`` (the smallest pretrained YOLO, ~6 MB, downloaded once by
Ultralytics into ``--weights-dir``). ``--model yolov8n.yaml`` trains from scratch without any
download. The device defaults to Apple ``mps`` when available, else CUDA, else CPU.

Results go to ``--project/--name`` (default ``runs/keys/train``); the best weights are
``.../weights/best.pt``, which ``detect_keys.py`` uses.
"""

from __future__ import annotations

import argparse
import os
import sys
from typing import Optional, Sequence

# Some torchvision ops (e.g. NMS in the conda-forge build) have no Apple-GPU (MPS) kernel; let
# PyTorch run those on the CPU instead of aborting. Must be set before torch is imported.
os.environ.setdefault("PYTORCH_ENABLE_MPS_FALLBACK", "1")

TRAIN_HINT = (
    "Training needs PyTorch and Ultralytics, which live in the optional `train` pixi environment:\n"
    "    pixi install -e train\n"
    "    pixi run -e train train -- --data <dataset>/data.yaml"
)


def pick_device(requested: Optional[str] = None) -> str:
    if requested:
        return requested
    import torch

    if torch.backends.mps.is_available():
        return "mps"
    if torch.cuda.is_available():
        return "0"
    return "cpu"


def main(argv: Optional[Sequence[str]] = None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--data", required=True, help="data.yaml written by generate_key_dataset")
    p.add_argument("--model", default="yolov8n.pt", help="yolov8n.pt (pretrained) | yolo11n.pt | yolov8n.yaml (scratch)")
    p.add_argument("--epochs", type=int, default=50)
    p.add_argument("--imgsz", type=int, default=960, help="training image size (keys are small: >= 640 recommended)")
    p.add_argument("--batch", type=int, default=8)
    p.add_argument("--fraction", type=float, default=1.0, help="use only this fraction of the training images")
    p.add_argument("--device", default=None, help="mps | cpu | 0 (CUDA). Default: auto")
    p.add_argument("--workers", type=int, default=2)
    p.add_argument("--project", default="runs/keys")
    p.add_argument("--name", default="train")
    p.add_argument("--weights-dir", default=None, help="where pretrained weights are downloaded (default: next to --project)")
    p.add_argument("--seed", type=int, default=0)
    args = p.parse_args(argv)
    try:
        from ultralytics import YOLO, settings
    except ImportError as e:
        raise SystemExit(TRAIN_HINT) from e

    project = os.path.abspath(args.project)
    weights_dir = os.path.abspath(args.weights_dir or os.path.join(project, "weights"))
    os.makedirs(weights_dir, exist_ok=True)
    model_arg = args.model
    if model_arg.endswith(".pt") and not os.path.exists(model_arg):
        # Ultralytics downloads named weights into the current directory; keep them in weights_dir.
        model_arg = os.path.join(weights_dir, os.path.basename(model_arg))
        if not os.path.exists(model_arg):
            from ultralytics.utils.downloads import attempt_download_asset

            model_arg = str(attempt_download_asset(model_arg))
    settings.update({"sync": False})  # no analytics
    device = pick_device(args.device)
    print(f"[train] model {model_arg} on {device}, data {args.data}, imgsz {args.imgsz}, epochs {args.epochs}", flush=True)
    model = YOLO(model_arg)
    # Synthetic keys are left/right specific (e.g. lshift vs rshift), so no horizontal flips.
    results = model.train(data=os.path.abspath(args.data), epochs=args.epochs, imgsz=args.imgsz, batch=args.batch,
                          device=device, workers=args.workers, project=project, name=args.name, exist_ok=True,
                          fraction=args.fraction, seed=args.seed, fliplr=0.0, mosaic=1.0, plots=True, verbose=False)
    save_dir = getattr(results, "save_dir", None) or os.path.join(project, args.name)
    best = os.path.join(str(save_dir), "weights", "best.pt")
    metrics = getattr(results, "results_dict", {}) or {}
    print(f"[train] done. best weights: {best}")
    for k in ("metrics/precision(B)", "metrics/recall(B)", "metrics/mAP50(B)", "metrics/mAP50-95(B)"):
        if k in metrics:
            print(f"[train] {k} = {metrics[k]:.4f}")
    print(f"[train] try it: pixi run -e train detect -- --weights {best} --sim")
    return 0


if __name__ == "__main__":
    sys.exit(main())
