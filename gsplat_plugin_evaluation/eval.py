"""Evaluate image quality metrics between a reference folder and an eval folder.

Usage:
    python eval.py <ref_folder> <eval_folder> [--metrics psnr ssim lpips] [--lpips-net alex|vgg]

Images are matched by the trailing 3-digit number in the stem (e.g. abc_001.png matches img_001.png).
"""

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

SUPPORTED_METRICS = ("psnr", "ssim", "lpips")
IMAGE_EXTS = {".png", ".jpg", ".jpeg", ".bmp", ".tiff", ".tif"}


def find_pairs(ref_dir: Path, eval_dir: Path):
    ref_images = sorted(p for p in ref_dir.iterdir() if p.suffix.lower() in IMAGE_EXTS)
    eval_by_digits = {
        p.stem[-3:]: p
        for p in eval_dir.iterdir()
        if p.suffix.lower() in IMAGE_EXTS and p.stem[-3:].isdigit()
    }
    pairs, missing = [], []
    for ref_p in ref_images:
        key = ref_p.stem[-3:]
        if key.isdigit() and key in eval_by_digits:
            pairs.append((ref_p, eval_by_digits[key]))
        else:
            missing.append(ref_p.name)
    return pairs, missing


def load_image(path: Path) -> np.ndarray | None:
    img = cv2.imread(str(path))
    return img


def main():
    parser = argparse.ArgumentParser(description="Compute image quality metrics between two folders")
    parser.add_argument("ref_folder", type=Path, help="Reference image folder")
    parser.add_argument("eval_folder", type=Path, help="Eval image folder")
    parser.add_argument(
        "--metrics",
        nargs="+",
        choices=SUPPORTED_METRICS,
        default=list(SUPPORTED_METRICS),
        metavar="METRIC",
        help=f"Metrics to compute (default: all). Choices: {SUPPORTED_METRICS}",
    )
    parser.add_argument(
        "--lpips-net",
        choices=("alex", "vgg"),
        default="alex",
        help="Backbone for LPIPS (default: alex)",
    )
    args = parser.parse_args()

    if not args.ref_folder.is_dir():
        sys.exit(f"Error: ref_folder '{args.ref_folder}' is not a directory")
    if not args.eval_folder.is_dir():
        sys.exit(f"Error: eval_folder '{args.eval_folder}' is not a directory")

    # Lazy imports so unused metric deps are never loaded
    compute_fns = {}
    if "psnr" in args.metrics:
        from compute_psnr import compute_psnr
        compute_fns["psnr"] = lambda r, e: compute_psnr(r, e)
    if "ssim" in args.metrics:
        from compute_ssim import compute_ssim
        compute_fns["ssim"] = lambda r, e: compute_ssim(r, e)
    if "lpips" in args.metrics:
        from compute_lpips import compute_lpips
        compute_fns["lpips"] = lambda r, e: compute_lpips(r, e, net=args.lpips_net)

    pairs, missing = find_pairs(args.ref_folder, args.eval_folder)
    if not pairs:
        sys.exit("Error: no matching image pairs found")
    if missing:
        snip = missing[:5]
        tail = "..." if len(missing) > 5 else ""
        print(f"Warning: {len(missing)} ref image(s) not found in eval folder: {snip}{tail}")

    accumulated: dict[str, list[float]] = {m: [] for m in args.metrics}

    header = f"{'Image':<20}" + "".join(f"{m.upper():>12}" for m in args.metrics)
    print(header)
    print("-" * len(header))

    for ref_p, ev_p in pairs:
        ref_img = load_image(ref_p)
        ev_img = load_image(ev_p)
        if ref_img is None:
            print(f"Warning: could not read {ref_p.name}, skipping")
            continue
        if ev_img is None:
            print(f"Warning: could not read {ev_p.name}, skipping")
            continue
        if ref_img.shape != ev_img.shape:
            print(f"Warning: shape mismatch for {ref_p.name} ({ref_img.shape} vs {ev_img.shape}), skipping")
            continue

        row = f"{ref_p.name:<20}"
        for metric, fn in compute_fns.items():
            val = fn(ref_img, ev_img)
            accumulated[metric].append(val)
            row += f"{val:>12.4f}"
        print(row)

    print("-" * len(header))
    mean_row = f"{'Mean':<20}"
    for metric in args.metrics:
        vals = accumulated[metric]
        finite = [v for v in vals if v != float("inf")]
        mean_val = np.mean(finite) if finite else float("inf")
        mean_row += f"{mean_val:>12.4f}"
    print(mean_row)


if __name__ == "__main__":
    main()
