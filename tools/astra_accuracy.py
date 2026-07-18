#!/usr/bin/env python3
"""
Measure the Astra Z-coordinate error to fill the report result table.
=====================================================================
Procedure (semi-automatic): for each true distance (measured with a ruler), place a flat object
facing the camera, press Enter; the script takes the median Z of the center region over N frames
and computes the error in mm + %. Prints a Markdown table to paste straight into the results chapter.

Examples:
  python tools/astra_accuracy.py 200 500 1000
  python tools/astra_accuracy.py 200 500 1000 --samples 60 --patch 40
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from layer1_vision.cameras.astra_openni import AstraCamera   # noqa: E402


def measure_center_z(cam, samples: int, patch: int) -> tuple:
    """Take the median Z of a patch x patch region around the frame center, over `samples` frames."""
    zs = []
    for _ in range(samples):
        _, depth = cam.read()
        h, w = depth.shape
        cy, cx = h // 2, w // 2
        r = patch // 2
        roi = depth[cy - r:cy + r, cx - r:cx + r]
        valid = roi[(roi > 0) & np.isfinite(roi)]
        if valid.size:
            zs.append(float(np.median(valid)))
    if not zs:
        return float("nan"), float("nan")
    arr = np.array(zs)
    return float(arr.mean()), float(arr.std())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("distances", type=float, nargs="+",
                    help="the true distances (mm), e.g.: 200 500 1000")
    ap.add_argument("--samples", type=int, default=60, help="frames sampled per point")
    ap.add_argument("--patch", type=int, default=40, help="center square side (pixels)")
    args = ap.parse_args()

    cam = AstraCamera(mode="depth")
    rows = []
    try:
        for gt in args.distances:
            input(f"\n>> Place a flat object facing the camera at {gt:.0f} mm (ruler), "
                  f"keep it in the CENTER of the frame, then press Enter...")
            mean_z, std_z = measure_center_z(cam, args.samples, args.patch)
            err = mean_z - gt
            err_pct = err / gt * 100 if gt else float("nan")
            rows.append((gt, mean_z, std_z, err, err_pct))
            print(f"   measured Z = {mean_z:.1f} mm (std {std_z:.1f}) | "
                  f"error {err:+.1f} mm ({err_pct:+.1f}%)", flush=True)
    finally:
        cam.close()

    print("\n\n=== Z-coordinate error table (paste into the report) ===\n")
    print("| True distance (mm) | Z measured (mm) | Std (mm) | Error (mm) | Error (%) |")
    print("|-------------------:|----------------:|---------:|-----------:|----------:|")
    for gt, mz, sz, err, ep in rows:
        print(f"| {gt:.0f} | {mz:.1f} | {sz:.1f} | {err:+.1f} | {ep:+.1f} |")


if __name__ == "__main__":
    main()
