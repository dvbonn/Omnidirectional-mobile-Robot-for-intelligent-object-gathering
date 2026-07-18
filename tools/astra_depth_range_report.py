#!/usr/bin/env python3
"""
T0 SPIKE - Measure the Astra's real depth range + FOV to decide SLAM viability.
==============================================================================
Notes recorded an observed depth range of only ~22-1022mm (~1m). If that is TRUE, with
a ~58 deg horizontal FOV (not 360 deg), the Astra is almost certainly NOT enough to map a room
-> we would have to propose adding a 2D LiDAR before investing in the slam_toolbox + NAV2 stack.

This tool only needs the Astra camera (mode="depth"), NOT the robot base.

Two modes:
  probe (default) : stream depth, aggregate whole-frame stats -> reliable max range
                    (p95/p99/max), %valid pixels, histogram, geometric FOV,
                    and a go/no-go VERDICT.
                    -> Point the camera at the FARTHEST/most OPEN scene possible (corridor,
                      large room) to reveal the sensor's true maximum range.
  calib           : measure error. Place a flat surface (wall) filling the frame center at a
                    KNOWN distance, enter the number -> the tool reports measured median vs
                    truth. Repeat for several points (0.5/1/2/3/4m). Empty Enter to finish.

How to run:
    python tools/astra_depth_range_report.py                 # probe 15s
    python tools/astra_depth_range_report.py --duration 30
    python tools/astra_depth_range_report.py --mode calib
    python tools/astra_depth_range_report.py --frames 300 --output docs/astra_depth_range.md

Results are printed to the console and written to a Markdown report (default docs/astra_depth_range.md).
"""

import argparse
import math
import os
import sys
import time
from datetime import datetime

import numpy as np

# Allow importing the layer1_vision package when run from the repo root
_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from layer1_vision.cameras.astra_openni import AstraCamera  # noqa: E402

# Max range of interest when building the histogram (mm). Values above are clipped to the last bin.
MAX_MM = 10_000

# Go/no-go verdict thresholds (per PLAN_SLAM_NAV2.md, based on p99 = robust max range)
VERDICT_GOOD_MM = 3_000      # >=3m -> good enough, continue the plan
VERDICT_MARGINAL_MM = 1_500  # 1.5-3m -> ok for a small area, high risk


# ================================================================================
# Utilities
# ================================================================================
def load_intrinsics():
    """Read fx/fy/cx/cy from config (to compute the geometric FOV). Returns a dict, or None."""
    path = os.path.join(_REPO_ROOT, "config", "astra_intrinsics.json")
    try:
        import json
        with open(path) as f:
            return json.load(f)
    except Exception:
        return None


def fov_deg(half_extent_px, focal_px):
    """FOV (degrees) for one axis: 2*atan( (size/2) / focal )."""
    return math.degrees(2.0 * math.atan(half_extent_px / focal_px))


def percentile_from_hist(hist, cum, total, p):
    """Compute the p-th percentile (0-100) from a 1mm/bin histogram. Returns mm."""
    if total == 0:
        return 0
    target = p / 100.0 * total
    idx = int(np.searchsorted(cum, target))
    return min(idx, len(hist) - 1)


def text_histogram(hist, bin_mm=500, width=46):
    """Draw a text histogram binned by bin_mm (mm)."""
    nbins = (len(hist) + bin_mm - 1) // bin_mm
    binned = np.add.reduceat(hist, np.arange(0, len(hist), bin_mm))
    peak = binned.max() if binned.size else 0
    lines = []
    for i in range(min(nbins, binned.size)):
        cnt = int(binned[i])
        if cnt == 0 and i > 0 and binned[i:].sum() == 0:
            break  # no more far data -> stop for brevity
        lo = i * bin_mm
        hi = lo + bin_mm
        bar = "#" * int(width * cnt / peak) if peak else ""
        lines.append(f"  {lo:5d}-{hi:5d}mm | {bar} {cnt}")
    return "\n".join(lines)


# ================================================================================
# PROBE mode - measure range + FOV + %valid
# ================================================================================
def run_probe(cam, args, intr):
    print(">> PROBE: point the camera at the FARTHEST/most OPEN scene (corridor, large room).")
    print(f">> Collecting {('%ds' % args.duration) if not args.frames else ('%d frames' % args.frames)} "
          f"(dropping the first {args.warmup} warm-up frames)...\n", flush=True)

    hist = np.zeros(MAX_MM + 1, dtype=np.int64)
    n_frames = 0
    valid_sum = 0           # running total of valid pixels (depth>0)
    pixel_sum = 0           # running total of pixels
    col_valid_any = None    # which columns (in the center band) ever had valid depth
    shape = None
    read_errors = 0

    t0 = time.time()
    i = 0
    while True:
        # Stop condition: by --frames or --duration
        if args.frames and n_frames >= args.frames:
            break
        if not args.frames and (time.time() - t0) >= args.duration:
            break
        try:
            _, depth = cam.read()
        except RuntimeError:
            read_errors += 1
            if read_errors > 30:
                raise
            continue
        i += 1
        if i <= args.warmup:
            continue

        if shape is None:
            shape = depth.shape
            h, w = shape
            col_valid_any = np.zeros(w, dtype=bool)

        h, w = depth.shape
        valid_mask = depth > 0
        valid_sum += int(valid_mask.sum())
        pixel_sum += depth.size

        v = depth[valid_mask].astype(np.int64)
        if v.size:
            np.clip(v, 0, MAX_MM, out=v)
            hist += np.bincount(v, minlength=MAX_MM + 1)

        # Vertical center band (+-15% around the middle row) to estimate horizontal coverage
        band = slice(int(h * 0.35), int(h * 0.65))
        col_valid_any |= valid_mask[band, :].any(axis=0)

        n_frames += 1
        if n_frames % 30 == 0:
            print(f"   ...{n_frames} frames, valid~{100*valid_sum/max(pixel_sum,1):.0f}%", flush=True)

    dt = time.time() - t0
    if n_frames == 0:
        raise RuntimeError("Collected no valid frames (warm-up too large or continuous read errors).")

    total_valid = int(hist.sum())
    cum = np.cumsum(hist)

    stats = {
        "frames": n_frames,
        "fps": n_frames / dt if dt else 0.0,
        "shape": shape,
        "valid_ratio": valid_sum / max(pixel_sum, 1),
        "read_errors": read_errors,
        "min_mm": int(np.argmax(hist > 0)) if total_valid else 0,
        "p05": percentile_from_hist(hist, cum, total_valid, 5),
        "p50": percentile_from_hist(hist, cum, total_valid, 50),
        "p95": percentile_from_hist(hist, cum, total_valid, 95),
        "p99": percentile_from_hist(hist, cum, total_valid, 99),
        "p999": percentile_from_hist(hist, cum, total_valid, 99.9),
        "max_mm": int(len(hist) - 1 - np.argmax(hist[::-1] > 0)) if total_valid else 0,
    }

    # Usable horizontal coverage + geometric FOV
    if col_valid_any is not None and col_valid_any.size:
        stats["usable_col_ratio"] = float(col_valid_any.mean())
        w = col_valid_any.size
    else:
        stats["usable_col_ratio"] = 0.0
        w = shape[1]
    h = shape[0]

    if intr:
        fx = intr.get("fx", 570.3422)
        fy = intr.get("fy", 570.3422)
        stats["fov_h_deg"] = fov_deg(w / 2.0, fx)
        stats["fov_v_deg"] = fov_deg(h / 2.0, fy)
    else:
        stats["fov_h_deg"] = stats["fov_v_deg"] = None

    stats["hist_text"] = text_histogram(hist, bin_mm=args.bins)

    # Number of pixel-readings with depth > 1.1m (far) - if =0 while the scene has far areas -> suspect a cap
    stats["beyond_1100"] = int(hist[1100:].sum())
    # Suspected firmware cap ~1022/1023mm: a cluster near 1022 + absolutely nothing >1.1m
    near_cap = int(hist[1000:1024].sum())
    stats["suspect_cap_1022"] = (stats["max_mm"] <= 1100 and stats["beyond_1100"] == 0 and
                                 near_cap > 0.05 * max(total_valid, 1))
    return stats


def verdict_from_stats(stats):
    """Return (code, title, description) - code in {A, MARGINAL, INCONCLUSIVE, B}."""
    robust_max = stats["p99"]
    # Scene too close (camera blocked / facing down / object against the lens) -> cannot measure far range
    if stats["p95"] < 500:
        return ("INCONCLUSIVE", "INCONCLUSIVE - scene too close",
                f"95% of pixels < {stats['p95']}mm (median {stats['p50']}mm) -> the camera is looking at a "
                "VERY CLOSE object / is blocked / facing down. Cannot assess the sensor's far range. "
                "Point the camera at the FARTHEST & most OPEN scene (corridor, large room) then RUN AGAIN.")
    if stats["suspect_cap_1022"]:
        return ("B", "STOP - suspected firmware cap ~1022mm",
                "The scene has far areas but NO pixel is >1.1m and there is a cluster near 1022mm -> range is "
                "limited to ~1m. Almost impossible to map a room. RECOMMEND: 2D LiDAR (RPLIDAR A1/C1).")
    if robust_max >= VERDICT_GOOD_MM:
        return ("A", "GOOD ENOUGH - continue the plan",
                f"Robust max range (p99) = {robust_max}mm >= {VERDICT_GOOD_MM}mm. "
                "depthimage_to_laserscan + slam_toolbox is viable (still mind the ~58 deg FOV, drive slowly with lots of overlap).")
    if robust_max >= VERDICT_MARGINAL_MM:
        return ("MARGINAL", "MARGINAL - small area only, high risk",
                f"Robust max range (p99) = {robust_max}mm (1.5-3m). Can map a SMALL area "
                "but the map drifts easily due to the narrow FOV + short range. Consider a LiDAR for reliability.")
    return ("B", "STOP - range too short",
            f"Robust max range (p99) = {robust_max}mm < {VERDICT_MARGINAL_MM}mm. "
            "Not enough to map. RECOMMEND: add a 2D LiDAR (RPLIDAR A1/C1, ~$70-100).")


def print_probe_report(stats):
    print("\n" + "=" * 64)
    print("PROBE RESULT - ASTRA DEPTH RANGE & FOV")
    print("=" * 64)
    print(f"  Frames collected : {stats['frames']}  ({stats['fps']:.1f} FPS)"
          f"{'  | read_errors=%d' % stats['read_errors'] if stats['read_errors'] else ''}")
    print(f"  Frame size       : {stats['shape'][1]}x{stats['shape'][0]} (WxH)")
    print(f"  %valid pixels    : {100*stats['valid_ratio']:.1f}%")
    if stats["fov_h_deg"] is not None:
        print(f"  Geometric FOV    : horizontal {stats['fov_h_deg']:.1f} deg  | vertical {stats['fov_v_deg']:.1f} deg")
    print(f"  Horiz. coverage  : {100*stats['usable_col_ratio']:.0f}% of columns have depth (center band)")
    print("  -- Valid depth distribution (mm) --")
    print(f"     min   : {stats['min_mm']}")
    print(f"     p05   : {stats['p05']}")
    print(f"     p50   : {stats['p50']}  (median)")
    print(f"     p95   : {stats['p95']}")
    print(f"     p99   : {stats['p99']}  <- ROBUST MAX RANGE (used for the verdict)")
    print(f"     p99.9 : {stats['p999']}")
    print(f"     max   : {stats['max_mm']}")
    print(f"  Pixels with depth >1.1m : {stats['beyond_1100']:,}  (=0 + far scene => suspect ~1m cap)")
    print("  -- Histogram --")
    print(stats["hist_text"])

    code, title, desc = verdict_from_stats(stats)
    print("\n" + "-" * 64)
    print(f"  VERDICT [{code}]: {title}")
    print(f"  {desc}")
    print("-" * 64 + "\n")
    return code, title, desc


def write_markdown(stats, code, title, desc, path, args):
    lines = [
        "# Astra - Depth range & FOV report (T0 spike)",
        "",
        f"**Measured at:** {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
        f"**Mode:** probe . {('%d frames' % args.frames) if args.frames else ('%ds' % args.duration)}",
        "",
        f"## VERDICT: `[{code}]` {title}",
        "",
        f"> {desc}",
        "",
        "## Metrics",
        "",
        "| Metric | Value |",
        "|--------|-------|",
        f"| Frames / FPS | {stats['frames']} / {stats['fps']:.1f} |",
        f"| Frame size | {stats['shape'][1]}x{stats['shape'][0]} (WxH) |",
        f"| %valid pixels | {100*stats['valid_ratio']:.1f}% |",
        f"| FOV horizontal / vertical (geometric) | {stats['fov_h_deg']:.1f} deg / {stats['fov_v_deg']:.1f} deg |"
        if stats["fov_h_deg"] is not None else "| FOV | (missing intrinsics) |",
        f"| Usable horizontal coverage | {100*stats['usable_col_ratio']:.0f}% of columns |",
        f"| min depth | {stats['min_mm']} mm |",
        f"| p50 (median) | {stats['p50']} mm |",
        f"| p95 | {stats['p95']} mm |",
        f"| **p99 (robust max range)** | **{stats['p99']} mm** |",
        f"| p99.9 | {stats['p999']} mm |",
        f"| max | {stats['max_mm']} mm |",
        "",
        "## Histogram (mm)",
        "",
        "```",
        stats["hist_text"],
        "```",
        "",
        "## Implications for PLAN_SLAM_NAV2.md",
        "",
        "- `[A]` range >=3m -> continue T1+ (slam_toolbox + NAV2 on Astra depth).",
        "- `[MARGINAL]` 1.5-3m -> small area only, map-drift risk; consider a LiDAR.",
        "- `[B]` <1.5m or ~1022mm cap -> STOP, propose a 2D LiDAR before investing in the stack.",
    ]
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")
    print(f">> Report written: {path}")


# ================================================================================
# CALIB mode - measure error at known distances
# ================================================================================
def _detrended_std(patch):
    """std(mm) after subtracting a fitted tilted plane over the ROI -> true sensor noise,
    without the surface tilt/curvature. Returns (std_raw, std_detrend, n_valid)."""
    mask = patch > 0
    z = patch[mask].astype(np.float64)
    if z.size < 16:
        return None, None, int(z.size)
    std_raw = float(z.std())
    ys, xs = np.nonzero(mask)
    A = np.column_stack([xs.astype(np.float64), ys.astype(np.float64), np.ones(z.size)])
    try:
        coef, *_ = np.linalg.lstsq(A, z, rcond=None)
        resid = z - A @ coef
        std_det = float(resid.std())
    except np.linalg.LinAlgError:
        std_det = std_raw
    return std_raw, std_det, int(z.size)


def measure_calib_frame(cam, roi, n=30, warmup=5):
    """Measure one distance point. Collect n frames, return dict:
       median_mm, std_raw_mm, std_detrend_mm (F21 noise),
       valid_center (center ROI), valid_full (whole frame - the F22 drop-off curve), n_frames.
       Returns None if there is no valid depth at the center."""
    medians, std_raws, std_dets = [], [], []
    vc_list, vf_list = [], []
    for i in range(n + warmup):
        try:
            _, depth = cam.read()
        except RuntimeError:
            continue
        if i < warmup:
            continue
        h, w = depth.shape
        cy, cx = h // 2, w // 2
        r = roi // 2
        patch = depth[max(0, cy - r):cy + r, max(0, cx - r):cx + r]
        valid = patch[patch > 0]
        vc_list.append(valid.size / max(patch.size, 1))
        vf_list.append(int((depth > 0).sum()) / max(depth.size, 1))  # F22: whole frame
        if valid.size:
            medians.append(float(np.median(valid)))
            sr, sd, _ = _detrended_std(patch)
            if sr is not None:
                std_raws.append(sr)
                std_dets.append(sd)
    if not medians:
        return None
    return {
        "median_mm": float(np.median(medians)),
        "std_raw_mm": float(np.mean(std_raws)) if std_raws else float("nan"),
        "std_detrend_mm": float(np.mean(std_dets)) if std_dets else float("nan"),
        "valid_center": float(np.mean(vc_list)),
        "valid_full": float(np.mean(vf_list)),
        "n_frames": len(medians),
    }


_CSV_HEADER = ("truth_mm,median_mm,err_mm,err_pct,std_raw_mm,std_detrend_mm,"
               "valid_center_pct,valid_full_pct,n_frames")


def run_calib(cam, args):
    print(">> CALIB: place a flat surface (wall/cardboard) filling the CENTER of the frame at a known distance.")
    print(">> Enter the true distance (mm) then Enter. Leave EMPTY then Enter to finish.")
    print(f">> Each point collects {args.calib_frames} frames; the CSV is appended to: {args.csv}\n")
    csv_path = args.csv if os.path.isabs(args.csv) else os.path.join(_REPO_ROOT, args.csv)
    os.makedirs(os.path.dirname(csv_path), exist_ok=True)
    with open(csv_path, "w") as f:                     # write the header now, append each point
        f.write(_CSV_HEADER + "\n")

    rows = []
    while True:
        try:
            s = input("True distance (mm) [Enter=done]: ").strip()
        except EOFError:
            break
        if not s:
            break
        try:
            truth = float(s)
        except ValueError:
            print("   ! Enter a number in mm, e.g. 1000")
            continue
        m = measure_calib_frame(cam, roi=args.roi, n=args.calib_frames)
        if m is None:
            print("   ! No valid depth at the center - object too far/close? (valid_full still logged as 0)")
            with open(csv_path, "a") as f:
                f.write(f"{truth:.0f},,,,,,,,0\n")
            rows.append((truth, None))
            continue
        err = m["median_mm"] - truth
        pct = 100 * err / truth if truth else 0.0
        print(f"   -> measured={m['median_mm']:.0f}mm  error={err:+.0f}mm ({pct:+.1f}%)  "
              f"noise(detrended)={m['std_detrend_mm']:.1f}mm  "
              f"valid: center={100*m['valid_center']:.0f}% full={100*m['valid_full']:.0f}%")
        # QUALITY warnings right at this point - avoid measuring all session then finding it broken
        warns = []
        if abs(pct) > 15:
            warns.append(f"median off by {pct:+.0f}% vs the ruler -> the center ROI is NOT on the plane")
        if np.isfinite(m["std_detrend_mm"]) and m["std_detrend_mm"] > 30:
            warns.append(f"std {m['std_detrend_mm']:.0f}mm too large -> the ROI crosses the object edge/background (not noise)")
        if m["median_mm"] >= 1015:
            warns.append("median ~1022 = firmware cap -> object below the minimum range (~0.6m) or reading the far background")
        if m["valid_center"] > 0.99 and abs(pct) > 15:
            warns.append("center still 100% valid but wrong distance -> the background fills the frame, object not isolated")
        for w in warns:
            print(f"      ! {w}")
        if warns:
            print("      -> RE-MEASURE this point: flat object FILLING the frame, perpendicular, distance >=600mm.")
        with open(csv_path, "a") as f:
            f.write(f"{truth:.0f},{m['median_mm']:.1f},{err:+.1f},{pct:+.2f},"
                    f"{m['std_raw_mm']:.2f},{m['std_detrend_mm']:.2f},"
                    f"{100*m['valid_center']:.1f},{100*m['valid_full']:.1f},{m['n_frames']}\n")
        rows.append((truth, m))

    print("\n" + "=" * 78)
    print("NOISE & %VALID BY DISTANCE  (F21: std . F22: valid_full)")
    print("=" * 78)
    print(f"  {'truth(mm)':>9} {'meas(mm)':>8} {'error':>8} {'std_det':>8} "
          f"{'valid_ctr':>10} {'valid_full':>11}")
    for truth, m in rows:
        if m is None:
            print(f"  {truth:>9.0f} {'-':>8} {'-':>8} {'-':>8} {'-':>10} {'0%':>11}")
        else:
            print(f"  {truth:>9.0f} {m['median_mm']:>8.0f} {m['median_mm']-truth:>+8.0f} "
                  f"{m['std_detrend_mm']:>7.1f}m {100*m['valid_center']:>9.0f}% "
                  f"{100*m['valid_full']:>10.0f}%")
    print("=" * 78)
    print(f">> CSV (plot F21/F22): {csv_path}")
    return rows


# ================================================================================
def main():
    ap = argparse.ArgumentParser(description="T0 spike - measure Astra depth range & FOV")
    ap.add_argument("--mode", choices=("probe", "calib"), default="probe")
    ap.add_argument("--duration", type=float, default=15.0, help="probe: seconds to collect data")
    ap.add_argument("--frames", type=int, default=0, help="probe: number of frames (takes priority over --duration if >0)")
    ap.add_argument("--warmup", type=int, default=10, help="probe: leading frames to drop")
    ap.add_argument("--bins", type=int, default=500, help="probe: histogram bin width (mm)")
    ap.add_argument("--roi", type=int, default=60, help="calib: center square side (px)")
    ap.add_argument("--calib-frames", type=int, default=100, help="calib: frames per distance")
    ap.add_argument("--csv", default=os.path.join("docs", "astra_noise_vs_distance.csv"),
                    help="calib: CSV of noise+%valid by distance (plot F21/F22)")
    ap.add_argument("--output", default=os.path.join("docs", "astra_depth_range.md"),
                    help="probe: Markdown report path")
    args = ap.parse_args()

    try:
        cam = AstraCamera(mode="depth")
    except Exception as e:
        print(f"[ERR] Could not open the Astra: {e}", file=sys.stderr)
        print("      Check: camera plugged in? `lsusb | grep 2bc5`? driver at tools/orbbec/openni2/?",
              file=sys.stderr)
        return 2

    try:
        intr = load_intrinsics()
        if args.mode == "probe":
            stats = run_probe(cam, args, intr)
            code, title, desc = print_probe_report(stats)
            out = args.output if os.path.isabs(args.output) else os.path.join(_REPO_ROOT, args.output)
            write_markdown(stats, code, title, desc, out, args)
            # exit code: A/MARGINAL=0, B=3 (LiDAR decision needed), INCONCLUSIVE=4 (rerun)
            return {"B": 3, "INCONCLUSIVE": 4}.get(code, 0)
        else:
            run_calib(cam, args)
            return 0
    except KeyboardInterrupt:
        print("\n[INTERRUPT] Stopped on request.")
        return 130
    finally:
        cam.close()


if __name__ == "__main__":
    sys.exit(main())
