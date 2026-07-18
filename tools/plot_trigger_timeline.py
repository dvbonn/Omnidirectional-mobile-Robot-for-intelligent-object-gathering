#!/usr/bin/env python3
"""
Plot F14 - StableTrigger timeline (stable for 2s -> capture, cooldown 5s).
=========================================================================
Source: CSV written by vision_node.py --trigger-log <path>
    columns: t, has_det, top_conf, elapsed_s, state, trigger_event

The figure shows:
  - the YOLO confidence line over time (+ the 0.45 threshold),
  - background bands by state: DETECTING (counting 2s) / COOLDOWN (5s),
  - a marker at each TRIGGER.
Summary stats (trigger count, FP, trigger spacing) are printed to the console.

Note: the figure axis/title/legend text is kept in Vietnamese to match the thesis figure.

    python tools/plot_trigger_timeline.py --csv docs/trigger_timeline.csv
"""
import argparse
import csv
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PNG = os.path.join(ROOT, "KLTN_figures", "real_data", "png")

STABLE_DURATION = 2.0
COOLDOWN = 5.0
CONF_THRESHOLD = 0.45
STATE_COLOR = {"DETECTING": "#F0E442", "COOLDOWN": "#D0D0D0"}


def load(csv_path):
    t, conf, state, ev, el = [], [], [], [], []
    with open(csv_path, newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            t.append(float(r["t"]))
            conf.append(float(r["top_conf"]))
            state.append(r["state"])
            ev.append(int(r["trigger_event"]))
            el.append(float(r["elapsed_s"]))
    t = np.array(t)
    if len(t):
        t = t - t[0]          # shift to 0 (drop the camera/YOLO warmup offset) for readability
    return t, np.array(conf), state, np.array(ev), np.array(el)


def _spans(t, state, want):
    """Return the continuous (t_start, t_end) segments where state == want."""
    out, start = [], None
    for i, s in enumerate(state):
        if s == want and start is None:
            start = t[i]
        elif s != want and start is not None:
            out.append((start, t[i]))
            start = None
    if start is not None:
        out.append((start, t[-1]))
    return out


def plot(t, conf, state, ev, el, out_path):
    # Widen by duration so 10+ triggers do not crowd (>=9in, ~0.11in/second)
    width = float(np.clip(0.11 * (t.max() - t.min()) + 4, 9, 18))
    fig, ax = plt.subplots(figsize=(width, 4.4))
    # Background bands by state
    for name, color in STATE_COLOR.items():
        for a, b in _spans(t, state, name):
            ax.axvspan(a, b, color=color, alpha=0.5, lw=0)
    # Confidence - only plot when an object is present (conf>0); leave gaps when absent for a clean
    # line, avoiding a "picket fence" of spikes dropping to 0 between appearances.
    conf_plot = np.where(conf > 0, conf, np.nan)
    ax.plot(t, conf_plot, "-", color="#0072B2", lw=1.2, label="Confidence YOLO (khi có vật)")
    ax.axhline(CONF_THRESHOLD, ls="--", color="gray", lw=1,
               label=f"Ngưỡng conf = {CONF_THRESHOLD}")
    # Trigger: thin line + numbered marker on top (tidier than overlapping "TRIGGER" text)
    trig_t = t[ev == 1]
    for i, tt in enumerate(trig_t, 1):
        ax.axvline(tt, color="#D55E00", lw=1.3, alpha=0.85)
        ax.plot(tt, 1.045, marker="v", color="#D55E00", ms=7, clip_on=False)
        ax.annotate(str(i), (tt, 1.08), color="#D55E00", fontsize=7.5,
                    ha="center", va="bottom", clip_on=False)
    # Legend
    from matplotlib.patches import Patch
    handles, _ = ax.get_legend_handles_labels()
    handles += [Patch(color=STATE_COLOR["DETECTING"], alpha=0.5,
                      label=f"DETECTING (đếm {STABLE_DURATION:.0f}s)"),
                Patch(color=STATE_COLOR["COOLDOWN"], alpha=0.5,
                      label=f"COOLDOWN ({COOLDOWN:.0f}s)")]
    if len(trig_t):
        handles.append(plt.Line2D([0], [0], color="#D55E00", lw=1.3,
                                  marker="v", ms=6, label="Lần TRIGGER (đánh số)"))
    ax.legend(handles=handles, loc="lower center", fontsize=8, ncol=4,
              framealpha=0.9, bbox_to_anchor=(0.5, -0.32))
    # Summary line - FP computed FROM THE DATA: any trigger that fired while elapsed < 2s
    gaps = np.diff(trig_t) if len(trig_t) >= 2 else np.array([])
    fp = int(np.sum(el[ev == 1] < STABLE_DURATION - 0.15)) if len(el) else 0
    sub = (f"{len(trig_t)} lần trigger"
           + (f" · cooldown tối thiểu = {gaps.min():.1f}s (≥ {COOLDOWN:.0f}s)" if gaps.size else "")
           + f" · dương tính giả (FP) = {fp}")
    ax.set_xlabel("Thời gian (s)")
    ax.set_ylabel("Confidence")
    ax.set_ylim(0, 1.1)
    ax.set_xlim(t.min(), t.max())
    ax.set_title("StableTrigger — ổn định 2s → chụp, cooldown 5s\n" + sub, fontsize=11)
    ax.grid(alpha=0.25)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    fig.savefig(out_path, bbox_inches="tight", dpi=120)
    plt.close(fig)
    return trig_t


def summary(t, state, ev, el, trig_t):
    has_det = sum(1 for s in state if s in ("DETECTING", "TRIGGERED"))
    fp = int(np.sum(el[ev == 1] < STABLE_DURATION - 0.15)) if len(el) else 0
    print("-- F14 summary --")
    print(f"  Duration           : {t.max() - t.min():.1f} s, {len(t)} frames")
    print(f"  TRIGGER count      : {len(trig_t)}")
    if len(trig_t) >= 2:
        gaps = np.diff(trig_t)
        print(f"  Trigger spacing    : {', '.join(f'{g:.1f}s' for g in gaps)} "
              f"(min {gaps.min():.1f}s >= cooldown {COOLDOWN:.0f}s? "
              f"{'PASS' if gaps.min() >= COOLDOWN - 0.5 else 'FAIL'})")
    print(f"  elapsed at trigger : min {el[ev == 1].min():.2f}s "
          f"(every trigger >= {STABLE_DURATION:.0f}s? {'PASS' if fp == 0 else 'FAIL'})")
    print(f"  False positives (FP): {fp}")
    print(f"  Frames with object  : {has_det}/{len(t)}")


def main():
    ap = argparse.ArgumentParser(description="Plot the F14 StableTrigger timeline")
    ap.add_argument("--csv", default=os.path.join("docs", "trigger_timeline.csv"))
    ap.add_argument("--out", default=os.path.join(PNG, "f14_stable_trigger_timeline.png"))
    args = ap.parse_args()
    csv_path = args.csv if os.path.isabs(args.csv) else os.path.join(ROOT, args.csv)
    if not os.path.exists(csv_path):
        print(f"[ERR] Missing CSV: {csv_path}\n"
              "      Run first: python layer1_vision/vision_node.py --trigger-log "
              "docs/trigger_timeline.csv --frames 2000", file=sys.stderr)
        return 2
    t, conf, state, ev, el = load(csv_path)
    if len(t) == 0:
        print("[ERR] Empty CSV.", file=sys.stderr)
        return 2
    trig_t = plot(t, conf, state, ev, el, args.out)
    summary(t, state, ev, el, trig_t)
    print(f">> {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
