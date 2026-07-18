#!/usr/bin/env python3
"""
Realtime Astra demo: detect the nearest blob + print (X,Y,Z) mm coordinates + FPS.
=================================================================================
Runs in mode="depth" (>=15 FPS). Shows a colorized depth image + bbox + center crosshair +
X,Y,Z (mm) text + FPS. ALSO writes each detection to a structured JSONL log
(layer1_vision.detection_log) - the schema matches the Brain/VLM prompting architecture + 3D
coordinates + ROS2 fields (stamp, frame_id, position_m), so a ROS2 node only needs to tail this
file and publish geometry_msgs/PointStamped. See layer1_vision/detection_log.py.

Two display modes (auto-selected via $DISPLAY):
  - GUI    (with a monitor)  : live window; 'q' to quit, 's' to save a snapshot.
  - HEADLESS (SSH/no X)      : print coords+FPS to the terminal, periodically save an annotated
                               image to --out; run for --frames frames then stop.

JSONL log (always written in both modes): default Log/astra_demo/detections_<epoch>.jsonl.
  tail -f to view in realtime:  tail -f Log/astra_demo/detections_*.jsonl

Examples:
  python tools/astra_coord_demo.py                       # auto GUI/headless
  python tools/astra_coord_demo.py --headless --frames 300
  python tools/astra_coord_demo.py --out Log/astra_demo --save-every 30
  python tools/astra_coord_demo.py --log Log/astra_demo/detections.jsonl --log-empty
"""
import argparse
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from layer1_vision.cameras.astra_openni import AstraCamera          # noqa: E402
from layer1_vision.depth_detect import detect_nearest_blob          # noqa: E402
from layer1_vision.depth_utils import Intrinsics, unproject         # noqa: E402
from layer1_vision.detection_log import (                           # noqa: E402
    DEFAULT_FRAME_ID, DetectionLogger, make_record,
)


def colorize_depth(depth_mm: np.ndarray, z_max: float = 2000.0) -> np.ndarray:
    """Depth (mm) -> colorized BGR image (JET); invalid pixels -> black."""
    d = np.clip(depth_mm, 0, z_max)
    norm = (d / z_max * 255).astype(np.uint8)
    color = cv2.applyColorMap(norm, cv2.COLORMAP_JET)
    color[depth_mm <= 0] = (0, 0, 0)
    return color


def annotate(canvas, det, coord, fps):
    """Draw bbox + crosshair + coordinate text + FPS onto the canvas (BGR)."""
    h, w = canvas.shape[:2]
    if det is not None:
        x, y, bw, bh = det["bbox"]
        cu, cv_ = (int(round(v)) for v in det["center"])
        cv2.rectangle(canvas, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
        cv2.drawMarker(canvas, (cu, cv_), (0, 0, 255), cv2.MARKER_CROSS, 18, 2)
        X, Y, Z = coord
        txt = f"X={X:+.0f} Y={Y:+.0f} Z={Z:.0f} mm"
        cv2.rectangle(canvas, (x, y - 22), (x + 230, y), (0, 255, 0), -1)
        cv2.putText(canvas, txt, (x + 3, y - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
    else:
        cv2.putText(canvas, "No object in frame", (10, h - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
    cv2.putText(canvas, f"FPS {fps:4.1f}", (w - 110, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    return canvas


def print_coord_line(i, det, coord, fps):
    """Print one coordinate line to the terminal (used for both GUI and headless)."""
    if det is not None:
        X, Y, Z = coord
        print(f"frame {i:4d} | X={X:+7.0f} Y={Y:+7.0f} Z={Z:6.0f} mm | {fps:4.1f} FPS",
              flush=True)
    else:
        print(f"frame {i:4d} | (no object)               | {fps:4.1f} FPS", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--headless", action="store_true",
                    help="force headless mode (default: auto-detect via $DISPLAY)")
    ap.add_argument("--frames", type=int, default=300, help="number of frames (headless)")
    ap.add_argument("--out", default="Log/astra_demo", help="image output folder (headless/snapshot)")
    ap.add_argument("--save-every", type=int, default=30, help="save 1 image every N frames (headless)")
    ap.add_argument("--z-max", type=float, default=2000.0, help="color scale/clamp for depth (mm)")
    ap.add_argument("--log", default=None,
                    help="JSONL log path (default <out>/detections_<epoch>.jsonl)")
    ap.add_argument("--frame-id", default=DEFAULT_FRAME_ID,
                    help="optical TF frame_id written to the log (for ROS2)")
    ap.add_argument("--log-empty", action="store_true",
                    help="also log frames with NO object (default: only when there is a detection)")
    ap.add_argument("--print-every", type=int, default=1,
                    help="print coords to the terminal every N frames (both GUI and headless; "
                         "raise it if the terminal floods at high FPS)")
    args = ap.parse_args()

    headless = args.headless or not os.environ.get("DISPLAY")
    os.makedirs(args.out, exist_ok=True)
    log_path = args.log or os.path.join(args.out, f"detections_{int(time.time())}.jsonl")
    intr = Intrinsics.load()
    print(f"Intrinsics: fx={intr.fx:.1f} fy={intr.fy:.1f} cx={intr.cx} cy={intr.cy}", flush=True)
    print(f"Mode: {'HEADLESS (save images)' if headless else 'GUI (q=quit, s=snapshot)'}", flush=True)
    print(f"JSONL log -> {log_path}  (frame_id={args.frame_id})", flush=True)

    cam = AstraCamera(mode="depth")
    dlog = DetectionLogger(log_path)
    fps = 0.0
    t_prev = time.time()
    saved = 0
    i = 0
    try:
        while True:
            _, depth = cam.read()
            det = detect_nearest_blob(depth)
            coord = (float("nan"),) * 3
            if det is not None:
                cu, cv_ = det["center"]
                coord = unproject(cu, cv_, det["median_z"], intr)

            now = time.time()
            inst = 1.0 / max(now - t_prev, 1e-6)
            fps = 0.9 * fps + 0.1 * inst if fps else inst
            t_prev = now

            # Write the JSONL log (ROS2 bridge)
            if det is not None:
                dlog.log(make_record(
                    frame=i, bbox=det["bbox"], center_px=det["center"],
                    coord_mm=coord, source="astra_depth_nearest",
                    frame_id=args.frame_id, fps=fps, stamp=now,
                ))
            elif args.log_empty:
                dlog.log(make_record(
                    frame=i, bbox=None, center_px=None, coord_mm=None,
                    source="astra_depth_nearest", frame_id=args.frame_id,
                    fps=fps, stamp=now,
                ))

            canvas = annotate(colorize_depth(depth, args.z_max), det, coord, fps)
            # red dot + "REC" indicating logging is active
            cv2.circle(canvas, (16, 22), 6, (0, 0, 255), -1)
            cv2.putText(canvas, "REC", (28, 27),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

            if i % args.print_every == 0:
                print_coord_line(i, det, coord, fps)

            if headless:
                if i % args.save_every == 0:
                    p = os.path.join(args.out, f"frame_{i:04d}.jpg")
                    cv2.imwrite(p, canvas)
                    saved += 1
                i += 1
                if i >= args.frames:
                    break
            else:
                cv2.imshow("Astra coord demo", canvas)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                if key == ord("s"):
                    p = os.path.join(args.out, f"snapshot_{int(now)}.jpg")
                    cv2.imwrite(p, canvas)
                    saved += 1
                    print(f"  saved {p}", flush=True)
                i += 1
    except KeyboardInterrupt:
        print("\n(stopped with Ctrl-C)", flush=True)
    finally:
        cam.close()
        dlog.close()
        if not headless:
            cv2.destroyAllWindows()
        print(f"\nDone: {i} frames, ~{fps:.1f} FPS, saved {saved} images to {args.out}/", flush=True)
        print(f"Log: {dlog.count} JSONL records -> {log_path}", flush=True)


if __name__ == "__main__":
    main()
