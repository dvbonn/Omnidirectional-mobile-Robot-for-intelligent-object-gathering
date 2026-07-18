#!/usr/bin/env python3
"""
capture_eval_dataset.py - Capture test images (Astra) + AUTO-generate ground-truth labels.

Since 'collectible' is a deterministic function of the OBJECT TYPE (fixed rules), you only need
to say what object you are capturing via --object; the label (object + collectible) is generated automatically.

collectible RULES (trash/single-use = true; valuable/fragile = false):
    water_bottle   -> True   [COCO: bottle]
    plastic_bottle -> True   [COCO: bottle]
    paper_box      -> True   [COCO: NONE -> YOLO cannot detect it!]
    phone          -> False  [COCO: cell phone]
    glass_cup      -> False  [COCO: cup -> YOLO thinks it is collectible]

USAGE (run once per object type, aim for ~10 images/type to reach ~50):
    python3 tools/capture_eval_dataset.py --object water_bottle
    python3 tools/capture_eval_dataset.py --object glass_cup
    ...
GUI (with a monitor): live window, 's' to save, 'q' to quit.
HEADLESS (SSH, no monitor): press Enter to capture, type 'q'+Enter to finish.

Images -> data/eval_dataset/images/  |  Labels -> data/eval_dataset/labels.json
"""
import argparse, json, os, sys, time
from pathlib import Path

import cv2

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))
from layer1_vision.cameras.astra_openni import AstraCamera

OBJECT_CLASSES = {
    "water_bottle":   {"label": "water bottle",   "collectible": True,  "coco": "bottle"},
    "plastic_bottle": {"label": "plastic bottle", "collectible": True,  "coco": "bottle"},
    "paper_box":      {"label": "paper box",      "collectible": True,  "coco": None},
    "phone":          {"label": "phone",          "collectible": False, "coco": "cell phone"},
    "glass_cup":      {"label": "glass cup",       "collectible": False, "coco": "cup"},
}

DATASET = REPO / "data" / "eval_dataset"
IMG_DIR = DATASET / "images"
LABELS  = DATASET / "labels.json"


def load_labels():
    if LABELS.exists():
        return json.loads(LABELS.read_text(encoding="utf-8"))
    return []


def save_labels(rows):
    DATASET.mkdir(parents=True, exist_ok=True)
    LABELS.write_text(json.dumps(rows, ensure_ascii=False, indent=2), encoding="utf-8")


def next_index(rows, obj):
    return sum(1 for r in rows if r["object"] == obj) + 1


def add_capture(rows, obj, bgr):
    info = OBJECT_CLASSES[obj]
    idx = next_index(rows, obj)
    fname = f"{obj}_{idx:03d}.jpg"
    IMG_DIR.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(IMG_DIR / fname), bgr, [cv2.IMWRITE_JPEG_QUALITY, 90])
    row = {"file": fname, "object": obj, "label": info["label"],
           "collectible": info["collectible"], "coco_class": info["coco"]}
    rows.append(row)
    save_labels(rows)
    return fname


def has_gui():
    return bool(os.environ.get("DISPLAY")) and hasattr(cv2, "imshow")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--object", required=True, choices=list(OBJECT_CLASSES))
    ap.add_argument("--max", type=int, default=20, help="max images this run")
    ap.add_argument("--headless", action="store_true", help="force headless mode")
    args = ap.parse_args()

    info = OBJECT_CLASSES[args.object]
    rows = load_labels()
    gui = has_gui() and not args.headless

    print("=" * 60)
    print(f"  CAPTURE: {info['label']}  (collectible={info['collectible']}, COCO={info['coco']})")
    print(f"  Total so far: {len(rows)} images; this type: {sum(1 for r in rows if r['object']==args.object)}")
    print(f"  Mode: {'GUI (s=save, q=quit)' if gui else 'HEADLESS (Enter=capture, q=quit)'}")
    print("=" * 60)

    cam = AstraCamera(mode="color")
    saved = 0
    try:
        # warmup auto-exposure
        for _ in range(5):
            cam.read()
        if gui:
            win = f"Capture {args.object}"
            while saved < args.max:
                bgr, _ = cam.read()
                disp = bgr.copy()
                txt = f"{info['label']} | collectible={info['collectible']} | saved={saved}"
                cv2.putText(disp, txt, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                            (0, 255, 0), 2)
                cv2.putText(disp, "s=save  q=quit", (10, 60), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (0, 200, 255), 2)
                cv2.imshow(win, disp)
                k = cv2.waitKey(1) & 0xFF
                if k == ord('s'):
                    fname = add_capture(rows, args.object, bgr)
                    saved += 1
                    print(f"  [+] {fname}  (this run {saved})")
                elif k == ord('q'):
                    break
            cv2.destroyAllWindows()
        else:
            while saved < args.max:
                cmd = input(f"  Enter=capture ({saved} saved) | q=quit > ").strip().lower()
                if cmd == 'q':
                    break
                for _ in range(2):      # flush to get the latest frame
                    bgr, _ = cam.read()
                fname = add_capture(rows, args.object, bgr)
                saved += 1
                print(f"  [+] {fname}")
    finally:
        cam.close()

    total = len(rows)
    n_true = sum(1 for r in rows if r["collectible"])
    print("-" * 60)
    print(f"  Saved {saved} images this run. TOTAL dataset: {total} images "
          f"({n_true} collectible / {total-n_true} not).")
    print(f"  Images: {IMG_DIR}")
    print(f"  Labels: {LABELS}")
    if total < 50:
        print(f"  Need {50-total} more images to reach 50. Capture another type: --object <key>")
    else:
        print(f"  Have >=50 images. Run: python3 scripts/eval_accuracy.py")


if __name__ == "__main__":
    main()
