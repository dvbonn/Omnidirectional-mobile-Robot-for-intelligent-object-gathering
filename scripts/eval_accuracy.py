#!/usr/bin/env python3
"""
eval_accuracy.py - Measure 'collectibility accuracy' for the 3 columns of Table 4.3 + Accuracy of Table 4.2.

Uses a self-captured dataset (data/eval_dataset/labels.json + images/). Auto-starts
llama-server (default ngl=-1, fastest) for the VLM branch.

3 methods:
  - YOLO-only : YOLOv8n (COCO) -> map class to collectible with a hard-coded table.
                * glass_cup: YOLO sees 'cup' -> maps True but GT=False -> WRONG.
                * paper_box: no COCO class -> YOLO does not detect -> WRONG.
                => 2 cases that expose YOLO's limits.
  - VLM-only  : Qwen2.5-VL reads the image -> 'collectible' field.
  - Proposed  : trigger = an object appears (YOLO has any box) -> VLM decides.
                Empty frame -> no trigger -> collectible=False.

    python3 scripts/eval_accuracy.py [--ngl -1] [--limit N]
Result -> docs/bang_4_3_accuracy_results.json

Note: the output JSON keys/values are kept in Vietnamese on purpose - they mirror the
Chapter 4 tables in the thesis report and the committed docs/bang_4_*.json evidence files.
"""
import argparse, base64, json, re, signal, sys, time
from pathlib import Path
import requests

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))
import importlib.util
_spec = importlib.util.spec_from_file_location("bvc", str(REPO / "scripts/bench_vlm_config.py"))
bvc = importlib.util.module_from_spec(_spec); _spec.loader.exec_module(bvc)

DATASET = REPO / "data" / "eval_dataset"
IMG_DIR = DATASET / "images"
LABELS  = DATASET / "labels.json"
OUT     = REPO / "docs" / "bang_4_3_accuracy_results.json"

# YOLO COCO class -> collectible (limit: 'cup' cannot tell the material apart)
YOLO_COLLECTIBLE = {"bottle": True, "cup": True, "bowl": True, "cell phone": False}
RELEVANT_COCO = set(YOLO_COLLECTIBLE)


def extract_json(text):
    try:
        d = json.loads(text)
        if isinstance(d, dict) and "object" in d:
            return d
    except Exception:
        pass
    for pat in [r'```json\s*(\{.*?\})\s*```', r'\{[^{}]*"object"[^{}]*\}', r'\{.*?\}']:
        for m in re.findall(pat, text, re.DOTALL):
            try:
                d = json.loads(m)
                if isinstance(d, dict) and "object" in d:
                    return d
            except Exception:
                continue
    return None


def to_bool(v):
    if isinstance(v, bool):
        return v
    if isinstance(v, (int, float)):
        return bool(v)
    if isinstance(v, str):
        # accept English and Vietnamese affirmatives (the VLM may answer in either)
        return v.strip().lower() in ("true", "yes", "1", "có", "co")
    return None


def vlm_collectible(img_b64):
    payload = bvc.build_payload(img_b64)
    r = requests.post(f"{bvc.URL}/v1/chat/completions", json=payload, timeout=300)
    if r.status_code != 200:
        return None, None, f"http {r.status_code}"
    txt = r.json()["choices"][0]["message"]["content"]
    d = extract_json(txt)
    if d is None:
        return None, None, "json fail"
    return to_bool(d.get("collectible")), d.get("object"), None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ngl", type=int, default=-1)
    ap.add_argument("--limit", type=int, default=0, help="0 = all")
    args = ap.parse_args()

    if not LABELS.exists():
        sys.exit(f"[ERR] missing {LABELS}. Capture first: python3 tools/capture_eval_dataset.py --object <key>")
    rows = json.loads(LABELS.read_text(encoding="utf-8"))
    if args.limit:
        rows = rows[:args.limit]
    n = len(rows)
    print(f"Dataset: {n} images ({sum(1 for r in rows if r['collectible'])} collectible)")

    from ultralytics import YOLO
    yolo = YOLO(str(REPO / "layer1_vision/model/yolov8n.pt"))
    coco = yolo.names

    # start llama-server
    lp = REPO / "Log" / f"eval_ngl{args.ngl}.log"
    proc, logf = bvc.start_server(args.ngl, lp)
    res = {"yolo_only": [], "vlm_only": [], "de_xuat": []}
    details = []
    try:
        if not bvc.wait_health():
            sys.exit("[ERR] llama-server did not come up")
        print("llama-server READY\n")
        for i, r in enumerate(rows):
            p = IMG_DIR / r["file"]
            gt = r["collectible"]
            img_b64 = base64.b64encode(p.read_bytes()).decode()

            # YOLO
            det = yolo(str(p), verbose=False, conf=0.35)[0]
            yolo_classes = [(coco[int(b.cls)], float(b.conf)) for b in det.boxes]
            rel = [(c, cf) for c, cf in yolo_classes if c in RELEVANT_COCO]
            rel.sort(key=lambda x: -x[1])
            yolo_detected = rel[0][0] if rel else None
            yolo_pred = YOLO_COLLECTIBLE[yolo_detected] if yolo_detected else False

            # VLM
            vlm_pred, vlm_obj, err = vlm_collectible(img_b64)

            # Proposed: trigger = an object is present (presence-based, YOLO is a cheap gate)
            # -> VLM decides collectible. Nothing in the frame -> no trigger.
            triggered = len(det.boxes) > 0
            prop_pred = vlm_pred if (triggered and vlm_pred is not None) else False

            res["yolo_only"].append(yolo_pred == gt)
            res["vlm_only"].append(vlm_pred == gt if vlm_pred is not None else False)
            res["de_xuat"].append(prop_pred == gt)
            details.append({
                "file": r["file"], "gt_object": r["object"], "gt_collectible": gt,
                "yolo_detected": yolo_detected, "yolo_pred": yolo_pred,
                "vlm_obj": vlm_obj, "vlm_pred": vlm_pred, "vlm_err": err,
                "triggered": triggered, "de_xuat_pred": prop_pred,
            })
            print(f"[{i+1}/{n}] {r['file']:<22} GT={gt!s:<5} | "
                  f"YOLO={yolo_detected}->{yolo_pred!s:<5} | VLM={vlm_pred} | proposed={prop_pred}")
    finally:
        proc.send_signal(signal.SIGINT)
        try: proc.wait(timeout=15)
        except Exception: proc.kill()
        logf.close()

    def acc(key):
        v = res[key]
        c = sum(v)
        return {"correct": c, "total": len(v), "accuracy_pct": round(100*c/len(v), 1) if v else 0}

    # NOTE: keys/values below stay in Vietnamese to match the report tables (Chapter 4)
    # and the committed docs/bang_4_*.json evidence files.
    summary = {
        "bang": "4.3 + 4.2",
        "tieu_de": "Accuracy collectibility — 3 phương pháp trên dataset tự chụp Astra",
        "ngay_do": time.strftime("%Y-%m-%d %H:%M:%S"),
        "n_anh": n,
        "n_collectible_true": sum(1 for r in rows if r["collectible"]),
        "quy_tac_collectible": "rác (chai nước/chai nhựa/hộp giấy)=true; điện thoại/ly thủy tinh=false",
        "yolo_collectible_map": YOLO_COLLECTIBLE,
        "ket_qua": {
            "yolo_only": acc("yolo_only"),
            "vlm_only": acc("vlm_only"),
            "de_xuat": acc("de_xuat"),
        },
        "accuracy_cho_bang_4_2": acc("vlm_only")["accuracy_pct"],
        "ghi_chu": "Cột Accuracy Bảng 4.2 = accuracy của VLM (vlm_only). YOLO sai ở glass_cup (COCO chỉ có 'cup') và paper_box (không có lớp COCO).",
        "chi_tiet": details,
    }
    OUT.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    print("\n" + "=" * 55)
    for k in ("yolo_only", "vlm_only", "de_xuat"):
        a = acc(k)
        print(f"  {k:<10}: {a['correct']}/{a['total']} = {a['accuracy_pct']}%")
    print("=" * 55)
    print(f"Result -> {OUT}")


if __name__ == "__main__":
    main()
