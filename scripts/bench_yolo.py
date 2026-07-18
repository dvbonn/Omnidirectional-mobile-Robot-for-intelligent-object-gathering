#!/usr/bin/env python3
"""
bench_yolo.py — Benchmark YOLOv8n cho MOT device (cpu hoac cuda).
Chay rieng tung process de do RAM model sach. Dung frame test 640x640 (giong
benchmark.py goc) de so sanh duoc voi so lieu cu.

    python3 scripts/bench_yolo.py --device cuda --runs 100 --out docs/_yolo_raw.jsonl
    python3 scripts/bench_yolo.py --device cpu  --runs 50  --out docs/_yolo_raw.jsonl
"""
import argparse, json, os, time
from pathlib import Path
import cv2, numpy as np, psutil

PROJECT = Path(__file__).resolve().parent.parent
YOLO_CANDIDATES = [PROJECT / "layer1_vision/model/yolov8n.pt", PROJECT / "yolov8n.pt"]

def ram_mb():
    return psutil.Process(os.getpid()).memory_info().rss / (1024 ** 2)

def test_frame():
    f = np.zeros((640, 640, 3), dtype=np.uint8)
    cv2.rectangle(f, (200, 200), (440, 440), (180, 120, 60), -1)
    cv2.putText(f, "TEST", (250, 340), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 3)
    return f

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--device", choices=["cpu", "cuda"], required=True)
    ap.add_argument("--runs", type=int, default=100)
    ap.add_argument("--warmup", type=int, default=5)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    yolo_path = next((p for p in YOLO_CANDIDATES if p.exists()), None)
    if yolo_path is None:
        raise SystemExit("[LOI] khong tim thay yolov8n.pt")

    import torch
    from ultralytics import YOLO
    dev = "cuda:0" if args.device == "cuda" else "cpu"
    if args.device == "cuda" and not torch.cuda.is_available():
        raise SystemExit("[LOI] CUDA khong kha dung")

    frame = test_frame()
    # --- Do RAM theo dung phuong phap benchmark.py goc: RSS ngay sau khi nap
    #     model, TRUOC khi inference khoi tao CUDA context (de so sanh duoc voi
    #     cot PC va so lieu cu cua bao cao) ---
    ram_before = ram_mb()
    model = YOLO(str(yolo_path))
    model.to(dev)
    ram_after_load = ram_mb()           # -> ram_model, ram_total (pre-inference)
    ram_model = ram_after_load - ram_before

    # warmup (kich hoat CUDA context, on dinh clock)
    for _ in range(args.warmup):
        model(frame, verbose=False, device=dev)

    times, dets = [], []
    for _ in range(args.runs):
        t0 = time.perf_counter()
        r = model(frame, verbose=False, conf=0.45, device=dev)
        if args.device == "cuda":
            torch.cuda.synchronize()
        times.append(time.perf_counter() - t0)
        dets.append(len(r[0].boxes))

    ram_steady = ram_mb()               # RSS thuc khi infer (gom CUDA runtime libs)
    avg_ms = sum(times)/len(times)*1000
    res = {
        "device": args.device,
        "torch": torch.__version__,
        "cuda_device": torch.cuda.get_device_name(0) if torch.cuda.is_available() else None,
        "fps": round(1000/avg_ms, 1),
        "latency_avg_ms": round(avg_ms, 1),
        "latency_min_ms": round(min(times)*1000, 1),
        "latency_max_ms": round(max(times)*1000, 1),
        "ram_model_mb": round(ram_model, 0),
        "ram_total_mb": round(ram_after_load, 0),       # pre-inference (giong bao cao)
        "ram_steady_mb": round(ram_steady, 0),          # post-inference thuc te (minh bach)
        "avg_detections": round(sum(dets)/len(dets), 1),
        "n_runs": args.runs,
    }
    with open(args.out, "a", encoding="utf-8") as f:
        f.write(json.dumps(res, ensure_ascii=False) + "\n")
    print(json.dumps(res, ensure_ascii=False, indent=2))

if __name__ == "__main__":
    main()
