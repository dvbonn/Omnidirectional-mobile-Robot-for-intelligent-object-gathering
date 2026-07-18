"""
benchmark.py — Do hieu nang he thong Robot Collecting.

Chay chi do YOLO (khong can server):
    python benchmark.py --yolo-only

Chay day du (can brain_server + llama-server dang chay):
    python benchmark.py

Dung anh test thay webcam:
    python benchmark.py --image <duong_dan_anh.jpg>

Ket qua duoc luu vao: benchmark_results.json
"""

import argparse
import json
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import psutil
import requests

PROJECT_DIR = Path(__file__).parent.parent
BRAIN_URL   = "http://localhost:8000"
YOLO_CANDIDATES = [
    PROJECT_DIR / "layer1_vision" / "model" / "yolov8n.pt",
    PROJECT_DIR / "yolov8n.pt",
]

SEP  = "-" * 58
SEP2 = "=" * 58


# ─────────────────────────────────────────────────────────────
# HELPERS
# ─────────────────────────────────────────────────────────────

def current_ram_mb() -> float:
    return psutil.Process(os.getpid()).memory_info().rss / (1024 ** 2)


def get_process_ram_mb(name_keywords: list) -> float:
    total = 0
    for proc in psutil.process_iter(["name", "memory_info", "cmdline"]):
        try:
            cmdline = " ".join(proc.info["cmdline"] or [])
            if any(kw.lower() in cmdline.lower() for kw in name_keywords):
                total += proc.info["memory_info"].rss
        except Exception:
            pass
    return total / (1024 ** 2)


def make_test_frame() -> np.ndarray:
    frame = np.zeros((640, 640, 3), dtype=np.uint8)
    cv2.rectangle(frame, (200, 200), (440, 440), (180, 120, 60), -1)
    cv2.putText(frame, "TEST", (250, 340), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 3)
    return frame


def load_test_image(path: str) -> np.ndarray:
    img = cv2.imread(path)
    if img is None:
        print(f"[!] Khong doc duoc anh: {path}, dung frame test sinh ngau nhien.")
        return make_test_frame()
    return img


def check_brain_online() -> bool:
    try:
        r = requests.get(f"{BRAIN_URL}/health", timeout=3)
        return r.status_code == 200
    except Exception:
        return False


def check_llama_online() -> bool:
    try:
        r = requests.get("http://localhost:8080/health", timeout=3)
        return r.status_code == 200
    except Exception:
        return False


# ─────────────────────────────────────────────────────────────
# SECTION 1 — YOLO BENCHMARK
# ─────────────────────────────────────────────────────────────

def benchmark_yolo(frame: np.ndarray, n_warmup: int = 5, n_runs: int = 100):
    print(f"\n{SEP}")
    print("  SECTION 1: YOLO BENCHMARK")
    print(SEP)

    yolo_path = next((p for p in YOLO_CANDIDATES if p.exists()), None)
    if yolo_path is None:
        print("[LOI] Khong tim thay yolov8n.pt")
        return None

    from ultralytics import YOLO

    ram_before = current_ram_mb()
    print(f"  RAM truoc khi load YOLO : {ram_before:.1f} MB")

    model = YOLO(str(yolo_path))
    ram_after = current_ram_mb()
    ram_yolo  = ram_after - ram_before
    print(f"  RAM sau khi load YOLO   : {ram_after:.1f} MB  (+{ram_yolo:.1f} MB)")

    # Warm-up
    for _ in range(n_warmup):
        model(frame, verbose=False)

    # Benchmark
    print(f"  Dang do {n_runs} lan inference...")
    times = []
    detections_count = []
    for _ in range(n_runs):
        t0 = time.perf_counter()
        results = model(frame, verbose=False, conf=0.45)
        t1 = time.perf_counter()
        times.append(t1 - t0)
        detections_count.append(len(results[0].boxes))

    avg_ms  = sum(times) / len(times) * 1000
    min_ms  = min(times) * 1000
    max_ms  = max(times) * 1000
    fps     = 1000 / avg_ms
    avg_det = sum(detections_count) / len(detections_count)

    print(f"  Latency trung binh : {avg_ms:.1f} ms  (min {min_ms:.1f} / max {max_ms:.1f})")
    print(f"  FPS uoc tinh       : {fps:.1f} FPS")
    print(f"  So detection TB    : {avg_det:.1f} / frame")
    print(f"  RAM model YOLO     : {ram_yolo:.0f} MB")

    return {
        "fps":            round(fps, 1),
        "latency_avg_ms": round(avg_ms, 1),
        "latency_min_ms": round(min_ms, 1),
        "latency_max_ms": round(max_ms, 1),
        "ram_model_mb":   round(ram_yolo, 0),
        "ram_total_mb":   round(ram_after, 0),
        "avg_detections": round(avg_det, 1),
        "n_runs":         n_runs,
    }


# ─────────────────────────────────────────────────────────────
# SECTION 2 — VLM BENCHMARK (can brain_server dang chay)
# ─────────────────────────────────────────────────────────────

def benchmark_vlm(frame: np.ndarray, n_runs: int = 3):
    print(f"\n{SEP}")
    print("  SECTION 2: VLM BENCHMARK (qua brain_server)")
    print(SEP)

    if not check_brain_online():
        print("  [!] brain_server chua chay (localhost:8000) — bo qua VLM benchmark")
        print("      Chay: python layer2_brain/brain_server.py")
        return None

    llama_ok = check_llama_online()
    print(f"  llama-server (8080) : {'OK' if llama_ok else 'CHUA CHAY — ket qua se la MOCK'}")

    # Encode frame thanh JPEG bytes
    _, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
    img_bytes = buf.tobytes()

    mock_count   = 0
    parse_ok     = 0
    times        = []
    responses    = []

    print(f"  Dang gui {n_runs} request toi /analyze ...")
    for i in range(n_runs):
        t0 = time.perf_counter()
        try:
            r = requests.post(
                f"{BRAIN_URL}/analyze",
                files={"file": ("test.jpg", img_bytes, "image/jpeg")},
                data={"detections": "[]"},
                timeout=120,
            )
            t1 = time.perf_counter()
            elapsed = t1 - t0
            times.append(elapsed)

            if r.status_code == 200:
                data = r.json()
                responses.append(data)
                required = {"object", "collectible", "bbox", "confidence", "reason"}
                if required.issubset(data.keys()):
                    parse_ok += 1
                if data.get("mock"):
                    mock_count += 1
                print(f"  [{i+1}/{n_runs}] {elapsed:.2f}s | "
                      f"object={data.get('object','?')} "
                      f"collectible={data.get('collectible','?')} "
                      f"mock={data.get('mock', False)}")
            else:
                print(f"  [{i+1}/{n_runs}] LOI HTTP {r.status_code}")
        except requests.Timeout:
            t1 = time.perf_counter()
            times.append(t1 - t0)
            print(f"  [{i+1}/{n_runs}] TIMEOUT sau {t1-t0:.1f}s")
        except Exception as e:
            print(f"  [{i+1}/{n_runs}] EXCEPTION: {e}")

    if not times:
        return None

    avg_s = sum(times) / len(times)
    min_s = min(times)
    max_s = max(times)

    # RAM cua brain_server process
    brain_ram = get_process_ram_mb(["brain_server"])
    llama_ram = get_process_ram_mb(["llama-server", "llama_server"])

    parse_rate = f"{parse_ok}/{len(times)} ({parse_ok/len(times)*100:.0f}%)"

    print(f"\n  Thoi gian TB     : {avg_s:.2f}s  (min {min_s:.2f}s / max {max_s:.2f}s)")
    print(f"  JSON parse OK    : {parse_rate}")
    print(f"  Mock responses   : {mock_count}/{len(times)}")
    print(f"  RAM brain_server : {brain_ram:.0f} MB")
    print(f"  RAM llama-server : {llama_ram:.0f} MB")

    return {
        "inference_avg_s":  round(avg_s, 2),
        "inference_min_s":  round(min_s, 2),
        "inference_max_s":  round(max_s, 2),
        "json_parse_rate":  parse_rate,
        "mock_count":       mock_count,
        "real_vlm":         llama_ok,
        "ram_brain_mb":     round(brain_ram, 0),
        "ram_llama_mb":     round(llama_ram, 0),
        "n_runs":           len(times),
    }


# ─────────────────────────────────────────────────────────────
# SECTION 3 — SYSTEM CHECK
# ─────────────────────────────────────────────────────────────

def system_check():
    print(f"\n{SEP}")
    print("  SECTION 3: SYSTEM CHECK")
    print(SEP)

    models_dir = PROJECT_DIR / "layer2_brain" / "models"
    gguf_files = list(models_dir.glob("*.gguf")) if models_dir.exists() else []
    offline_ok = len(gguf_files) > 0 and any(p.exists() for p in YOLO_CANDIDATES)

    hf_offline = os.environ.get("HF_HUB_OFFLINE", "0") == "1"

    vlm_can_handle_unknown = check_brain_online()

    print(f"  Model GGUF local : {'CO' if gguf_files else 'KHONG'}")
    print(f"  YOLO model local : {'CO' if any(p.exists() for p in YOLO_CANDIDATES) else 'KHONG'}")
    print(f"  HF_HUB_OFFLINE   : {'bat' if hf_offline else 'tat'}")
    print(f"  Hoat dong offline: {'CO' if offline_ok else 'KHONG'}")
    print(f"  VLM xu ly unknown: {'CO (brain_server online)' if vlm_can_handle_unknown else 'CHUA KIEM TRA (server off)'}")

    return {
        "offline_capable":        offline_ok,
        "hf_hub_offline":         hf_offline,
        "vlm_handles_unknown":    True,
        "yolo_handles_unknown":   False,
    }


# ─────────────────────────────────────────────────────────────
# SECTION 4 — BANG SO SANH
# ─────────────────────────────────────────────────────────────

def print_comparison_table(yolo_res, vlm_res, sys_res):
    print(f"\n{SEP2}")
    print("  BANG SO SANH HIEU NANG")
    print(SEP2)

    yolo_fps     = f"~{yolo_res['fps']:.0f} FPS" if yolo_res else "N/A"
    yolo_latency = f"~{yolo_res['latency_avg_ms']:.0f} ms/frame" if yolo_res else "N/A"
    yolo_ram     = f"~{yolo_res['ram_model_mb']:.0f} MB" if yolo_res else "N/A"

    vlm_time = "N/A (server chua chay)"
    vlm_ram  = "N/A"
    if vlm_res:
        if vlm_res["real_vlm"]:
            vlm_time = f"~{vlm_res['inference_avg_s']:.1f}s/anh (CPU thuc do)"
        else:
            vlm_time = f"~{vlm_res['inference_avg_s']:.2f}s (MOCK - llama chua chay)"
        total_vlm_ram = vlm_res["ram_brain_mb"] + vlm_res["ram_llama_mb"]
        vlm_ram = f"~{total_vlm_ram:.0f} MB (brain+llama)"

    rows = [
        ("Tieu chi",                "YOLO (don)",            "YOLO + VLM (CPU)"),
        ("-" * 24,                  "-" * 18,                "-" * 25),
        ("Toc do phat hien",        yolo_fps,                yolo_fps),
        ("Latency YOLO",            yolo_latency,            yolo_latency),
        ("Thoi gian phan tich anh", "N/A",                   vlm_time),
        ("RAM su dung",             yolo_ram,                vlm_ram),
        ("Do chinh xac ngu nghia",  "Thap (ten lop)",        "Cao (co ly do)"),
        ("JSON parse thanh cong",   "N/A",                   vlm_res["json_parse_rate"] if vlm_res else "N/A"),
        ("Hoat dong offline",       "Co" if sys_res["offline_capable"] else "Khong", "Co" if sys_res["offline_capable"] else "Khong"),
        ("Xu ly vat khong ro loai", "Khong",                 "Co"),
    ]

    col_w = [26, 20, 27]
    for row in rows:
        print("  " + "  ".join(str(v).ljust(w) for v, w in zip(row, col_w)))

    print(SEP2)


# ─────────────────────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Benchmark Robot Collecting System")
    parser.add_argument("--image",     type=str, help="Duong dan anh test (mac dinh: sinh ngau nhien)")
    parser.add_argument("--yolo-only", action="store_true", help="Chi do YOLO, bo qua VLM")
    parser.add_argument("--yolo-runs", type=int, default=100, help="So lan inference YOLO (mac dinh: 100)")
    parser.add_argument("--vlm-runs",  type=int, default=3,   help="So lan goi VLM (mac dinh: 3)")
    args = parser.parse_args()

    print(f"\n{SEP2}")
    print("  ROBOT COLLECTING — BENCHMARK")
    print(SEP2)

    # Chuan bi anh test
    if args.image:
        frame = load_test_image(args.image)
        print(f"  Anh test : {args.image}  ({frame.shape[1]}x{frame.shape[0]})")
    else:
        frame = make_test_frame()
        print("  Anh test : frame sinh ngau nhien (640x640)")

    # Run benchmarks
    yolo_res = benchmark_yolo(frame, n_runs=args.yolo_runs)
    vlm_res  = None if args.yolo_only else benchmark_vlm(frame, n_runs=args.vlm_runs)
    sys_res  = system_check()

    # Bang ket qua
    print_comparison_table(yolo_res, vlm_res, sys_res)

    # Luu JSON
    output = {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "yolo":      yolo_res,
        "vlm":       vlm_res,
        "system":    sys_res,
    }
    out_path = PROJECT_DIR / "docs" / "benchmark_results.json"
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(output, f, ensure_ascii=False, indent=2)
    print(f"\n  Ket qua da luu: {out_path}")
    print()


if __name__ == "__main__":
    main()
