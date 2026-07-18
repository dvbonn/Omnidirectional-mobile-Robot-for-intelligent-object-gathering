# scripts — Vận hành, benchmark & kiểm thử

Script khởi chạy VLM server, kiểm tra môi trường, và đo hiệu năng pipeline (số liệu Chương 4). Chạy từ **thư mục gốc** repo.

---

## Vận hành

| Script | Mục đích |
|---|---|
| `start_vlm_server.bat` | Khởi chạy `llama-server` cho Qwen2.5‑VL (Windows). Tự sinh bởi `setup_vlm.py` |
| `start_vlm_server.sh` | Như trên cho Linux/Jetson (CUDA) |
| `verify_setup.py` | Kiểm tra toàn bộ setup trước khi chạy (exit 0 = OK) |

```bash
python scripts/verify_setup.py        # kiểm tra file/model/deps
.\scripts\start_vlm_server.bat        # Windows — cổng 8080
bash scripts/start_vlm_server.sh      # Jetson/Linux
```

---

## Benchmark & đánh giá

| Script | Đo gì |
|---|---|
| `benchmark.py` | Hiệu năng pipeline end‑to‑end (`--yolo-only` để chỉ đo YOLO) |
| `bench_yolo.py` | YOLOv8n cho 1 device (cpu/cuda) — chạy riêng process để đo RAM sạch |
| `bench_vlm_config.py` | Qwen2.5‑VL Q4_K_M cho 1 cấu hình `n_gpu_layers` |
| `probe_vram.py` | VRAM llama‑server cấp phát theo từng `n_gpu_layers` |
| `eval_accuracy.py` | Accuracy "collectibility" — so YOLO‑only vs VLM trên `data/eval_dataset/` |
| `plot_benchmarks.py` | Vẽ hình từ kết quả benchmark → `docs/figures/` |
| `plot_architecture.py` | Vẽ sơ đồ kiến trúc |

```bash
# Chỉ YOLO (không cần server):
python scripts/benchmark.py --yolo-only

# Đầy đủ (cần brain_server + llama-server đang chạy):
python scripts/benchmark.py

# Accuracy (tự khởi động llama-server):
python scripts/eval_accuracy.py
```

Kết quả ghi ra `docs/benchmark_results*.json` và `docs/bang_4_*.json`.
