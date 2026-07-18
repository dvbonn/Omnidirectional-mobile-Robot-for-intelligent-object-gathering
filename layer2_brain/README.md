# Layer 2 — Brain Server

Bộ não suy luận: **FastAPI** nhận ảnh từ Layer 1 → gọi **Qwen2.5‑VL** (qua `llama.cpp`) → trả JSON quyết định *"vật này có thu gom được không?"*.

VLM chạy như một tiến trình **riêng** (`llama-server` cổng 8080). Brain Server (cổng 8000) là lớp mỏng: nhận ảnh, dựng prompt theo **chính sách thu gom**, gọi VLM, parse JSON. Nếu VLM chưa bật → trả **mock response** (không crash).

---

## Kiến trúc

```
POST /analyze (ảnh) ──▶ brain_server ──▶ llama-server :8080 (Qwen2.5-VL GGUF)
                            │                   │
                            │◀── JSON ──────────┘
                            ▼
{ object, collectible, bbox, confidence, reason }
```

| File | Vai trò |
|---|---|
| `brain_server.py` | **Entry point.** FastAPI server, endpoint `/analyze`, `/health` |
| `setup_vlm.py` | Script cài đặt: tải model + llama.cpp + sinh script khởi chạy |
| `config.json` | Cấu hình model, GPU layers, port, prompt (chính sách thu gom) |
| `models/` | Model GGUF: `Qwen2.5-VL-3B-Instruct-Q4_K_M.gguf` + `mmproj-*.gguf` |
| `uploads/` | Ảnh tạm nhận từ Layer 1 (tự xóa) |

---

## Setup

```bash
# Từ thư mục gốc — tải model + llama.cpp + sinh script khởi chạy
python layer2_brain/setup_vlm.py
```

Script tự: kiểm tra prerequisites (git/cmake/python) → tải model (~2–3 GB) vào `models/` → phát hiện/build `llama-server` → sinh `scripts/start_vlm_server.bat`. Yêu cầu Python 3.8+ và Git.

---

## Chạy & test

```bash
# T1 — VLM engine (bắt buộc để có kết quả THẬT)
.\scripts\start_vlm_server.bat        # chờ: "llama server listening at http://127.0.0.1:8080"

# T2 — Brain API
python layer2_brain/brain_server.py
```

Test bằng Swagger UI: **http://localhost:8000/docs** → `POST /analyze` → upload ảnh → `Execute`.

> Nếu `llama-server` chưa chạy, `/health` báo `llama_cpp: "disconnected"` và `/analyze` trả `"mock": true` — đây là hành vi bình thường khi dev.

### API

| Endpoint | Method | Mô tả |
|---|---|---|
| `/` | GET | Redirect → Swagger UI |
| `/health` | GET | Trạng thái server + llama.cpp |
| `/analyze` | POST | Nhận ảnh (multipart) → JSON quyết định |
| `/docs` | GET | Swagger UI |

**Response `/analyze`:**
```json
{
  "object": "bottle",
  "collectible": true,
  "bbox": [120, 80, 200, 300],
  "confidence": 0.87,
  "reason": "Chai nhựa có thể thu gom được",
  "processing_time_s": 4.2
}
```

---

## Cấu hình (`config.json`)

```jsonc
{
  "vlm": {
    "context_size": 2048,   // Tăng nếu RAM/VRAM đủ
    "gpu_layers": 99,        // 0 = CPU-only; 99 = offload tối đa lên GPU (CUDA)
    "temperature": 0.3,      // 0 = deterministic
    "max_tokens": 256,
    "threads": 4
  },
  "llama_cpp_server": { "host": "127.0.0.1", "port": 8080 },
  "brain_server":     { "host": "0.0.0.0",   "port": 8000 },
  "prompt": { "system": "...chính sách thu gom...", "user_template": "...format JSON..." }
}
```

**Chính sách thu gom** (trong `prompt.system`) — VLM quyết định `collectible`:
- `true`: rác tái chế / đồ dùng một lần (chai nhựa, ly nhựa, hộp giấy, lon…)
- `false`: đồ có giá trị (điện thoại, ví, điện tử) hoặc dễ vỡ/nguy hiểm (thủy tinh, sành sứ, dao kéo)
