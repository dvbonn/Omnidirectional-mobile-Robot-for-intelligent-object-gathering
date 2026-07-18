# Layer 1 — Vision Node

Thị giác nhanh: Camera (webcam **hoặc** Orbbec Astra) → **YOLOv8n** → logic trigger ổn định → gửi ảnh cho Brain (Layer 2). Khi dùng Astra, bổ sung **tọa độ 3D** từ depth.

Đây cũng là nơi chứa **driver Astra** và tiện ích depth được **dùng lại bởi stack ROS2** (`astra_ros`, `yolo_ros`).

---

## Kiến trúc

```
Camera ──▶ YOLOv8n ──▶ StableTrigger (ổn định ≥2s) ──▶ chụp ảnh ──▶ POST /analyze (Brain)
  │                                                        │
  └─ Astra depth ──▶ depth_utils.bbox_center_coord ──▶ tọa độ 3D (X,Y,Z) ──▶ detection_log (JSONL)
```

| File | Vai trò |
|---|---|
| `vision_node.py` | **Entry point.** Vòng lặp camera → YOLO → trigger → gửi Brain API |
| `cameras/astra_openni.py` | Driver Astra qua Orbbec OpenNI2 (ctypes). Mode `depth` / `color` / `both` |
| `cameras/test_grab_switch.py` | Test đổi mode depth↔color KHÔNG reopen device (chống treo USB2) |
| `depth_detect.py` | Phát hiện vật bằng depth thuần (tách khối gần nhất) — không cần YOLO |
| `depth_utils.py` | Unproject pixel → tọa độ 3D camera; lấy median depth trong bbox |
| `detection_log.py` | Ghi detection JSONL — cầu nối Vision → ROS2 (superset schema Brain) |
| `model/` | Model YOLO `.pt` (yolov8n tự tải lần đầu) |
| `temp_images/` | Ảnh tạm chụp khi trigger |

---

## Setup

Đã cài qua `requirements.txt` ở root (`opencv-python`, `ultralytics`). YOLO `yolov8n.pt` tự tải lần đầu (~6 MB); hoặc đặt sẵn file vào `model/`.

**Astra (tuỳ chọn)** — chỉ cần khi `--source astra`. Astra là thiết bị legacy, **chỉ chạy với Orbbec OpenNI2 fork** (không dùng system OpenNI2 hay pyorbbecsdk). Driver stack persistent phải nằm ở `tools/orbbec/openni2/` (chứa `libOpenNI2.so`). Xem `tasks/todo.md` P0.1.

> ⚠️ **USB 2.0**: Astra không stream depth + color đồng thời. Mode `both` phải toggle stop/start → chỉ ~2 FPS.

---

## Chạy

```bash
# Webcam + Brain API (mặc định)
python layer1_vision/vision_node.py

# Webcam, bỏ qua Brain API (in mock result)
python layer1_vision/vision_node.py --mock-brain

# Astra color + YOLO (~30 FPS)
python layer1_vision/vision_node.py --source astra

# Astra + YOLO + tọa độ 3D (~2 FPS, mode both)
python layer1_vision/vision_node.py --source astra --astra-3d

# Test ảnh tĩnh — hoàn toàn offline
python layer1_vision/vision_node.py --image test.jpg --mock-brain
```

### CLI flags

| Flag | Mặc định | Ý nghĩa |
|---|---|---|
| `--source {webcam,astra}` | `webcam` | Nguồn camera |
| `--astra-3d` | off | Bật depth → tọa độ 3D (mode `both`) |
| `--image PATH` | — | Chạy trên 1 ảnh tĩnh thay vì camera |
| `--mock-brain` | off | Bỏ qua Brain API, in mock result |
| `--headless` | off | Không mở cửa sổ OpenCV |
| `--frames N` | 300 | Số frame chạy rồi dừng (dùng cho benchmark) |
| `--trigger-log PATH` | — | Ghi timeline trigger ra file |
