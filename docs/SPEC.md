# SPEC — Tích hợp Orbbec Astra 3D Depth Camera

**Version:** 1.0
**Date:** 2026-05-25
**Status:** Draft — chờ approval
**Owner:** Robot Collecting Team

---

## 1. Objective

### 1.1 Mục tiêu
Thay thế webcam USB hiện tại (`cv2.VideoCapture(0)`) bằng camera **Orbbec Astra (depth + RGB)** trong [layer1_vision/vision_node.py](layer1_vision/vision_node.py), và sử dụng dữ liệu depth để **loại bỏ giả định cứng** trong [layer3_control/kinematics.py](layer3_control/kinematics.py:34) (`CAMERA_HEIGHT = 300mm`).

### 1.2 Vấn đề đang giải quyết
| # | Vấn đề hiện tại | Giải pháp với depth |
|---|---|---|
| 1 | `pixel_to_world()` giả định camera cao **đúng 300mm** so với bàn — sai số lớn khi camera lệch | Unproject pixel → (X, Y, Z) **thực** dùng depth & intrinsics |
| 2 | Không biết chiều cao vật thể → gripper hạ tay sai độ cao, gắp hụt hoặc đụng bàn | Median depth trong bbox → Z chính xác |
| 3 | Không phát hiện vật cản trên đường tay đi | Ray-march depth dọc trajectory → cảnh báo collision |
| 4 | Webcam độ phân giải thấp + không có depth → kinematics chỉ là 2D approximation | RGB + Depth aligned frame từ Astra |

### 1.3 Non-goals (out of scope cho spec này)
- ❌ Point cloud processing / Open3D / ICP
- ❌ 6-DOF grasp pose estimation (chỉ top-down grasp)
- ❌ Hand-eye calibration tự động (manual config file)
- ❌ Multi-camera fusion
- ❌ ROS integration

### 1.4 Success criteria
- ✅ `vision_node.py` chạy được với Astra trên **Jetson AGX Xavier (L4T)** ở ≥15 FPS
- ✅ YOLO detection chạy trên Astra RGB stream (640×480 hoặc 1280×720)
- ✅ `kinematics.calculate_angles()` nhận `depth_frame` optional, trả về Z thực ±10mm so với ground truth
- ✅ Pipeline degrade gracefully khi Astra disconnect (log error, không crash)
- ✅ `--mock-camera` flag cho dev offline (load RGB + depth từ file `.npz`)

---

## 2. Architecture & Design

### 2.1 Camera abstraction layer (mới)
Thay vì hardcode `cv2.VideoCapture` trong `vision_node.py`, tách thành interface:

```
layer1_vision/
└── cameras/
    ├── __init__.py
    ├── base.py              # Camera ABC — interface chung
    ├── webcam.py            # Wrap cv2.VideoCapture (giữ làm reference)
    ├── astra.py             # Orbbec Astra implementation
    └── mock.py              # Load RGB+depth từ .npz cho test offline
```

**`Camera` ABC** (`base.py`) trả về `Frame` dataclass:

```python
@dataclass
class Frame:
    rgb: np.ndarray              # H×W×3 uint8 BGR
    depth_mm: np.ndarray | None  # H×W float32, đơn vị mm, NaN = invalid
    timestamp: float             # time.time() khi capture
    intrinsics: CameraIntrinsics | None  # fx, fy, cx, cy
```

```python
class Camera(ABC):
    @abstractmethod
    def open(self) -> None: ...
    @abstractmethod
    def read(self) -> Frame | None: ...
    @abstractmethod
    def close(self) -> None: ...
    @property
    @abstractmethod
    def has_depth(self) -> bool: ...
```

### 2.2 Orbbec Astra implementation
Vì model cụ thể chưa được chốt, `astra.py` viết để **hỗ trợ cả 2 SDK paths**:

| SDK | Khi nào dùng | Wheel |
|---|---|---|
| **pyorbbecsdk** (Orbbec SDK 2.x) | Astra+, Femto, Gemini | PyPI `pyorbbecsdk` hoặc build từ source |
| **OpenNI2** (legacy) | Astra Pro, Astra Mini | `primesense` + `libopenni2` system lib |

Strategy: try `pyorbbecsdk` trước, fallback `openni2`. Selection qua config (`config.json`):

```json
{
  "camera": {
    "driver": "orbbec_sdk",   // "orbbec_sdk" | "openni2" | "webcam" | "mock"
    "rgb_resolution": [640, 480],
    "depth_resolution": [640, 480],
    "depth_align_to_rgb": true,
    "depth_min_mm": 200,
    "depth_max_mm": 2000
  }
}
```

### 2.3 Pipeline impact
```
┌────────────────────────────────────────────────────────────────┐
│  vision_node.py                                                │
│                                                                │
│  Camera (Astra)                                                │
│      │                                                         │
│      ▼  Frame { rgb, depth_mm, intrinsics }                   │
│  YOLO(rgb) → detections [bbox]                                │
│      │                                                         │
│      ▼  + depth crop trong bbox                               │
│  StableTrigger                                                 │
│      │                                                         │
│      ▼  POST /analyze  (multipart: rgb.jpg + depth.npy)       │
│  Brain (Layer 2)  ← KHÔNG đổi (depth passthrough)             │
│      │                                                         │
│      ▼  collectible=true + bbox                               │
│  POST /execute  { bbox, depth_metadata: { median_z_mm, ... }} │
│      │                                                         │
│      ▼                                                         │
│  Control (Layer 3) → kinematics.calculate_angles(             │
│      bbox, depth_frame=...,  # optional                       │
│      intrinsics=...                                            │
│  )                                                             │
└────────────────────────────────────────────────────────────────┘
```

### 2.4 Kinematics changes
[layer3_control/kinematics.py](layer3_control/kinematics.py) — backward compatible:

```python
def pixel_to_world(
    x, y, w, h,
    depth_frame: np.ndarray | None = None,  # MỚI
    intrinsics: CameraIntrinsics | None = None,  # MỚI
    frame_w=CAMERA_RES_W,
    frame_h=CAMERA_RES_H,
) -> dict:
    if depth_frame is not None and intrinsics is not None:
        return _unproject_with_depth(x, y, w, h, depth_frame, intrinsics)
    return _legacy_pinhole_approx(x, y, w, h, frame_w, frame_h)  # code cũ
```

**Collision check** (mới):
```python
def check_path_clear(
    start_xyz_mm: tuple, end_xyz_mm: tuple,
    depth_frame: np.ndarray, intrinsics: CameraIntrinsics,
    clearance_mm: float = 30.0,
) -> bool:
    """Sample N điểm dọc segment, project xuống pixel, so với depth."""
```

### 2.5 Data flow giữa layers (depth metadata)
Để **không phải gửi full depth frame qua HTTP** (latency), Layer 1 tự crop & compute metadata trước khi gửi Layer 3:

```json
{
  "object": "bottle",
  "collectible": true,
  "bbox": [120, 80, 200, 300],
  "confidence": 0.87,
  "depth_metadata": {
    "median_z_mm": 425.3,
    "min_z_mm": 410.1,
    "max_z_mm": 480.5,
    "valid_ratio": 0.92
  },
  "intrinsics": { "fx": 525.0, "fy": 525.0, "cx": 319.5, "cy": 239.5 }
}
```

Cho collision check: Layer 1 gửi thêm 1 depth slice nhỏ (downsampled 80×60 float16 ≈ 9KB) qua field riêng nếu cần.

---

## 3. Commands & Workflows

### 3.1 Install Orbbec SDK trên Jetson
```bash
# Option A: pyorbbecsdk (recommended cho Astra+/Femto/Gemini)
sudo apt install -y libusb-1.0-0-dev libudev-dev
pip install pyorbbecsdk

# Option B: OpenNI2 (legacy Astra)
sudo apt install -y libopenni2-0 libopenni2-dev
pip install primesense

# udev rules (cả 2 option đều cần)
sudo cp tools/orbbec/99-orbbec-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### 3.2 Verify camera
```bash
python tools/orbbec/verify_astra.py
# Output mong đợi:
#   ✓ Astra device found: serial=XXX, firmware=YYY
#   ✓ Color stream:  640x480 @ 30 FPS
#   ✓ Depth stream:  640x480 @ 30 FPS, range 200-8000 mm
#   ✓ Aligned:       True
#   ✓ Intrinsics:    fx=525.0 fy=525.0 cx=319.5 cy=239.5
```

### 3.3 Run pipeline với Astra
```bash
# Full Astra (mới — mặc định khi config.driver=orbbec_sdk)
python layer1_vision/vision_node.py

# Force webcam (rollback)
python layer1_vision/vision_node.py --camera webcam

# Test offline với depth recording
python layer1_vision/vision_node.py --camera mock --recording tools/orbbec/sample_recording.npz

# Test single frame
python layer1_vision/vision_node.py --image test.jpg --depth test_depth.npy --mock-brain
```

### 3.4 Calibration workflow (manual)
```bash
# 1. Capture 20 ảnh checkerboard với cả RGB + Depth
python tools/orbbec/capture_calibration_set.py --output calibration/

# 2. Compute intrinsics + hand-eye transform
python tools/orbbec/calibrate.py --input calibration/ --output config/camera_calibration.json

# 3. Verify
python tools/orbbec/verify_calibration.py --config config/camera_calibration.json
```

---

## 4. Project Structure (sau khi impl)

```
Robot_Collecting_Project/
├── layer1_vision/
│   ├── vision_node.py              # ← CẬP NHẬT: dùng Camera ABC
│   ├── cameras/                    # ← MỚI
│   │   ├── __init__.py             # factory: get_camera(driver_name)
│   │   ├── base.py                 # Camera ABC + Frame + CameraIntrinsics
│   │   ├── webcam.py               # OpenCV VideoCapture wrap
│   │   ├── astra.py                # Orbbec implementation (pyorbbecsdk + openni2)
│   │   └── mock.py                 # Load .npz recordings
│   └── depth_utils.py              # ← MỚI: extract_bbox_depth_metadata()
│
├── layer2_brain/
│   └── brain_server.py             # ← MINOR: passthrough depth_metadata
│
├── layer3_control/
│   ├── kinematics.py               # ← CẬP NHẬT: depth-aware pixel_to_world
│   └── control_node.py             # ← MINOR: accept depth_metadata
│
├── tools/orbbec/                   # ← MỚI
│   ├── verify_astra.py             # Sanity check camera
│   ├── capture_calibration_set.py
│   ├── calibrate.py                # OpenCV chessboard calibration
│   ├── verify_calibration.py
│   ├── record_session.py           # Save RGB+depth → .npz cho test
│   ├── 99-orbbec-libusb.rules      # udev rules cho Jetson
│   └── sample_recording.npz        # Sample data (gitignored if >10MB)
│
├── config/                         # ← MỚI
│   └── camera_calibration.json     # Intrinsics + hand-eye transform
│
├── layer2_brain/config.json        # ← CẬP NHẬT: thêm section "camera"
├── requirements.txt                # ← CẬP NHẬT: thêm pyorbbecsdk
└── docs/SPEC.md                    # this file
```

---

## 5. Code Style & Conventions

### 5.1 Match existing project style
- **Bilingual docstrings**: Vietnamese cho user-facing logs, English cho type hints / docstrings code-facing
- **Logging format**: `[CAMERA]`, `[KINEMATICS]` prefix giống current pattern trong vision_node.py:56
- **Type hints**: dùng `from __future__ import annotations` (Python 3.8+ compat)
- **Path handling**: `pathlib.Path` không string concat
- **Error handling**: log + return `None` cho recoverable, raise cho fatal (giống current `init_camera` trong vision_node.py:69)

### 5.2 Camera-specific conventions
- **Depth units**: **luôn là mm float32**, NaN cho invalid pixels (không 0 — vì 0 có thể là valid trong tương lai)
- **Intrinsics**: dataclass frozen, immutable sau khi load từ config
- **Frame timestamps**: `time.time()` (float seconds), không `datetime`
- **Coordinate system**:
  - Pixel: (u, v) — u=cột (x), v=hàng (y), gốc top-left
  - Camera frame: X-right, Y-down, Z-forward (OpenCV convention)
  - World frame: định nghĩa qua hand-eye calibration, mặc định = camera frame

### 5.3 Cấm
- ❌ Không hardcode device serial / index trong code — luôn qua config
- ❌ Không dùng `cv2.VideoCapture(astra_uvc_index)` cho RGB — phải qua SDK để đảm bảo sync với depth
- ❌ Không dùng numpy `np.float64` cho depth — quá nặng cho Jetson, dùng `float32`

---

## 6. Testing Strategy

### 6.1 Unit tests (mới — `tests/`)
```
tests/
├── test_cameras_base.py        # Frame dataclass, ABC contract
├── test_cameras_mock.py        # Load .npz, replay
├── test_depth_utils.py         # extract_bbox_depth_metadata edge cases
├── test_kinematics_depth.py    # pixel_to_world with/without depth
└── test_collision_check.py     # check_path_clear sampling
```

Coverage target: **≥80% cho `cameras/`, `depth_utils.py`, `kinematics.py`** (existing code chưa có test, đây là chance để bootstrap).

### 6.2 Test fixtures (dùng `pytest`)
- **Sample .npz recording**: 30 frames RGB+depth của 1 cảnh có chai nhựa — cho integration test offline
- **Mocked intrinsics**: standard pinhole `fx=fy=525, cx=320, cy=240`
- **Ground truth**: 5 test cases với (bbox, depth, expected world_xyz) đã đo tay

### 6.3 Integration test (cần hardware)
```bash
pytest tests/integration/test_astra_pipeline.py --hardware
# Skip nếu không có Astra plug vào
```

Test scenarios:
1. Camera open → read 100 frames → close (no leaks, FPS ≥15)
2. Place bottle 30cm trước camera → YOLO detect → kinematics ra Z trong [280, 320] mm
3. Disconnect cable mid-stream → camera.read() trả None, không crash

### 6.4 Manual verification checklist
- [ ] `tools/orbbec/verify_astra.py` pass trên Jetson AGX
- [ ] Vision node chạy 1 phút không leak (monitor `nvidia-smi` + `htop`)
- [ ] So depth từ Astra với thước kẻ thực tế ở 3 khoảng cách (200, 500, 1000mm) — sai số <5%
- [ ] Toggle giữa `--camera astra` và `--camera webcam` không cần rebuild

---

## 7. Boundaries

### 7.1 ✅ Always do
- **Always** đóng camera trong `finally` block (giống pattern `cap.release()` ở vision_node.py:403)
- **Always** validate depth value trước khi unproject (NaN check, range check `depth_min_mm` ≤ d ≤ `depth_max_mm`)
- **Always** log camera serial + firmware khi open, để debug khi swap hardware
- **Always** backward compatible: `pixel_to_world()` cũ vẫn chạy khi không có depth
- **Always** version config schema — thêm `"version": "1.0"` vào `camera_calibration.json`

### 7.2 ⚠️ Ask first
- ⚠️ Đổi default driver từ `webcam` → `orbbec_sdk` trong config gốc (impact: ai pull repo mới sẽ crash nếu chưa có Astra)
- ⚠️ Bump Python version requirement (nếu pyorbbecsdk yêu cầu 3.10+)
- ⚠️ Đổi REST API contract giữa Layer 2 ↔ Layer 3 (thêm `depth_metadata`) — cần coordinate với người đang dùng API
- ⚠️ Remove webcam.py code path (chỉ làm sau khi Astra ổn định production ≥2 tuần)
- ⚠️ Bake calibration values vào code (luôn phải qua config file)

### 7.3 ❌ Never
- ❌ **Never** assume Astra always connected — luôn check `camera.is_open` trước read
- ❌ **Never** stream raw depth qua HTTP (~600KB/frame × 30 FPS = 18 MB/s — kill mạng). Chỉ gửi metadata + optional slice nhỏ
- ❌ **Never** dùng `from orbbec import *` — explicit imports để track dependency
- ❌ **Never** commit `.npz` recordings >10MB (dùng git-lfs hoặc external storage)
- ❌ **Never** hardcode device path `/dev/video0` — Orbbec multi-device cùng host sẽ break
- ❌ **Never** block main thread chờ depth — nếu depth chậm hơn RGB, dùng latest depth có (timestamp-based sync)

---

## 8. Risks & Mitigations

| Risk | Severity | Mitigation |
|---|---|---|
| Astra model chốt sau spec → SDK choice sai | High | Camera ABC + 2 implementations (orbbec_sdk + openni2) → swap không sửa caller |
| pyorbbecsdk không có wheel cho Jetson ARM64 | High | Verify trước impl. Fallback: build từ source (~30 phút trên Jetson) |
| Depth noise làm collision check false-positive | Medium | Median filter 3×3 trước khi sample. Threshold clearance_mm tunable |
| Frame sync drift RGB ↔ Depth | Medium | Dùng `depth_align_to_rgb=True` trong SDK config (HW-aligned), không sync software |
| FPS drop dưới 15 trên Jetson khi cả YOLO + depth | Medium | Profile sớm. Optional: chỉ chạy depth khi YOLO trigger, không every frame |
| Hand-eye calibration sai → tay robot trật vị trí thực tế | High | Verification script bắt buộc trước go-live. Sai số <±10mm ở 3 test points |

---

## 9. Open questions (cần resolve trước impl)

1. **Astra model cụ thể** — sẽ quyết định SDK chính (pyorbbecsdk hay openni2). Ảnh hưởng requirements.txt và install script.
2. **Robot arm coordinate frame** — gốc tọa độ robot ở đâu so với camera? Cần đo tay hoặc dùng marker.
3. **Bao nhiêu FPS đủ?** — Pipeline hiện tại bottleneck ở VLM (4-8s), nên camera 15 FPS có khi đã quá đủ. Confirm để tối ưu power.
4. **Có cần record session để debug không?** — Nếu có, cần thêm tool `record_session.py` (đã list ở section 4).

---

## 10. Approval

- [ ] User confirms objective & non-goals
- [ ] User confirms abstraction layer approach (Camera ABC)
- [ ] User confirms data contract giữa Layer 1 ↔ Layer 3 (depth_metadata JSON)
- [ ] User confirms test strategy includes integration test với hardware

**Khi approved → chạy `/plan` để break thành tasks ordered.**
