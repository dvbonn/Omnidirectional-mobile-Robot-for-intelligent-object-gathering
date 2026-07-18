# Layer 3 — Control Node

Điều khiển phần cứng tay gắp: nhận lệnh thu gom từ Brain (Layer 2) → tính **inverse kinematics** → gửi góc servo xuống board (Arduino/ESP32/STM32).

> ⚠️ **Placeholder.** Kinematics và điều khiển phần cứng hiện **mô phỏng bằng log** — chưa nối board thật. Các hằng số cánh tay và camera là giả định, cần chỉnh theo phần cứng thực tế. Firmware tay gắp thật ở [`Source_code/STM32_Robot_arm/`](../Source_code/STM32_Robot_arm/).

---

## Kiến trúc

```
POST /execute (object, collectible, bbox) ──▶ kinematics.calculate_angles()
                                                     │
                          collectible=false ─────────┤─▶ skipped
                          ngoài tầm với    ─────────┤─▶ out_of_reach
                                                     ▼
                                        simulate_hardware_control() → góc servo (log)
```

| File | Vai trò |
|---|---|
| `control_node.py` | **Entry point.** Flask server (cổng 8001), endpoint `/execute` |
| `kinematics.py` | `pixel_to_world()` + `calculate_angles()` — IK 3 khớp cánh tay |

---

## Setup

Chỉ cần `flask` (đã có trong `requirements.txt`). Không có bước tải model.

**Chỉnh theo phần cứng** — sửa hằng số đầu `kinematics.py`:
- `ARM_LINK_1/2/3` — chiều dài các đoạn cánh tay (mm)
- `SERVO_MIN/MAX` — giới hạn góc servo
- `CAMERA_FOV_H`, `CAMERA_HEIGHT` — thông số camera (pixel → mm). Khi có Astra, thay giả định `CAMERA_HEIGHT=300mm` bằng depth thật (xem [docs/SPEC.md](../docs/SPEC.md)).

---

## Chạy

```bash
python layer3_control/control_node.py     # cổng 8001

# Test thủ công:
curl -X POST http://localhost:8001/execute -H "Content-Type: application/json" \
  -d '{"object":"bottle","collectible":true,"bbox":[120,80,200,300],"confidence":0.87}'
```

### API — `http://localhost:8001`

| Endpoint | Method | Mô tả |
|---|---|---|
| `/execute` | POST | Nhận lệnh thu gom → tính góc → (mô phỏng) điều khiển |
| `/health` | GET | Health check |
| `/status` | GET | Trạng thái robot (`idle`/`processing`/`error`, task_count) |
| `/reset` | POST | Reset về vị trí home |

**Kết quả `/execute`** theo tình huống: `skipped` (không thu gom), `out_of_reach` (ngoài tầm), hoặc kết quả điều khiển kèm góc servo.
