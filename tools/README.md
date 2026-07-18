# tools — Công cụ Astra & hạ tầng demo

Công cụ đo/hiệu chỉnh camera Orbbec Astra (SPIKE Phase 0), chụp dataset đánh giá, dựng mạng WiFi AP, và cứu hộ demo. Chạy từ **thư mục gốc** repo.

> Astra cần driver Orbbec OpenNI2 fork ở `tools/orbbec/openni2/` — xem [../layer1_vision/README.md](../layer1_vision/README.md).

---

## Camera Astra — đo & hiệu chỉnh

| Tool | Mục đích |
|---|---|
| `test_astra.py` | Test kết nối Astra (Orbbec OpenNI2 qua ctypes) |
| `astra_stream.py` | Stream MJPEG lên trình duyệt — xem live qua SSH (không cần màn hình) |
| `astra_coord_demo.py` | Realtime: phát hiện khối gần nhất + in tọa độ (X,Y,Z) mm + FPS |
| `astra_depth_range_report.py` | Đo tầm depth thực + FOV → quyết định viability SLAM (T0 SPIKE) |
| `astra_accuracy.py` | Đo sai số tọa độ Z (điền bảng báo cáo) |
| `plot_astra_noise.py` | Vẽ nhiễu depth & %pixel hợp lệ theo cự ly (F21/F22) |
| `plot_trigger_timeline.py` | Vẽ timeline StableTrigger (F14) |

```bash
python tools/test_astra.py                 # kiểm tra camera
python tools/astra_stream.py               # xem live qua http
python tools/astra_depth_range_report.py   # báo cáo tầm depth
```

## Dataset đánh giá

```bash
python tools/capture_eval_dataset.py       # chụp ảnh Astra + tự sinh nhãn GT → data/eval_dataset/
```

## Hạ tầng demo (Jetson)

| Tool | Mục đích |
|---|---|
| `setup_hostapd.sh` | Dựng WiFi AP + DHCP + NAT trên Jetson (đế ESP32 nối vào). AP `192.168.137.1` |
| `demo_doctor.sh` | Cứu hộ khi demo T10 (detect → đi tới vật) gặp lỗi — chẩn đoán nhanh |

```bash
sudo bash tools/setup_hostapd.sh           # WAN_IF mặc định enp2s0
bash tools/demo_doctor.sh                   # khi demo lỗi
```

## Khác

- `orbbec/` — driver stack Orbbec OpenNI2 persistent (bắt buộc cho mọi tool/driver Astra).
- `agent-skills/` — bộ skill cho AI agent (không liên quan runtime robot).
