# BÀN GIAO — Stack ROS2 cho robot thu gom (SLAM + NAV2 + YOLO detect → đi tới vật)

Tài liệu cho **partner** ghép phần cứng. Đọc theo thứ tự: hiểu workspace → biết mình đã sửa gì
trong code của bạn → nắm workflow demo → build → test theo thang bậc → cấu hình phần cứng.
Chi tiết vận hành: [DEMO_RUNBOOK.md](DEMO_RUNBOOK.md) · Kế hoạch đầy đủ: [PLAN_SLAM_NAV2.md §6](PLAN_SLAM_NAV2.md).

---

## 0. TL;DR
- Workspace `ros2_ws/` (ROS2 Foxy) — **7 package, build sạch**, làm xong toàn bộ phần **không cần phần cứng**.
- Đã có: cầu nối đế WiFi (đã harden), Astra→/scan, SLAM, NAV2, tự hành WFD, YOLO detect, và orchestrator "detect → đi tới vật".
- **Chỉ còn việc cần ĐẾ thật**: chạy live + đo kích thước + tune. Mọi thứ khác đã verify (unit-test + loopback + camera live).
- **Giữ nguyên `Source_code/`** (bản gốc của bạn) làm tham chiếu — mọi thay đổi của mình nằm trong `ros2_ws/`.

---

## 1. Cấu trúc workspace (7 package)
| Package | Node (executable) | Vai trò | Nguồn |
|---------|-------------------|---------|-------|
| **cpp_package** | `wifi_node`, `explorer_node` | Cầu nối đế WiFi (TCP server :2004) + tự hành WFD | **của bạn — mình đã harden `wifi_node`** (xem §2) |
| **astra_ros** | `astra_node`, `astra_color_node`, **`camera_manager`** | Camera Astra → ROS2. `camera_manager` = 1 node đổi mode depth/color/both theo `/camera_mode` | mình |
| **yolo_ros** | `yolo_node` | YOLOv8n: ảnh màu → `/yolo/image_annotated/compressed` + `/yolo/detections` | mình |
| **vlm_nav_orchestrator** | `orchestrator_node` | T10: detect → định vị vật → NAV2 đi tới → servo (state machine) | mình |
| **robot_bringup** | (launch + config) | Launch tổng + `slam_toolbox.yaml`/`nav2_params.yaml` (tune Astra) | mình |
| **robot_description** | (URDF/TF) | Mô hình robot + static TF | mình |
| **base_bridge** | `base_node`, `fake_odom` | **CHỈ tham chiếu**: `protocol.py` (spec byte-format + 13 unit-test) + `fake_odom` (test). `base_node` (TCP client) KHÔNG dùng — `wifi_node` thay thế | mình |

---

## 2. ⚠️ Mình đã sửa gì trong `wifi_node` của bạn (đọc kỹ)
Giao thức byte-format **giữ NGUYÊN** (đã đối chiếu `message_type.h`/`kinematic.h` từng byte). Chỉ vá an toàn + độ bền
trong `cpp_package/src/socket_handler.cpp` + `.h` (code-review tìm 3 BLOCKER → đã sửa + test lại):

| # | Sửa | Lý do |
|---|-----|-------|
| 1 | **Watchdog STOP** — `/cmd_vel` im > `cmd_timeout` (mặc định **1.5s**) → tự gửi `STOP_ROBOT` | Chống **xe chạy hoang** khi ROS/NAV2 treo (1.5s đủ rộng để không phanh-giật giữa lúc tự hành) |
| 2 | **Ráp khung TCP** (gộp bộ đệm) — đọc đúng `[len][payload]` từ burst 4 khung | Bản cũ chỉ đọc khung ĐẦU mỗi `recv()` → **bỏ lỡ ROBOT_STATE → `/odom` đứng yên** |
| 3 | Gửi `/cmd_vel` đúng **13 byte** (`velocity_msg_t`), không phải `sizeof(union)` ~97B | Tránh rác/lệch khung sang firmware |
| 4 | `recv()` EINTR/lỗi thoáng qua → bỏ qua, không giết luồng | Giữ `/odom` sống |
| 5 | `is_running` **atomic** + destructor `shutdown(listen_sock)` | **Ctrl-C thoát sạch** (bản cũ kẹt ở `accept()`) |
| 6 | odom điền `twist` + mutex thread-safe; thêm `STOP_ROBOT=4` vào `message_type.h` | NAV2 dùng twist; an toàn đa luồng |

> 💡 Nếu bạn cập nhật firmware/`wifi_node`, đồng bộ lại từ `Source_code/` (mình giữ nguyên bản đó).
> Đổi watchdog: `--ros-args -p cmd_timeout:=<giây>`.

---

## 3. WORKFLOW DEMO — hiểu đúng để ghép đúng

### 3.1 Ràng buộc gốc (PHẢI nắm): Astra USB2 — 1 mode/lúc
Camera Astra **không stream color (YOLO) + depth (/scan cho NAV2) đồng thời**. Vì vậy **`camera_manager`**
giữ 1 camera, đổi mode theo topic `/camera_mode` (`depth`|`color`|`both`). Mọi pha demo xoay quanh điều này.

### 3.2 Hai luồng demo
**(A) Quét map + tự hành (SLAM/NAV2)** — camera ở **depth**:
```
camera_manager(depth) → /scan → slam_toolbox → /map
   explorer (WFD) → frontier → NAV2 NavigateToPose → /cmd_vel → wifi_node → ĐẾ
   ĐẾ: EKF → ROBOT_STATE → wifi_node → /odom (nuôi lại SLAM + NAV2)
```
**(B) Detect → đi tới vật (T10 orchestrator, Hybrid)** — orchestrator điều phối camera:
```
DETECT   (camera color) : /yolo/detections → có vật mục tiêu? (true/false)
LOCATE   (camera both)  : depth ở tâm bbox + intrinsics → 3D → tf → POSE vật trên map
NAVIGATE (camera depth) : NavigateToPose(standoff goal) → NAV2 tự lập đường + né → tới gần
APPROACH (camera both)  : servo căn giữa bbox (vθ) + tiến tới ~0.3m (vx) → /cmd_vel
ARRIVED                 : dừng (gắp = pha sau)
```
> Orchestrator tự phát `/camera_mode` cho từng pha → `camera_manager` đổi camera. Robot DỪNG lúc detect
> (vì lúc đó camera color, không có /scan). Đây là hành vi đúng theo phần cứng.

### 3.3 Topic chính
| Topic | Kiểu | Ý nghĩa |
|-------|------|---------|
| `/camera_mode` | std_msgs/String | orchestrator → camera_manager (depth/color/both) |
| `/camera/depth/image_raw` (+`/camera_info`) | Image 16UC1 | depth (depth/both mode) → /scan, LOCATE |
| `/camera/color/image_raw` | Image bgr8 | color (color/both mode) → YOLO |
| `/scan` | LaserScan | từ depth (depthimage_to_laserscan) → SLAM/NAV2 |
| `/yolo/detections` | std_msgs/String (JSON) | `[{class_name, confidence, bbox}]` |
| **`/yolo/image_annotated/compressed`** | CompressedImage | **xem trên Foxglove** (ảnh + box, JPEG ~47KB) |
| `/odom` + tf `odom→base_link` | Odometry | từ ĐẾ (wifi_node) |
| `/orchestrator/state` | std_msgs/String | DETECT/LOCATE/NAVIGATE/APPROACH/ARRIVED |
| `/cmd_vel` | Twist | lệnh vận tốc → wifi_node → đế |

---

## 4. BUILD
```bash
cd ~/Robot_collecting_VLM_Model/ros2_ws
source /opt/ros/foxy/setup.bash
colcon build --symlink-install
source install/setup.bash          # mỗi terminal mới phải source 2 dòng (cân nhắc thêm vào ~/.bashrc)
```
**Kỳ vọng:** `Summary: 7 packages finished`, không error. (Phụ thuộc đã có sẵn trên Jetson: `libavahi-client-dev`,
`torch`+`ultralytics` cho YOLO, `nav2`, `slam_toolbox`, `depthimage_to_laserscan`, `rosbridge_suite`.)

---

## 5. THANG BẬC TEST (làm theo thứ tự — dừng nếu bước nào fail)

### ① Không cần gì — unit test (byte-format + toán định vị)
```bash
python3 src/base_bridge/test/test_protocol.py            # → 13/13 PASS (giao thức đế)
python3 src/vlm_nav_orchestrator/test/test_geometry.py   # → 11/11 PASS (pixel→3D, goal, servo)
```

### ② Cầu nối đế — KHÔNG cần phần cứng (dùng ESP32 giả)  ⭐
```bash
# T.A: ros2 run cpp_package wifi_node
# T.B: python3 src/cpp_package/test/fake_esp32.py          # giả đế: gửi telemetry, in lệnh nhận
# T.C: ros2 topic pub -r 5 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: -0.1}, angular: {z: 0.3}}"
# T.D: ros2 topic echo /odom --field pose.pose.position
```
**Kỳ vọng:** A log `New connection accepted`; B in `RX VELOCITY vx=0.200...`; D `x` tăng dần;
ngừng C → ~1.5s sau B in `RX STOP_ROBOT` (watchdog); Ctrl-C A → thoát trong ~1s.

### ③ Camera hot-switch — cần camera (KHÔNG cần đế)
```bash
ros2 run astra_ros camera_manager                                        # mở mode depth
ros2 topic pub --once /camera_mode std_msgs/msg/String "{data: color}"   # switch ~1-2s
ros2 topic list | grep camera     # color: /camera/color/image_raw ; depth: /camera/depth/*
```
**Kỳ vọng:** đổi mode → topic tương ứng xuất hiện (depth↔color↔both). *(Đã test PASS với camera thật.)*

### ④ YOLO detect — cần camera (KHÔNG cần đế)
```bash
ros2 launch robot_bringup yolo.launch.py                 # mặc định target_only:=true (chỉ vật thu gom)
ros2 launch robot_bringup yolo.launch.py target_only:=false   # nhận MỌI lớp COCO (demo sinh động)
```
Foxglove (`ws://<ip>:9090`, kiểu **Rosbridge**) → Image panel nguồn ảnh = **`/yolo/image_annotated/compressed`**
→ thấy khung + nhãn. *(Warmup CUDA ~10-20s lần đầu. Reconnect Foxglove nếu topic chưa vào dropdown.)*

### ⑤ Bench đế THẬT (sau khi dựng mạng — §6)
```bash
ros2 run cpp_package wifi_node           # đợi ESP32 connect → "New connection accepted"
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"   # đế tiến nhẹ
ros2 topic echo /odom                    # pose đổi theo chuyển động thật
```
> **An toàn:** kê bánh khỏi sàn lần đầu; watchdog sẽ STOP sau 1.5s khi ngừng /cmd_vel.

### ⑥ SLAM lái tay (đế thật) → quét + lưu map
```bash
ros2 launch robot_bringup bringup.launch.py nav:=false autonomous:=false
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# lái CHẬM, bám tường trong 1m → ros2 run nav2_map_server map_saver_cli -f ~/maps/room
```

### ⑦ DEMO đầy đủ: detect → đi tới vật (đế thật)
```bash
ros2 launch robot_bringup collect.launch.py target_class:=bottle
```
Foxglove xem `/yolo/image_annotated/compressed`, `/orchestrator/state`, `/map`, TF.
Robot: DETECT → LOCATE → NAVIGATE (tự lập đường tới chai) → APPROACH (căn giữa) → ARRIVED.

---

## 6. CẤU HÌNH cho phần cứng thật (trước bước ⑤–⑦)
1. **Mạng WiFi** (đã chốt phương án AP): cắm USB WiFi vào Jetson → `sudo bash tools/setup_robot_ap.sh`
   → Jetson thành AP `192.168.137.1`. ESP32 (`WIFI_SET`) join AP này → tự `connect()` `192.168.137.1:2004`.
   *(Firmware đã cứng IP này — KHÔNG cần sửa.)*
2. **Đo kích thước đế thật** → sửa `robot_description/urdf/robot.urdf.xacro` (vị trí camera) +
   `robot_bringup/config/nav2_params.yaml` (`robot_radius`, hiện giả định 0.22) → build lại.
3. Laptop join WiFi `MecanumRobot` → Foxglove `ws://192.168.137.1:9090`.

---

## 7. RANH GIỚI bàn giao
| ✅ Đã verify (KHÔNG cần đế) | 🔲 Cần đế thật |
|------------------------------|----------------|
| Build 7 pkg · unit-test 13/13 + 11/11 | Bench cmd_vel→chuyển động (⑤) |
| wifi_node: velocity 13B, watchdog STOP, /odom từ telemetry, Ctrl-C sạch (loopback) | Quét map thật + lưu (⑥) |
| **camera_manager hot-switch (LIVE camera thật)** | NAV2 đi+né thật; servo APPROACH |
| YOLO detect (LIVE) · orchestrator wiring (detect→locate→navigate, sim-free) | collect.launch chạy end-to-end (⑦) |
| Mọi launch validate (bringup/slam/yolo/collect) | Tune standoff/servo/target_z; đo robot_radius/hand-eye |

---

## 8. LƯU Ý / điểm cần kiểm chứng với đế
- **Astra chỉ "thấy" ~1m + góc 58°** → tự hành dễ lao vào vùng chưa cảm nhận. Chạy CHẬM, khu vực nhỏ;
  **lái tay (⑥) validate SLAM trước**, rồi mới tự hành/collect.
- **`explorer_node`** chạy `spinScan` ngay từ constructor → robot có thể tự xoay trước khi SLAM sẵn sàng.
  Cân nhắc `spin_timer_->cancel()` trong constructor (code thuật toán của bạn — để bạn quyết).
- **Orchestrator + thiếu NAV2 server** (test không đế) → cycle nhanh làm camera switch thrash; trên đế thật
  NAVIGATE giữ lâu nên OK. Đừng lo nếu thấy switch liên tục lúc test bộ phận.
- `robot_radius` + hand-eye TF đang **giả định** → phải đo thật.
- USB2: KHÔNG chạy 2 node mở camera cùng lúc (`camera_manager` thay cho `astra_node`+`astra_color_node`).
- `wifi_node` bind `:2004` `INADDR_ANY` không xác thực client — OK cho mạng AP riêng, đừng phơi mạng công cộng.
- Bẫy Foxglove: chọn kiểu **Rosbridge** (không "ROS 2"); xem ảnh dùng topic **`.../compressed`** (rosbridge cũ
  không tải nổi ảnh raw 900KB); topic mới xuất hiện sau khi connect cần **reconnect** để vào dropdown.

---

## 9. SƠ ĐỒ KIẾN TRÚC (tóm tắt)
```
                          ┌──────────────────── Jetson (ROS2 Foxy) ────────────────────┐
ĐẾ ESP32  ──WiFi:2004──►  │  wifi_node  ──/odom+tf──►  slam_toolbox ──/map──► NAV2      │
(EKF, motor)  ◄─/cmd_vel─ │     ▲                          ▲                    │       │
                          │     └──────── /cmd_vel ────────┴──── orchestrator ◄─┘       │
Astra USB2 ──► camera_manager ──/depth──► depthimage_to_laserscan ──/scan──► (SLAM/NAV2)│
   (1 mode)        │      └────/color──► yolo_node ──/detections + /...compressed──► orchestrator
                   └◄── /camera_mode (depth|color|both) ──── orchestrator                │
                          └──────────── rosbridge :9090 ──► Foxglove (laptop) ───────────┘
```

**Liên hệ:** mọi thay đổi của mình trong `ros2_ws/` (xem `git diff`). Bản gốc của bạn ở `Source_code/` nguyên vẹn.
