# DEMO RUNBOOK — SLAM + NAV2 tự hành với đế mecanum (WiFi)

Hướng dẫn vận hành khi ghép đế. Bám sát kế hoạch [PLAN_SLAM_NAV2.md §6](PLAN_SLAM_NAV2.md).
**Mục tiêu:** sau chuẩn bị, mỗi lần demo chỉ còn **config + run**.

> Kiến trúc: ESP32 (WiFi STA) → `connect() 192.168.137.1:2004` → Jetson chạy `wifi_node` (TCP server).
> Internet/SSH Jetson qua Ethernet; WiFi (USB dongle) làm AP riêng cho đế. Tất cả tính toán local.

---

## A. CHUẨN BỊ 1 LẦN (đã làm sẵn trong code, ✓)
- [x] Workspace `ros2_ws/` **7 package** build sạch. Gộp `cpp_package` (wifi_node + explorer_node).
- [x] `wifi_node` đã harden: watchdog STOP (cmd_timeout **1.5s**), ráp khung TCP, gửi 13B, odom twist, Ctrl-C sạch (loopback test PASS).
- [x] `camera_manager` (hot-switch USB2 depth/color/both theo `/camera_mode`) — test LIVE camera thật PASS.
- [x] `yolo_ros` (YOLOv8n → `/yolo/image_annotated/compressed` + `/yolo/detections`) — detect LIVE PASS.
- [x] `vlm_nav_orchestrator` (T10: detect → đi tới vật, Hybrid) — geometry 11/11 + wiring sim-free PASS.
- [x] Config Astra: `robot_bringup/config/{slam_toolbox,nav2_params}.yaml` (use_sim_time:False, max_range 1m, RPP).
- [x] Launch: `slam.launch.py` (lái tay) · `bringup.launch.py` (SLAM+NAV2+tự hành) · `yolo.launch.py` · **`collect.launch.py`** (detect→đi tới) · `tools/setup_robot_ap.sh`.

## B. KHI GHÉP ĐẾ — CONFIG (làm 1 lần mỗi setup)

### B1. Mạng (W0) — cắm USB WiFi vào Jetson
```bash
sudo bash tools/setup_robot_ap.sh                 # tự dò card; hoặc truyền: <iface> <ssid> <pass>
# Nếu báo "KHÔNG hỗ trợ AP mode" → đổi dongle khác.
```
→ Jetson thành AP `192.168.137.1`, SSID `MecanumRobot` / pass `robot12345`.

### B2. Đế ESP32
- `WIFI_SET` cho ESP32: ssid=`MecanumRobot`, pass=`robot12345` (qua app/serial của partner).
- Bật đế → nó tự `connect()` tới `192.168.137.1:2004`.

### B3. Đo & cập nhật kích thước thật (Open Q3) — QUAN TRỌNG cho costmap/TF
- Đo `base_link → camera_depth_optical_frame` (x,y,z) thật → sửa `robot_description/urdf/robot.urdf.xacro` (hiện giả định [0.15,0,0.20]).
- Đo bán kính/footprint đế → `nav2_params.yaml` (`robot_radius`, hiện 0.22).
- `colcon build --packages-select robot_description robot_bringup`.

### B4. Laptop xem demo
- Laptop join WiFi `MecanumRobot` → Foxglove: **Rosbridge** `ws://192.168.137.1:9090`.

## C. RUN

> Mỗi terminal: `source /opt/ros/foxy/setup.bash && source ~/Robot_collecting_VLM_Model/ros2_ws/install/setup.bash`

### C1. (W5) Quét map LÁI TAY trước — an toàn, validate SLAM
```bash
ros2 launch robot_bringup bringup.launch.py nav:=false autonomous:=false   # đế thật (odom:=wifi mặc định)
ros2 run teleop_twist_keyboard teleop_twist_keyboard                        # terminal khác
```
- Lái CHẬM, **bám tường/vật trong 1m**, xoay ở góc, quay về điểm đầu (loop-closure).
- Foxglove: thấy `/map` lớn dần, `/scan`, TF.
- Lưu map: `ros2 run nav2_map_server map_saver_cli -f ~/maps/room`

### C2. (W6) Demo TỰ HÀNH đầy đủ (WFD explorer + NAV2)
```bash
ros2 launch robot_bringup bringup.launch.py        # = odom:=wifi nav:=true autonomous:=true
```
- explorer spin 360° → detect frontier → NAV2 đi tới → lặp. Foxglove xem map mở rộng.
- Tắt tự hành, chỉ nav tay (gửi goal qua Foxglove): `autonomous:=false`.

### C3. Test KHÔNG cần đế (kiểm tra stack)
```bash
ros2 launch robot_bringup bringup.launch.py odom:=fake nav:=false autonomous:=false
```

### C4. YOLO detect (chỉ cần camera, KHÔNG cần đế)
```bash
ros2 launch robot_bringup yolo.launch.py                  # target_only:=true (vật thu gom)
ros2 launch robot_bringup yolo.launch.py target_only:=false   # mọi lớp COCO
```
- Foxglove → Image panel nguồn ảnh = **`/yolo/image_annotated/compressed`** (KHÔNG phải raw — rosbridge cũ không tải nổi 900KB).
- Warmup CUDA ~10–20s lần đầu; reconnect Foxglove nếu topic chưa vào dropdown.
- ⚠️ Mở camera mode color → KHÔNG chạy cùng SLAM/bringup (USB2 đơn).

### C5. (T10) DEMO ĐẦY ĐỦ: detect → đi tới vật (đế thật)
```bash
ros2 launch robot_bringup collect.launch.py target_class:=bottle
```
- camera_manager + yolo + slam + nav2 + orchestrator. Robot: **DETECT → LOCATE → NAVIGATE (tự lập đường) → APPROACH (servo căn giữa) → ARRIVED**.
- orchestrator tự phát `/camera_mode` → camera_manager đổi camera từng pha (robot dừng lúc detect — đúng vì USB2).
- Foxglove xem: `/yolo/image_annotated/compressed`, `/orchestrator/state`, `/map`, `/scan`, TF.
- `target_class:=""` (rỗng) → bắt vật có confidence cao nhất.

## D. TẮT AN TOÀN
- `Ctrl+C` ở terminal launch (foreground → SIGINT lan đúng, watchdog gửi STOP, node camera đóng camera).
- ⚠️ KHÔNG `kill -9` node giữ camera (`camera_manager`/`astra_node`/`astra_color_node`) — sẽ phải mở lại camera (busy `rc=4118`); nếu lỡ kill -9, mở lại node là camera nhận lại được.

## E. TROUBLESHOOTING
| Triệu chứng | Nguyên nhân / xử lý |
|-------------|---------------------|
| `wifi_node` không thấy "New connection accepted" | ESP32 chưa join AP / sai ssid-pass; AP chưa lên (`ip addr show <iface>` phải có 192.168.137.1); firewall |
| `/odom` đứng yên (=0) | ESP32 chưa gửi ROBOT_STATE / chưa connect; kiểm tra `ros2 topic echo /odom` |
| Xe chạy hoang khi mất tín hiệu | watchdog `cmd_timeout` (mặc định **1.5s**) phải gửi STOP — kiểm log "gửi STOP_ROBOT". Đổi: `-p cmd_timeout:=<giây>` |
| Foxglove không nối | Sai loại (chọn **Rosbridge** không "ROS 2"); URL `ws://192.168.137.1:9090` không localhost |
| Không thấy ảnh YOLO trên Foxglove | Dùng topic **`/yolo/image_annotated/compressed`** (raw 900KB rosbridge không tải nổi); reconnect Foxglove để topic vào dropdown |
| `/camera/color` hoặc `/scan` không ra | camera_manager đang ở mode khác — chỉ 1 mode/lúc (USB2). Kiểm `/camera_mode`. Đừng chạy 2 node mở camera |
| Map trôi / NAV2 đâm vật | Astra cap 1m (§6.3a): chạy chậm, khu vực nhỏ, bám cấu trúc; cân nhắc LiDAR 2D nâng cấp |
| NAV2 lifecycle không active | `ros2 lifecycle get /controller_server`; xem log lifecycle_manager; cần `/map`+TF đủ |
| collect.launch: camera switch liên tục | Khi thiếu NAV2 (test bộ phận) orchestrator cycle nhanh → thrash; trên đế thật NAVIGATE giữ lâu nên OK |
| `ros2 topic hz` không hiện (msg lớn) | Bẫy Foxy — dùng subscriber rclpy đếm thay vì hz |

## F. CHƯA LÀM / GIỚI HẠN
- Hand-eye TF + robot_radius đang GIẢ ĐỊNH → phải đo (B3) trước khi tin costmap/nav.
- Astra 1m: tự hành rủi ro đâm vùng chưa cảm nhận → lái tay (C1) trước, tự hành (C2) thận trọng.
- **T10 (detect→đi tới)**: phần mềm XONG (`collect.launch.py`, đã verify bộ phận) — CÒN chạy live trên đế + tune standoff/servo/target_z. Định vị vật chính xác cần vật trong ~1m (giới hạn depth Astra).
- **VLM grounding** (chọn vật theo tên tiếng Việt) chưa nối — hiện dùng YOLO detect theo lớp (vd bottle). VLM là lớp thêm sau (giữ HTTP, gọi từ orchestrator).
- `explorer_node` spinScan chạy ngay khi khởi động → robot có thể tự xoay trước khi SLAM sẵn sàng (xem HANDOFF §8).
