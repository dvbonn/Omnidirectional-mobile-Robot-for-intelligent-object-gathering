# ros2_ws — Stack robot tự hành (ROS2 Foxy)

Workspace ROS2 điều khiển robot mecanum thật: **SLAM + NAV2 + tự khám phá (WFD) + detect (YOLO) → đi tới vật**. Chạy trên **Jetson AGX Xavier**, ROS2 **Foxy**.

Vận hành đầy đủ: [../docs/DEMO_RUNBOOK.md](../docs/DEMO_RUNBOOK.md) · Bàn giao & lý do thiết kế: [../docs/HANDOFF.md](../docs/HANDOFF.md) · Kế hoạch: [../docs/PLAN_SLAM_NAV2.md](../docs/PLAN_SLAM_NAV2.md).

---

## Kiến trúc

```
                 ┌───────────── Jetson (ROS2 Foxy) ─────────────┐
Astra depth ─▶ astra_node ─▶ /scan* ─▶ slam_toolbox ─▶ /map ─┐
                 │  (*qua depthimage_to_laserscan)            ├─▶ NAV2 ─▶ /cmd_vel ─┐
Astra color ─▶ yolo_node ─▶ /yolo/detections ─▶ orchestrator ┘  (planner/controller │
explorer (WFD) ─▶ frontier goal ─────────────────────────────────  /recoveries/bt)   │
                 └──────────────────────────────────────────────────────────────────┼─▶ wifi_node
                                                                                     │   (TCP :2004)
   ĐẾ ESP32 ◀── WiFi ──── wifi_node ◀── /cmd_vel                                      │
   ĐẾ: EKF ─▶ ROBOT_STATE ── WiFi ──▶ wifi_node ─▶ /odom ─▶ (nuôi lại SLAM + NAV2) ◀──┘
```

> ⚠️ **Astra USB 2.0 — 1 mode/lúc**: camera không stream color (YOLO) + depth (/scan) đồng thời. `camera_manager` giữ 1 camera, đổi mode theo topic `/camera_mode` (`depth`|`color`|`both`). Orchestrator điều phối: DETECT(color) → LOCATE(both) → NAVIGATE(depth) → APPROACH(both).

---

## 7 package

| Package | Executable | Vai trò |
|---|---|---|
| **cpp_package** | `wifi_node`, `explorer_node` | Cầu nối đế WiFi (TCP server `:2004`) đã harden + tự hành WFD (frontier) |
| **astra_ros** | `astra_node`, `astra_color_node`, `camera_manager` | Astra → ROS2. `camera_manager` = 1 node đổi mode theo `/camera_mode` |
| **yolo_ros** | `yolo_node` | YOLOv8n: ảnh màu → `/yolo/image_annotated/compressed` + `/yolo/detections` |
| **vlm_nav_orchestrator** | `orchestrator_node` | State machine: DETECT → LOCATE → NAVIGATE → APPROACH → ARRIVED |
| **robot_bringup** | (launch + config) | Launch tổng + `slam_toolbox.yaml` / `nav2_params.yaml` (tune cho Astra) |
| **robot_description** | (URDF/TF) | `robot.urdf.xacro` + static TF |
| **base_bridge** | `fake_odom` | Test không cần đế; `protocol.py` = spec byte-format tham chiếu |

`astra_node`/`yolo_node` **dùng lại** driver Astra + YOLO của [`../layer1_vision/`](../layer1_vision/).

---

## Setup

```bash
# 1. Package hệ thống (ROS2 Foxy)
sudo apt install ros-foxy-slam-toolbox ros-foxy-navigation2 ros-foxy-nav2-bringup \
                 ros-foxy-depthimage-to-laserscan ros-foxy-rosbridge-server \
                 ros-foxy-teleop-twist-keyboard

# 2. Build
cd ros2_ws
colcon build --symlink-install
source install/setup.bash        # lặp lại ở MỖI terminal mới
```

**Driver Astra**: `astra_node` cần driver Orbbec OpenNI2 ở `../tools/orbbec/openni2/` (xem [../layer1_vision/README.md](../layer1_vision/README.md)) và intrinsics ở `../config/astra_intrinsics.json`.

**Mạng WiFi cho đế** (mỗi setup 1 lần): Jetson làm AP, ESP32 nối vào.
```bash
sudo bash ../tools/setup_hostapd.sh     # AP 192.168.137.1, SSID MecanumRobot
```

**Đo kích thước thật khi ghép đế** — sửa rồi build lại:
- `base_link → camera_depth_optical_frame` → `robot_description/urdf/robot.urdf.xacro`
- bán kính đế → `robot_bringup/config/nav2_params.yaml` (`robot_radius`)
```bash
colcon build --packages-select robot_description robot_bringup
```

---

## Chạy (launch)

> Thêm `odom:=fake` để test **không cần đế thật** (đứng yên nhưng vẫn có TF odom→base_link).

```bash
# Chỉ dựng TF + wiring (không cắm đế/camera) — kiểm tra graph & cây TF
ros2 launch robot_bringup bringup.launch.py odom:=fake sensor:=none nav:=false autonomous:=false

# Quét map LÁI TAY (validate SLAM) — cần Astra:
ros2 launch robot_bringup slam.launch.py odom:=fake
ros2 run teleop_twist_keyboard teleop_twist_keyboard      # terminal khác

# Tự hành đầy đủ (WFD explorer + NAV2, đế thật):
ros2 launch robot_bringup bringup.launch.py

# Demo THU GOM: detect (YOLO) → đi tới vật:
ros2 launch robot_bringup collect.launch.py target_class:=bottle
```

### Launch files & tham số chính

| Launch | Mục đích | Args |
|---|---|---|
| `slam.launch.py` | Chỉ SLAM (lái tay quét map) | `odom={wifi,fake,none}` |
| `bringup.launch.py` | Stack thật end-to-end: TF+đế+Astra+SLAM+NAV2+WFD | `odom`, `sensor={astra,none}`, `nav`, `autonomous`, `cmd_timeout` |
| `collect.launch.py` | Demo detect → đi tới vật (dùng `camera_manager`) | `target_class`, `odom`, `nav`, `autonomous`, `detect_scan_sec`, `detect_look_sec` |
| `yolo.launch.py` | Chỉ YOLO node | `target_only` |
| `astra_scan.launch.py` | Astra → /scan (kiểm tra depth→laserscan) | — |

Lưu map: `ros2 run nav2_map_server map_saver_cli -f ~/maps/room`

---

## Foxglove (xem demo từ laptop)

Rosbridge chạy sẵn ở `:9090`. Laptop join WiFi `MecanumRobot` → Foxglove Studio → Rosbridge `ws://192.168.137.1:9090`. Panel hữu ích: `/map`, `/scan`, TF, `/yolo/image_annotated/compressed`, `/orchestrator/state`.

> Có **hai** `ros2_ws`: bản này (đầy đủ 7 package) và `Source_code/ros2_ws/` (chỉ `cpp_package` — bản gốc tham chiếu của partner).
