# PLAN — SLAM (quét Map) + NAV2 (điều hướng tự hành) trong ROS2

**Created:** 2026-06-25
**ROS2 distro:** Foxy (ros-base, headless) trên Jetson AGX Xavier / L4T R35.1 / Ubuntu 20.04
**Estimated effort:** ~10–14 ngày làm việc (nhiều rủi ro phần cứng → buffer cao)
**Strategy:** **Sim-first, vertical slicing** — dựng & nghiệm thu toàn bộ stack trên mô phỏng trước (không phụ thuộc đế thật), rồi mới hoán đổi sang phần cứng. Mỗi task deliver được 1 năng lực test ngay.

> Tham chiếu memory: [[project_ros2_architecture]] (ROS2 ở ranh giới Layer2↔Layer3, tf2 cam→robot), [[project-mecanum-base-protocol]] (SET_ROBOT_VELOCITY / ROBOT_STATE EKF pose), [[project-jetson-astra]] (giới hạn USB 2.0, intrinsics), [[project_jetson_disk_status]] (disk 4.4 GB — nút thắt).

---

## 0. Quyết định đã chốt (từ câu hỏi với user 2026-06-25)

| # | Quyết định | Hệ quả thiết kế |
|---|-----------|-----------------|
| Cảm biến SLAM | **Chỉ dùng Astra depth** (không gắn LiDAR) | `depthimage_to_laserscan` → fake `/scan` → `slam_toolbox` 2D. **Cap ~1m + FOV 58.6° ĐÃ XÁC NHẬN (T0, 2026-06-25)**; user chọn vẫn dùng → chiến lược **khu vực nhỏ + bám tường + odom-heavy** (xem T0 KẾT QUẢ + T5). |
| Phạm vi nav | **Kết hợp VLM hiện có** | NAV2 đưa robot tới *khu vực* → bàn giao **visual servoing + VLM** ở pha tiếp cận cuối + (tương lai) gắp. KHÔNG cần full autonomy patrol. |
| Đế mecanum | **Chưa kết nối / chưa chắc** | **Dựng + nghiệm thu trên Gazebo trước**, `base_bridge` thật làm sau khi đế sẵn sàng. |

---

## 1. Kiến trúc & quyết định kỹ thuật

### 1.1 Thuật toán — vì sao slam_toolbox + NAV2 (không phải cái khác)
- **SLAM 2D:** `slam_toolbox` (sync/async) là chuẩn de-facto cho ROS2, hỗ trợ Foxy tốt, có chế độ **localization** (thay AMCL), serialize map (`.posegraph`) + xuất `map.pgm/yaml`. Tốt hơn Cartographer trên Foxy (Cartographer Foxy ít được maintain).
- **Navigation:** **NAV2 là tiêu chuẩn ROS2** — không có framework thay thế "tốt hơn" cho điều hướng tự hành 2D. Xavier thừa sức chạy. (rtabmap = 3D RGBD, **bị loại** vì USB2 không xuất RGB+depth đồng thời; đã xác nhận trong [[project-jetson-astra]].)
- **Vì sao không full autonomy:** đúng tinh thần quyết định cũ "nav2 quá mức cho việc *tiến tới vật + dừng*". Lần này NAV2 chỉ lo **đi tới khu vực trên map**; pha *căn giữa + tiến sát vật* vẫn do visual servoing/VLM (chính xác hơn ở cự ly gần, không phụ thuộc chất lượng map).

### 1.2 Phân tách chuyển động (mecanum holonomic)
- **NAV2 điều khiển bằng vx + vθ** (như diff-drive) cho đơn giản & robust trên Foxy (Foxy **chưa có** MPPI; DWB holonomic finicky). Mecanum là superset nên chạy được ngay.
- **vy (đi ngang)** để dành cho **visual servoing** ở pha cuối (căn `X` ngang của vật về 0) — nơi holonomic phát huy giá trị thật.

### 1.3 Cây TF (REP-105)
```
map ──(slam_toolbox)──> odom ──(base_bridge / Gazebo)──> base_link ──(static)──> camera_link ──(static)──> camera_depth_optical_frame
```
- `map→odom`: slam_toolbox (mapping) hoặc slam_toolbox-localization/AMCL (chạy lại trên map đã lưu).
- `odom→base_link`: **base_bridge** từ `ROBOT_STATE` (pose EKF đã fuse IMU+optical flow của đế). Trong sim: plugin Gazebo.
- `base_link→camera_*`: `static_transform_publisher` (đo tay/đọc từ thiết kế cơ khí) — đây là tf2 cam→robot đã nói trong [[project_ros2_architecture]]. Repo đã quy ước frame `camera_depth_optical_frame` ([detection_log.py](../layer1_vision/detection_log.py)).

### 1.4 Sơ đồ pipeline runtime (mục tiêu cuối)
```
                 (mapping 1 lần)                         (vận hành)
Astra(depth) → /depth/image_raw + /camera_info → depthimage_to_laserscan → /scan
                                                                              │
                                                  ┌───────────────────────────┴──────────┐
                                                  ▼                                        ▼
                                           slam_toolbox                              NAV2 costmaps
                                           (map.pgm/yaml + map→odom)                  (global+local)
                                                                                          │
   base_bridge: ROBOT_STATE → /odom + tf(odom→base_link)                                  ▼
   base_bridge: /cmd_vel → SET_ROBOT_VELOCITY (TCP/CAN) ◄──── NAV2 controller (vx,vθ) ─────┘
                                          ▲
                                          └──── visual servoing (vx,vy,vθ) [pha cuối]
                                                       ▲
   vlm_nav_orchestrator:  goal(tên vật) → NAV2 NavigateToPose(tới khu vực)
                          → switch Astra sang color → Brain/VLM detect → servo → handoff gắp
```

### 1.5 Bố cục code (workspace mới, KHÔNG đụng 3-layer HTTP hiện có)
```
ros2_ws/                         # colcon workspace mới ở gốc repo
  src/
    robot_description/           # URDF/xacro mecanum + camera, static TF
    robot_bringup/               # launch files, params (slam/nav2), maps/, rviz config
    astra_ros/                   # node rclpy: AstraCamera(mode=depth) → Image + CameraInfo
    base_bridge/                 # node rclpy: /cmd_vel↔SET_ROBOT_VELOCITY, ROBOT_STATE→/odom+tf
    vlm_nav_orchestrator/        # node: NAV2 action client + handoff sang VLM/visual servoing
  # depthimage_to_laserscan, slam_toolbox, navigation2 = dùng binary apt, không build lại
```
> 3-layer HTTP (Vision/Brain/VLM) **giữ nguyên**. `vlm_nav_orchestrator` gọi Brain qua HTTP `:8000` như cũ; chỉ thêm lớp ROS2 cho di chuyển. Bỏ `control_node.py`/`kinematics.py` (placeholder gắp cánh tay — sai, xem [[project-mecanum-base-protocol]]) khỏi luồng đế.

### 1.6 Cách xem demo (visualization) — CHỐT: Foxglove Studio
**Bối cảnh thật (xác nhận 2026-06-25):** máy `asiclab-desktop` (IP 192.168.1.250) **CHÍNH LÀ Jetson** — KHÔNG có desktop riêng. Jetson có Xorg+GDM chạy (có màn hình), nhưng disk chỉ 4.4 GB nên tránh cài cả RViz2 + Gazebo GUI.

**Quyết định:** xem demo từ **laptop/điện thoại qua Foxglove Studio** (user chọn).
- **Trên Jetson:** chạy `rosbridge_server` (`ros-foxy-rosbridge-suite`, có sẵn apt, nhẹ, không build) → WebSocket `ws://192.168.1.250:9090`. (Nâng cấp sau nếu nghẽn băng thông: build `foxglove_bridge` từ nguồn — không có binary Foxy.)
- **Trên laptop/điện thoại:** mở **Foxglove Studio** (app web/desktop, mọi OS) → "Open connection → Rosbridge → `ws://192.168.1.250:9090`" (cùng WiFi LAN).
- **Sim chạy headless:** dùng **`gzserver`** (KHÔNG mở `gzclient`/GUI) → tiết kiệm GPU/không cần X; mọi hình ảnh (map, /scan, TF, robot, costmap, path) xem qua Foxglove.
- **Ra lệnh nav:** dùng panel của Foxglove publish `/goal_pose` (thay nút "2D Goal Pose" của RViz). Foxglove hiển thị: 3D panel (TF+robot+map+scan+path), Image panel (depth/RGB), State Transitions.
- **Backup xem lại:** `rosbag2 record` toàn demo → phát lại + xem trên Foxglove bất cứ lúc nào (tốt cho video báo cáo).

---

## 2. Dependency graph

```
T0 (spike tầm depth — BLOCKING, fail-fast)
T1 (dọn disk + cài gói Foxy)
        │
        ▼
T2 (workspace + robot_description + static TF)
        │
        ├────────────► T3 (Gazebo sim: base + depth cam + odom)
        │                      │
        ▼                      ▼
        └──────────────► T4 (depthimage_to_laserscan → /scan)
                               │
                               ▼
                        T5 (slam_toolbox → map.pgm/yaml)   ★ Checkpoint 2
                               │
                               ▼
                        T6 (NAV2 params: costmap+planner+controller+localization)
                               │
                               ▼
                        T7 (nav2_bringup: NavigateToPose trong sim)  ★ Checkpoint 3
                               │
          ┌────────────────────┼─────────────────────┐
          ▼                                            ▼
   T8 (base_bridge thật)                        T9 (astra_ros thật trên Jetson)
          └────────────────────┬─────────────────────┘
                               ▼
                        SLAM + Nav trên phần cứng thật  ★ Checkpoint 4
                               │
                               ▼
                        T10 (vlm_nav_orchestrator: NAV2 → VLM handoff)
                               ▼
                        T11 (end-to-end + docs)          ★ Checkpoint 5 Go/No-go
```

---

## Phase 0 — Spike & chuẩn bị (fail-fast)

### T0 — Spike: đo tầm depth thực của Astra + đánh giá khả thi SLAM
**Scope:** S · **Deps:** none · **BLOCKING** (cao nhất)

**Mục tiêu:** Memory ghi tầm depth quan sát được chỉ **~22–1022 mm (~1 m)**. Nếu đúng vậy, FOV ~58° + tầm ~1 m gần như **không đủ để quét map một căn phòng** — phải báo lại user trước khi đổ công vào stack.

**Steps:**
1. `AstraCamera(mode="depth")`, đặt vật ở các mốc 0.5/1/2/3/4 m, log `min/max/median` mm hợp lệ (loại 0=invalid).
2. Đo FOV ngang hữu dụng (số cột có depth hợp lệ ở tường phẳng cách ~1.5 m).
3. Ghi histogram độ nhiễu theo khoảng cách.

**Acceptance criteria:**
- [x] `tools/astra_depth_range_report.py` xuất bảng: tầm max tin cậy (mm), FOV ngang (°), %pixel hợp lệ theo cự ly.
- [x] Kết luận rõ: **(B)** tầm ~1m — user quyết định tiếp tục với chiến lược tầm ngắn.

**Verification:** `python tools/astra_depth_range_report.py` → in bảng + lưu `docs/astra_depth_range.md`.

> ⚠️ **Đây là cổng go/no-go quan trọng nhất.** Đừng làm T2+ trước khi T0 xác nhận tầm đo đủ dùng.

**✅ KẾT QUẢ T0 (2026-06-25):** `tools/astra_depth_range_report.py` (probe) → camera khoẻ 29 FPS, 100% valid, **FOV ngang 58.6°** (xác nhận hẹp). **Tầm tối đa cap ~1022mm, 0 pixel >1.1m** → verdict **[B] cap ~1m XÁC NHẬN**. Báo cáo: `docs/astra_depth_range.md`.
> **Quyết định user:** **VẪN DÙNG depth-only SLAM** (chấp nhận giới hạn) vì xe có thể quay+di chuyển và use-case (NAV2→khu vực→VLM servoing pha cuối) dung thứ sai số map. → **Tiếp tục T1+** theo **chiến lược tầm ngắn** (T5). LiDAR 2D giữ làm đường nâng cấp.
> ⚠️ Lưu ý vật lý: quay/di chuyển khắc phục FOV hẹp nhưng **KHÔNG** khắc phục tầm 1m → xe phải **luôn bám cấu trúc trong 1m**; map tốt ở khu vực nhỏ nhiều đặc trưng, trôi ở khoảng trống lớn.

---

### T1 — Dọn disk + cài gói Foxy
**Scope:** S · **Deps:** none

**Acceptance criteria:**
- [x] `sudo apt clean && pip cache purge` → xác nhận ≥6 GB trống (`df -h /`). *(thực tế 5.0 GB — đủ)*
- [x] Cài: `ros-foxy-slam-toolbox ros-foxy-navigation2 ros-foxy-nav2-bringup ros-foxy-depthimage-to-laserscan ros-foxy-nav2-regulated-pure-pursuit-controller ros-foxy-tf2-ros` (đã xác nhận có trong apt).
- [x] Cài `ros-foxy-rosbridge-suite` (cho Foxglove — xem §1.6). **KHÔNG cài rviz2** trên Jetson ✓
- [ ] ~~Cài `ros-foxy-gazebo-ros-pkgs`~~ → **HOÃN** (disk chật; T3 bỏ qua, T4 chạy với camera thật).
- [x] Disk còn ≥3 GB sau khi cài: **5.0 GB trống** ✓

**Verification:** `source /opt/ros/foxy/setup.bash && ros2 pkg list | grep -E "slam_toolbox|nav2|depthimage"` ra đủ gói.

**✅ KẾT QUẢ T1 (2026-06-25):** `apt clean` giải phóng ~1.8 GB + xoá 4 bản vscode-server cũ ~2.0 GB → **tổng ~5.0 GB trống** (81% disk). Đã cài xong: `ros-foxy-slam-toolbox ros-foxy-navigation2 ros-foxy-nav2-bringup ros-foxy-depthimage-to-laserscan ros-foxy-rosbridge-suite ros-foxy-nav2-regulated-pure-pursuit-controller ros-foxy-tf2-ros`. ⚠️ **`ros-foxy-gazebo-ros-pkgs` CHƯA CÀI** (~1.5 GB — disk còn chật) → **T3 hoãn**; không chặn T4+ vì T4 đã validate với camera thật. Foxglove: rosbridge `:9090` OK; bẫy kết nối: chọn **"Rosbridge (ROS 1 & ROS 2)"** (không "ROS 2") + `ws://192.168.1.250:9090` (không `localhost`).

### ★ Checkpoint 1 (sau T0–T1)
- [x] Tầm depth: **T0 = nhánh B** (cap ~1m) — user chọn tiếp tục chiến lược tầm ngắn (không đổi LiDAR).
- [x] Disk còn ≥3 GB: **5.0 GB trống**; gói core cài xong (Gazebo hoãn — không chặn T4+).

---

## Phase 1 — Nền tảng workspace + TF + mô phỏng

### T2 — colcon workspace + robot_description + static TF
**Scope:** M · **Deps:** T1

**Acceptance criteria:**
- [x] `ros2_ws/` build sạch (`colcon build --symlink-install`), `source install/setup.bash` OK.
- [x] `robot_description`: URDF/xacro mecanum (footprint, bánh) + link `camera_link`, `camera_depth_optical_frame`.
- [x] Static TF `base_link→camera_depth_optical_frame` trans [0.15,0,0.20] quat [-0.5,0.5,-0.5,0.5] ✓ *(kích thước GIẢ ĐỊNH — TODO đo tay trước T9)*
- [x] `robot_state_publisher` chạy, cây TF `base_link→…→camera_depth_optical_frame` không gãy ✓

**Files:** `ros2_ws/src/robot_description/{urdf,launch}/*`
**Verification:** `ros2 launch robot_description description.launch.py` + xem TF/model qua Foxglove.

**✅ XONG (2026-06-25):** workspace `ros2_ws/` build sạch (colcon `--symlink-install`); `robot_description` (URDF mecanum + camera, fixed joints) + `robot_bringup` (launch). `check_urdf` xác nhận cây 1 gốc `base_link`. Runtime TF `base_link→camera_depth_optical_frame` = trans [0.15,0,0.20] + quat optical [-0.5,0.5,-0.5,0.5] ✓. **Fix Foxy:** `robot_description` phải bọc `ParameterValue(value_type=str)` (nếu không launch_ros parse YAML → vỡ ở dấu ':'). **Đã thêm `robot_bringup/launch/view_foxglove.launch.py`** (robot_state_publisher + rosbridge :9090) — kiểm chứng rosbridge lắng nghe :9090, /robot_description + /tf sẵn sàng cho Foxglove. ⚠️ Kích thước đế GIẢ ĐỊNH (TODO đo tay trước T9).

---

### T3 — Gazebo sim: đế + depth camera + odom
**Scope:** L · **Deps:** T2

**Mục tiêu:** Có "đế ảo" để dựng/nghiệm thu toàn bộ stack mà KHÔNG cần phần cứng. Dùng diff-drive plugin (vx,vθ) + depth-camera sensor plugin mô phỏng đúng FOV/tầm Astra đo được ở T0 (để sim phản ánh hạn chế thật).

**Acceptance criteria:**
- [ ] World nhỏ (1 phòng ~5×5 m, vài vật cản) load được (`gzserver` headless OK).
- [ ] Plugin publish `/odom` + tf `odom→base_link`; điều khiển qua `/cmd_vel` (`teleop_twist_keyboard`).
- [ ] Depth camera plugin publish `/depth/image_raw` + `/camera_info` với FOV ngang & `range_max` set theo **kết quả T0**.
- [ ] RViz hiện depth + robot di chuyển mượt.

**Files:** `ros2_ws/src/robot_bringup/{worlds,launch,urdf_gazebo}/*`
**Verification:** `ros2 launch robot_bringup sim.launch.py`; lái robot trong Gazebo; `ros2 topic echo /odom`.

> 💡 Chạy `gzserver` **headless** trên Jetson (không `gzclient`). Xem robot di chuyển + depth qua **Foxglove Studio** trên laptop/điện thoại (§1.6), không cần X/GUI trên Jetson.

> ⏸️ **T3 HOÃN (2026-06-25):** Disk ~5.0 GB không đủ cho Gazebo (~1.5 GB). **T4 đã validate pipeline cảm biến thành công với camera thật** (không cần sim) → không chặn tiến độ. Path tiếp theo cho T5: dùng **đế thật (T8)** khi sẵn sàng, hoặc giải phóng thêm disk rồi cài Gazebo sau. Nếu cần sim gấp: rosbag turtlebot3 mẫu (xem Risks).

### ★ Checkpoint 1.5 (sau T3)
- [ ] Cây TF `map?`→`odom`→`base_link`→`camera_*` (chưa có map là OK). ← **bỏ qua, T3 hoãn**
- [ ] Lái được robot ảo, có `/odom`, có `/depth/image_raw`. ← **bỏ qua, T3 hoãn**

---

## Phase 2 — Sensor → LaserScan

### T4 — depthimage_to_laserscan → /scan
**Scope:** S · **Deps:** T3 *(thực tế: bypass T3, dùng camera thật)*

**Acceptance criteria:**
- [x] Node `depthimage_to_laserscan` nhận `/camera/depth/image_raw`+`/camera/depth/camera_info`, publish `/scan` (640 tia, 110 hữu hạn, 0.20–0.86m) ✓
- [x] Tune: `scan_height=10`, `range_max=0.90m`, `range_min=0.20m`, `output_frame=camera_depth_optical_frame` ✓
- [x] FOV xác nhận: [-29.3°…+29.3°] = **58.6°** (khớp T0), KHÔNG 360° ✓ *(xác nhận qua subscriber rclpy, không qua RViz — T3 hoãn)*

**Files:** `ros2_ws/src/robot_bringup/launch/astra_scan.launch.py` (params inline) + `ros2_ws/src/astra_ros/` (node).
**Verification:** subscriber rclpy đọc `/scan` (KHÔNG dùng `ros2 topic hz` — lỗi hiển thị msg lớn ở Foxy).

**✅ XONG (2026-06-25) — chạy với CAMERA THẬT (không cần Gazebo):** thêm package `astra_ros` (node publish depth Astra `mode=depth` → `/camera/depth/image_raw` 16UC1 mm + `/camera/depth/camera_info` từ intrinsics) — đây là **T9 làm sớm**. `astra_scan.launch.py` = astra_node + depthimage_to_laserscan + rosbridge. Kết quả thật: `/scan` 640 tia, FOV [-29.3..29.3]° (=58.6°, khớp T0), **110/640 tia hữu hạn, min 0.20m max 0.862m** → pipeline cảm biến CHẠY end-to-end. ⚠️ Publish ~7Hz (msg 614KB reliable chậm — TODO: cân nhắc SensorDataQoS/giảm tải). ⚠️ Bẫy Foxy đã gặp: `ros2 topic hz` không hiện với msg lớn, `--once` không tồn tại, `ros2 run X &` để mồ côi node con ôm camera → kill node thật bằng SIGINT (không -9).

---

## Phase 3 — SLAM (deliverable: một tấm map)

### T5 — slam_toolbox mapping + lưu map
**Scope:** M · **Deps:** T4 + `/odom` (cần đế thật T8 — T3 Gazebo đã hoãn)

> ⚠️ **T5 bị BLOCK bởi T3 hoãn:** slam_toolbox cần `/odom` + chuyển động thật. Path tiếp theo: **làm T8 (base_bridge) trước** để có `/odom` thật → quay lại T5. Hoặc giải phóng disk + cài Gazebo cho sim.

**Acceptance criteria:**
- [x] `slam_toolbox` (online_async) nhận `/scan`+`/odom`, publish tf `map→odom` + `/map`. ✓ *(smoke-test: ra OccupancyGrid + tf map→odom 50Hz với camera thật + fake_odom)*
- [x] Params chỉnh cho FOV hẹp/tầm ngắn (cap 1m): `max_laser_range=1.0`, `resolution=0.03`, `minimum_travel_distance=0.1` + `minimum_travel_heading=0.1`, `use_scan_matching=true`, **siết `correlation_search_space_dimension=0.3` (tin odom hơn)**. ✓ *(config viết xong)*
- [ ] **SOP mapping tầm ngắn:** teleop CHẬM, **bám tường/đồ vật trong 1m** (boustrophedon/men mép), xoay ở góc để bắt đặc trưng, **quay về điểm xuất phát để loop-closure**. KHÔNG tự hành lúc quét. Khu vực ≤~4×4m, nhiều đặc trưng. *(chờ đế thật để lái)*
- [ ] Lái robot khắp khu vực → map hội tụ, không "trôi" nặng. *(chờ đế thật)*
- [ ] Lưu được map: `ros2 run nav2_map_server map_saver_cli -f maps/room` → ra `room.pgm`+`.yaml`. *(chờ map thật; CLI map_saver cần map đủ lớn + đúng QoS transient_local)*

**Files:** `ros2_ws/src/robot_bringup/config/slam_toolbox.yaml` ✓, `launch/slam.launch.py` ✓ (arg `odom:=fake|base|none`), `maps/`.
**Verification:** `ros2 launch robot_bringup slam.launch.py odom:=fake` (chưa có đế) → Foxglove thấy /map+/scan+TF.

**✅ CONFIG XONG + SMOKE-TEST PASS (2026-06-25):** `slam_toolbox.yaml` viết xong, tune đúng tầm ngắn. **Smoke-test sim-free** (camera Astra THẬT + `fake_odom` đứng yên) → slam_toolbox load config OK, subscribe `/scan`, publish `/map` (OccupancyGrid 15×24 @0.03m — nón FOV vì camera tĩnh) + tf `map→odom` ✓. **Wiring SLAM xác nhận end-to-end.** CÒN LẠI: cần ĐẾ THẬT (`/odom` + chuyển động) để quét map đầy đủ + lưu — đây là phần duy nhất còn chờ phần cứng.

### ★ Checkpoint 2 (sau T5)
- [ ] Quét & lưu được map (trực tiếp với phần cứng thật — T3 Gazebo đã hoãn). **Đây là mốc "Ứng dụng SLAM để quét Map".**
- [ ] Review chất lượng map trước khi sang NAV2.

---

## Phase 4 — NAV2 (deliverable: đi tới điểm trên map)

### T6 — NAV2 params (costmap + planner + controller + localization)
**Scope:** L · **Deps:** T5

**Acceptance criteria:**
- [x] `nav2_params.yaml`: global+local costmap dùng `/scan` (obstacle + inflation layer); **`obstacle_range=0.9`/`raytrace_range=1.0` theo tầm Astra**; `robot_radius=0.22` (⚠️ giả định, đo đế thật). ✓
- [x] Planner = NavFn; Controller = **Regulated Pure Pursuit** (`desired_linear_vel=0.18`, `use_rotate_to_heading=true` bù FOV hẹp). ✓
- [x] Vận tốc bảo thủ; `use_sim_time: False` toàn bộ (phần cứng thật, không Gazebo clock). ✓
- [ ] Localization: chạy lại trên map đã lưu bằng `slam_toolbox` mode `localization` (ưu tiên) — fallback AMCL. *(cần map thật)*

**Files:** `ros2_ws/src/robot_bringup/config/nav2_params.yaml`.
**Verification:** `ros2 launch nav2_bringup ... ` lên, không lỗi lifecycle (`ros2 lifecycle get /controller_server` = active).

**✅ CONFIG XONG (2026-06-25):** `nav2_params.yaml` viết xong (base mẫu Foxy), tune cho mecanum+Astra: RPP controller, costmap range≈1m, vận tốc bảo thủ, `use_sim_time:False`. YAML parse OK + plugin name RPP đúng. CÒN LẠI: nghiệm thu lifecycle `active` cần **map thật** (từ T5) + đế → làm cùng T7 khi có phần cứng.

---

### T7 — nav2_bringup tích hợp: NavigateToPose trong sim
**Scope:** M · **Deps:** T6

**Acceptance criteria:**
- [ ] Launch tổng `bringup_sim.launch.py`: Gazebo + scan + localization + NAV2 + RViz.
- [ ] RViz "2D Goal Pose" → robot tự lập kế hoạch & đi tới, dừng đúng vị trí (±0.2 m).
- [ ] Né được vật cản tĩnh trong costmap; recovery (spin/backup) hoạt động khi kẹt.
- [ ] Gọi được action: `ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose ...`.

**Verification:** 3 goal khác nhau trong sim đều tới nơi; 1 goal có vật chắn → tự vòng tránh.

### ★ Checkpoint 3 (sau T7) — **Mốc "Apply NAV2 để điều hướng" đạt ở mức mô phỏng**
- [ ] Toàn bộ stack SLAM+Nav chạy end-to-end trong sim. Demo quay video.
- [ ] Review trước khi đụng phần cứng.

---

## Phase 5 — Ghép phần cứng thật

### T8 — base_bridge node (rclpy)
**Scope:** M · **Deps:** ~~T7~~ (làm SỚM, độc lập) + đế sẵn sàng cho test cuối (Open Q2)

**Acceptance criteria:**
- [x] Sub `/cmd_vel` (`geometry_msgs/Twist`) → encode `SET_ROBOT_VELOCITY {cmd=3, vx, vy, vθ}` (13B) → gửi đế. **Transport abstract** (`TcpTransport` đầy đủ — TCP :2004; `CanTransport` STUB raise NotImplementedError — 13B > 8B/frame, làm khi chốt Q2). ✓
- [x] Đọc telemetry `ROBOT_STATE` (pose EKF) → publish `nav_msgs/Odometry` `/odom` + broadcast tf `odom→base_link`. ✓
- [x] STOP_ROBOT khi `/cmd_vel`=0 quá lâu (watchdog `cmd_timeout`) hoặc shutdown. ✓
- [x] **Test loopback (ESP32 GIẢ, không cần đế thật):** cmd_vel→server giả nhận đúng `vx/vy/vθ` (7 lệnh @5Hz); telemetry giả→`/odom`+tf (12 msg, x tăng đúng). 13 unit-test byte-format PASS. ✓
- [ ] Test với ĐẾ THẬT: gửi goal vx nhỏ → đế chạy đúng hướng; `/odom` phản ánh chuyển động. *(chờ đế)*

**Files:** `ros2_ws/src/base_bridge/base_bridge/{base_node.py,transport.py,protocol.py,fake_odom.py}` + `test/test_protocol.py`.
**Verification:** `ros2 run base_bridge base_node --ros-args -p host:=<ip>` → `ros2 topic pub /cmd_vel ...` → đế nhúc nhích; `ros2 topic echo /odom`.

**✅ XONG SỚM (2026-06-25) — code + test KHÔNG cần phần cứng:** Tách 3 lớp: `protocol.py` (encode/decode thuần, 13 unit-test PASS) + `transport.py` + `base_node.py` (watchdog STOP, auto-reconnect) + `fake_odom`. Loopback test ESP32 giả PASS 2 chiều.

> ⚠️ **CẬP NHẬT 2026-06-26 — ĐỌC Source_code (firmware + ROS2 của partner) → ĐỔI HƯỚNG T8.** Xem **§6 (Kế hoạch ghép đế chi tiết)** bên dưới. Tóm tắt:
> - ✅ **Byte-format `protocol.py` của tôi KHỚP 100% firmware** (`velocity_t`=3 float→13B, `state_t`={vel,pos}=6 float→25B). Không còn là giả định.
> - 🔴 **TOPOLOGY ĐẢO:** firmware nay là **TCP CLIENT** `connect()` tới `192.168.137.1:2004`; partner đã viết **node ROS2 = TCP SERVER** (`cpp_package/wifi_node`). → `base_node.py` (client) của tôi **SAI CHIỀU**, không dùng trực tiếp. **Open Q2 (CAN) HUỶ — chốt WiFi/TCP, Jetson=server.**
> - 👉 **Quyết định: dùng `cpp_package` của partner làm cầu nối chính** (khớp firmware), **port logic đã test của tôi** (watchdog STOP, ráp frame, fix kích thước gửi) vào đó. `protocol.py`+unit-test giữ làm spec đối chiếu; `base_node.py` archive.

---

### T9 — astra_ros node + SLAM/Nav trên Jetson thật
**Scope:** M · **Deps:** T8

**Acceptance criteria:**
- [x] Node wrap `AstraCamera(mode="depth")` → publish `/camera/depth/image_raw` (16UC1, mm) + `/camera/depth/camera_info` (fx=fy=570.34, cx=319.5, cy=239.5) ✓ *(làm sớm trong T4)*
- [~] FPS ≥10 trên Jetson; không leak USB sau 5 phút. *(driver thuần 29.7 FPS; qua ROS consumer d2l ~6Hz dưới tải — ĐỦ cho SLAM teleop chậm. Đã đổi QoS BEST_EFFORT (khớp d2l) + numpy 1-copy. ⚠️ Rate KHÔNG đo tin cậy được từ subscriber Python — ảnh 614KB deserialize chậm + BEST_EFFORT rớt mẫu; cần consumer C++/rosbag để đo thật)*
- [ ] Thay Gazebo bằng `astra_ros`+`base_bridge`, chạy lại T4→T7 trên phần cứng → quét map **khu vực nhỏ thật** + đi tới điểm. *(blocked — cần đế thật)*

**Files:** `ros2_ws/src/astra_ros/astra_ros/astra_node.py`.
**Verification:** Trên Jetson: lái tay (teleop) quanh phòng nhỏ → `/scan` thật → slam_toolbox ra map thật → lưu map → NavigateToPose tới điểm.

### ★ Checkpoint 4 (sau T9) — **Map thật + Nav thật**
- [ ] Quét được map khu vực nhỏ thật, lưu được; robot tự đi tới điểm trên map đó.
- [ ] Đo: độ chính xác dừng, tần suất "lost localization" (kỳ vọng cao do FOV hẹp → ghi nhận để tune).

---

## Phase 6 — Tích hợp VLM (handoff điều hướng → gắp)

### T10 — vlm_nav_orchestrator node
**Scope:** L · **Deps:** T9 + pipeline VLM hiện có

**Acceptance criteria:**
- [ ] Nhận mục tiêu (tên vật tiếng Việt) → state machine:
  1. **NAV** — NavigateToPose tới waypoint khu vực (depth mode cho costmap).
  2. **SWITCH** — dừng nav, chuyển Astra sang `color` (USB2 không song song — xem [[project-jetson-astra]]).
  3. **DETECT** — gọi Brain `POST :8000/analyze` (YOLO+VLM) tìm vật.
  4. **SERVO** — visual servoing publish `/cmd_vel` (vx,vy,vθ): giảm X ngang→vy, giảm Z tiến→vx (toggle color/depth `both` ~2 FPS để lấy Z).
  5. **HANDOFF** — khi đủ gần → trigger gắp (tay STM32, để sau) hoặc dừng + báo.
- [ ] Hủy/abort an toàn ở mọi state (STOP_ROBOT).
- [ ] Tích hợp HTTP Brain (giữ nguyên) + ROS2 (rclpy spin thread riêng, pattern hybrid như Brain).

**Files:** `ros2_ws/src/vlm_nav_orchestrator/{orchestrator_node.py, geometry.py}` + `test/test_geometry.py`.
**Verification:** Sim/bench: ra lệnh "lại chỗ cái chai" → robot nav tới khu vực → (camera thật) detect → servo căn giữa.

**🟡 V1 XONG (2026-06-27) — chọn Hybrid (user chốt):** `vlm_nav_orchestrator` state machine **DETECT → LOCATE → NAVIGATE → APPROACH → ARRIVED**:
- **DETECT**: đọc `/yolo/detections` (từ `yolo_ros`) → có vật mục tiêu? (true/false). *(YOLO đủ; VLM grounding theo tên là lớp thêm sau.)*
- **LOCATE**: depth ở tâm bbox + intrinsics → điểm 3D → tf2(camera→map) → pose vật trên map.
- **NAVIGATE**: `NavigateToPose(standoff goal)` → NAV2 tự lập đường + né (depth/scan).
- **APPROACH**: visual servoing `/cmd_vel` (vθ căn giữa bbox + vx tiến tới `target_z`) → ARRIVED.
- Publish `/camera_mode` (color|depth|both) — yêu cầu chế độ camera; cần "camera manager" đáp ứng (xem 6.6).
- **Lõi toán `geometry.py` (pixel→3D, transform→map, standoff goal, servo): 11/11 unit-test PASS.** Node build OK; **wiring sim-free PASS** (fake detection+depth+tf → DETECT→LOCATE→NAVIGATE, tính goal + gửi NavigateToPose; nav fail êm khi thiếu server).
- ⚠️ **CẦN ĐẾ + NAV2 live** để nghiệm thu thật: navigation, servo APPROACH, và **camera choreography** (hot-switch USB2 color↔depth↔both — cần 1 camera-manager node, là việc tiếp theo, xem 6.6).

---

### T11 — End-to-end test + documentation
**Scope:** M · **Deps:** T10

**Acceptance criteria:**
- [ ] Kịch bản đầy đủ chạy ≥3 lần ổn định (nav → detect → servo).
- [ ] `docs/SLAM_NAV2_RUNBOOK.md`: cách quét map, lưu map, chạy nav, troubleshooting (lost localization, scan rỗng, USB toggle).
- [ ] Cập nhật `README.md` + sơ đồ kiến trúc; ghi rõ giới hạn (FOV hẹp, tầm ngắn) + lộ trình LiDAR.

### ★ Checkpoint 5 (Go/No-go cuối)
- [ ] Full pipeline đạt; nếu visual-servoing/USB toggle quá yếu → fallback: dừng ở "nav tới khu vực + báo người".

---

## 6. GHÉP ĐẾ THẬT — Kế hoạch chi tiết (cập nhật 2026-06-27)

> **Cập nhật 2026-06-27:** partner thêm `explorer_node` (WFD autonomous exploration) + `frontier_based_exploration` + `config/slam_params.yaml`, và nav2_params chuyển costmap sang `/scan`. **Phát hiện lớn:** partner đang phát triển/test **toàn bộ stack tự hành trên TurtleBot3 Gazebo sim** (LiDAR 360° 3.5m, `use_sim_time:true`) — KHÔNG phải Astra thật. Xem 6.3a + 6.6.

### 6.1 Những gì đọc được trong `Source_code/` (cập nhật 2026-06-27)
| Thành phần | Nội dung | Đánh giá |
|-----------|----------|----------|
| `Mecanum_robot/` (ESP32, ESP-IDF) | Firmware đế: kinematic, motor PI, EKF (BNO055+PMW3901), **wifi (STA)**, socket_handler, mdns, **canbus (có nhưng KHÔNG dùng)** | Hoàn chỉnh. `message_type.h` = ground-truth protocol. |
| `cpp_package/wifi_node` + `socket_handler` | TCP server :2004. Sub `/cmd_vel`→SET_ROBOT_VELOCITY; ROBOT_STATE→pub `/odom`+tf `odom→base_link` @50Hz. mDNS Avahi. **(socket_handler KHÔNG đổi từ 06-26)** | Đúng topology, **còn lỗi — 6.3**. |
| `cpp_package/explorer_node` + `frontier_based_exploration` **(MỚI 06-27)** | **WFD autonomous exploration**: sub `/map`+`/odom`, detect frontier (BFS 2 lớp), gửi NAV2 `navigate_to_pose` tới frontier gần nhất; `spinScan` xoay 360° quét tại mỗi waypoint (bù FOV hẹp ✓); blacklist frontier hỏng. | **Thuật toán tốt, nhưng nguy hiểm với Astra 1m — 6.3a.** Thay lái-tay bằng tự hành. |
| `cpp_package/config/{slam_params,nav2_params}.yaml` **(slam MỚI, nav2 cập nhật 06-27)** | slam: LiDAR 3.5m, `use_sim_time:true`, "AI generate, robot thật sửa sau". nav2: **đã chuyển pointcloud→`/scan`** ✓, DWB holonomic, `use_sim_time:true`, robot_radius 0.1, dùng `/map` từ slam (không map_server). | **CONFIG SIM/TurtleBot3 — phải swap sang Astra (config của tôi) cho phần cứng thật.** |
| `Multi_platform_app/`, `Jetson/Can_node.cpp`, `STM32_Robot_arm/` | App C# (tham chiếu); CAN cũ (BỎ); cánh tay (pha sau) | — |

### 6.2 Kiến trúc ĐÚNG (xác nhận từ code, KHÁC giả định cũ)
```
ESP32 (TCP CLIENT, wifi STA) ──connect()──► 192.168.137.1:2004 ◄── Node ROS2 (TCP SERVER, cpp_package/wifi_node) trên Jetson
   gửi (Jetson→ESP32): payload thô, byte[0]=cmd, KHÔNG length  → SET_ROBOT_VELOCITY(13B)/STOP(1B)
   gửi (ESP32→Jetson): [1B length][payload] ×4 mỗi 200ms        → MOTOR_SPEED, BNO055, ROBOT_STATE(25B), PMW3901
```
- **`protocol.py` của tôi KHỚP 100%** firmware (`velocity_t`=3float→13B; `state_t`={vel,pos}=6float→25B). Giữ làm spec + 13 unit-test đối chiếu.
- **Jetson là SERVER** → `base_node.py` (client) của tôi sai chiều, **không dùng**. Dùng `wifi_node` của partner.

### 6.3 ⚠️ Lỗi/thiếu sót trong `wifi_node` partner (cần sửa khi ghép — W2)
1. 🔴 **KHÔNG có watchdog/STOP**: `/cmd_vel` ngừng nhưng TCP còn → đế giữ vận tốc cuối → **xe chạy hoang**. Firmware chỉ `robot_brake()` khi mất kết nối. → **thêm watchdog gửi STOP_ROBOT khi cmd_vel im >0.5s** (port từ `base_node.py`).
2. 🔴 **Ráp frame TCP sai**: `receive_task` đọc `rx_buffer+1` chỉ lấy cmd của frame ĐẦU mỗi `recv()`. Firmware gửi 4 frame liền (MOTOR_SPEED, BNO055, **ROBOT_STATE**, PMW3901) → TCP gộp → node thường **bỏ lỡ ROBOT_STATE → /odom đứng yên (=0)**. → **thêm FrameParser `[len][payload]`** (logic đã test trong `protocol.py`).
3. 🟠 **Gửi cmd_vel `sizeof(generic_msg_t)`** (union ~97B) thay vì `sizeof(velocity_msg_t)`=13B → lãng phí + dễ lệch khung. → gửi đúng 13B.
4. ✅ ~~costmap đọc `/camera/depth/points`~~ **ĐÃ SỬA (06-27): nav2 partner nay dùng `/scan` (LaserScan)** — khớp astra_ros+d2l của ta. (Pointcloud đã comment.)
5. 🟠 **nav2 partner DÙNG vy holonomic (DWB)** — trái quyết định plan (NAV2=vx+vθ, vy để dành servoing); DWB holonomic finicky/nặng trên Foxy. → cân nhắc giữ **RPP (nav2_params.yaml của tôi)** hoặc thống nhất lại.
6. 🟡 odom thiếu `twist`+covariance; `package.xml` thiếu khai báo `rclcpp_action`/`nav2_msgs`/`tf2_geometry_msgs`/avahi (build OK vì gói đã cài, nhưng nên bổ sung); maintainer/license = TODO.

### 6.3a 🔴 RỦI RO LỚN MỚI: Tự hành (WFD) + config SIM vs Astra thật
- **Config toàn bộ là SIM/TurtleBot3** (`use_sim_time:true`, slam `max_laser_range:3.5`, LiDAR 360°). Trên phần cứng thật PHẢI swap: `use_sim_time:false`, slam `max_laser_range:1.0` + tune Astra (dùng `slam_toolbox.yaml` của tôi), nav2 robot_radius theo đế thật (không 0.1), range theo `/scan` Astra.
- **Tự hành (explorer) + Astra cap 1m = nguy hiểm:** explorer lái robot tới frontier (vùng chưa biết). Astra chỉ "thấy" ~1m + FOV 58° → costmap gần như trống ngoài 1m → **robot lao vào vùng chưa cảm nhận được → dễ đâm**. `spinScan` 360° tại waypoint giúp dựng map cục bộ 1m trước khi đi (giảm thiểu tốt), nhưng KHÔNG khắc phục tầm 1m. Với LiDAR 3.5m (sim) thì mượt; với Astra thật rủi ro cao.
  → **Giảm thiểu:** vận tốc rất chậm; khu vực nhỏ nhiều đặc trưng; **lái-tay quét map TRƯỚC (validate SLAM+nav an toàn), bật explorer SAU** khi đã tin stack; cân nhắc ngưỡng frontier xa bị chặn; bắt buộc có watchdog STOP (6.3 #1).
- **Khớp tốt:** `spinScan` xoay-tại-chỗ chính là chiến lược bù FOV hẹp plan đã nêu (T5). Tận dụng được.

### 6.4 Bảng task ghép đế (thay Phase 5 cũ) — W0→W7 (cập nhật 06-27)
| ID | Task | Scope | Chặn bởi | Ghi chú |
|----|------|-------|----------|---------|
| **W0** | **✅ CHỐT A: USB WiFi làm AP @`192.168.137.1`** + DHCP; `WIFI_SET` ESP32 join AP. KHÔNG sửa firmware. | S | — | Internet/SSH ở Ethernet `enp2s0` (192.168.1.250) → WiFi rảnh. Demo di động: ESP32+laptop cùng join AP. ⚠️ Verify dongle có "AP" (`iw list`). |
| **W1** | ✅ **XONG (2026-06-27):** Copy `cpp_package`→`ros2_ws/src/` (rsync, giữ `Source_code/` nguyên) + bổ sung deps `package.xml` (rclcpp_action/nav2_msgs/tf2_geometry_msgs). **5 package build sạch**, đủ executable `wifi_node`+`explorer_node`+`astra_node`+`fake_odom`. Đã dọn artifact build/install/log + pycache. CÒN LẠI: chạy `wifi_node` bind :2004 (cần W0 mạng + đế). | S | W0 | Gộp 1 workspace cạnh astra_ros/robot_bringup. base_bridge = reference-only (protocol.py spec + fake_odom test). |
| **W2** | ✅ **XONG (2026-06-27):** Harden `wifi_node` + **code-review end-to-end → sửa 3 BLOCKER**: (B1) `cmd_timeout` mặc định **1.5s** (0.5s gây phanh-giật khi explorer spin 1Hz); (B2) `recv` EINTR → continue (không giết luồng → giữ /odom sống); (B3) `is_running` **atomic** + destructor `shutdown(listen_sock)` → **Ctrl-C thoát sạch** (trước kẹt ở accept). + FrameParser ráp khung, gửi 13B, odom twist, mutex, bỏ send_task chết. **Re-verify PASS hết:** build 5pkg, unit 13/13, velocity 13B (10 msg), watchdog STOP, /odom từ burst (x tăng, twist), SIGINT thoát 1s. | M | W1 | 🔴 An toàn + /odom tin cậy. Thêm tool `cpp_package/test/fake_esp32.py` + `docs/HANDOFF.md` để partner test không cần phần cứng. |
| **W4** | ✅ **XONG (2026-06-27) — config Astra sẵn:** dùng `robot_bringup/config/{slam_toolbox,nav2_params}.yaml` (use_sim_time:False, slam max_range 1.0, nav2 **RPP** + range 0.9/1.0 + robot_radius 0.22 giả định). KHÔNG dùng config sim cpp_package. CÒN: đo robot_radius/hand-eye thật (B3 runbook). | M | — | Chốt RPP (vx+vθ); DWB-holo partner giữ làm bản sim. |
| **#3** | ✅ **XONG (2026-06-27):** `bringup.launch.py` (full stack: wifi_node/fake_odom + astra+scan + slam + nav2 lifecycle + explorer + rosbridge, tham số `odom/nav/autonomous/cmd_timeout`) + sửa `slam.launch.py` odom→wifi_node. LaunchDescription validate OK. | M | — | "run" = 1 lệnh. |
| **#4/#5** | ✅ **XONG (2026-06-27):** `tools/setup_robot_ap.sh` (dựng AP 192.168.137.1, verify AP-mode) + `docs/DEMO_RUNBOOK.md` (connect→config→run + troubleshooting). | S | — | Chờ cắm dongle để chạy script. |
| **W3** | **Bench test đế thật 2 chiều**: `/cmd_vel`→đế đúng vx/vy/vθ; `/odom` phản ánh pose; STOP khi ngừng | M | W2 + **đế** | Thay loopback ESP32-giả bằng đế thật. |
| **W5** | **SLAM thật — lái TAY (=T5):** `ros2 launch robot_bringup bringup.launch.py nav:=false autonomous:=false` + teleop. Quét map nhỏ → lưu. | M | W3 | Validate SLAM AN TOÀN trước tự hành. |
| **W6** | **NAV2 + Tự hành (=T6/T7):** `ros2 launch robot_bringup bringup.launch.py` → nav tay (goal Foxglove) → bật explorer WFD. | L | W5 | 🔴 Rủi ro cao (6.3a): chậm, khu vực nhỏ, watchdog bật. |
| → | Sau đó **T10** (VLM handoff) → **T11** (end-to-end + docs) | | | Như cũ. |

> **✅ Pre-hardware XONG (2026-06-27):** W1, W2 (+test), W4 (config), #3 (launch), #4/#5 (AP script + runbook). **Khi ghép đế chỉ còn:** cắm USB WiFi → `sudo bash tools/setup_robot_ap.sh` → WIFI_SET ESP32 → đo kích thước (B3) → `ros2 launch ... bringup.launch.py` → tune tốc độ. Xem `docs/DEMO_RUNBOOK.md`.
> **Còn cần đế (không rút gọn được):** W3 bench, W5 quét map thật, W6 tune tự hành; đo robot_radius/hand-eye.
> **Chiến lược an toàn:** W5 lái tay validate SLAM trước → W6 mới bật tự hành. Config đã đổi SIM→Astra (W4).

### 6.5 Hand-eye `base_link→camera` (Open Q3) vẫn cần
Đo đế thật + camera trước W4/W5 (hiện URDF dùng giá trị giả định [0.15,0,0.20]).

### 6.6 ✅ CAMERA MANAGER (hot-switch USB2) — XONG (2026-06-27)
**`astra_ros/camera_manager`**: 1 node giữ DUY NHẤT `AstraCamera`, sub `/camera_mode`
(depth|color|both) → `close()`+mở lại đúng mode → publish `/camera/depth/image_raw`(+info)
HOẶC `/camera/color/image_raw` (both = cả 3, toggle ~2FPS). Thay `astra_node`+`astra_color_node`
khi chạy T10. Đáp ứng `/camera_mode` mà `orchestrator_node` phát → các pha DETECT(color)/
NAVIGATE(depth)/APPROACH(both) tự chuyển camera. Build OK; **✅ TEST LIVE PASS (camera thật, 2026-06-27):** depth mode→chỉ depth+info; color mode→chỉ color; both→cả hai; switch sạch, SIGINT đóng camera không orphan, camera mở lại OK.

**Test camera_manager (khi camera rảnh — tắt yolo.launch/astra trước):**
```
ros2 run astra_ros camera_manager                                  # mở mode depth mặc định
ros2 topic pub --once /camera_mode std_msgs/msg/String "{data: color}"   # → switch sang color (~1-2s)
ros2 topic list | grep camera     # depth mode: /camera/depth/*; color mode: /camera/color/image_raw
```
→ Mảnh ghép cuối: nay đủ để ráp launch T10 (camera_manager + yolo + slam + nav2 + orchestrator) chạy trên đế.

**✅ Launch T10 tổng: `robot_bringup/launch/collect.launch.py`** (XONG, validate 17 entities) —
camera_manager + depthimage_to_laserscan + odom(wifi|fake) + slam_toolbox + NAV2(lifecycle) +
yolo_node + vlm_nav_orchestrator + rosbridge. `target_class:=bottle`, `nav:=false`/`odom:=fake` để test bộ phận.
CÒN: chạy live trên ĐẾ (cần odom thật + di chuyển) + tune; (lưu ý orchestrator cycle nhanh khi
thiếu NAV server sẽ thrash camera switch — trên đế thật NAVIGATE giữ lâu nên OK).

Trình tự T10 còn lại: (a) camera_manager; (b) chạy live trên đế: explorer/teleop → orchestrator DETECT→…→ARRIVED;
(c) tune standoff/servo gains/target_z; (d) (tùy) thêm VLM grounding chọn vật theo tên.

---

## 3. Risks & Mitigations

| Rủi ro | Mức | Giảm thiểu |
|--------|-----|-----------|
| **Astra cap ~1m (XÁC NHẬN T0) → mù với mọi thứ >1m** | 🔴 Cao (hiện thực) | User chấp nhận. Giảm thiểu: chỉ map **khu vực nhỏ ≤4×4m nhiều đặc trưng**, **bám tường trong 1m**, odom-heavy. Map trôi ở khoảng trống → use-case VLM dung thứ. **LiDAR 2D = đường nâng cấp** (chất lượng nhảy vọt). |
| FOV 58.6° (không 360°) → scan matching/loop closure yếu | 🟠 TB | **Xe quay+di chuyển khắc phục FOV** (cung 58° quét dần); lái chậm nhiều overlap; tune slam_toolbox travel-dist nhỏ. |
| USB2 không stream depth+RGB đồng thời (nav cần depth, VLM cần RGB) | 🟠 TB | Phân pha: nav=depth, detect=color (switch); servo=`both` toggle ~2 FPS; ghi rõ trong T10. |
| Disk 4.4 GB cạn khi cài Gazebo/nav2 | 🟠 TB | `apt clean` trước (T1); **chạy sim trên desktop**; theo dõi `df -h`. |
| Đế chưa kết nối → không test hardware | 🟠 TB | Sim-first (T2–T7) độc lập phần cứng; base_bridge transport abstract. |
| Foxy EOL (2023), nav2 0.4.7 cũ, vài bug | 🟡 Thấp | Bám config Foxy đã kiểm chứng; dùng slam_toolbox-localization thay AMCL. |
| 13B velocity struct > 8B/frame CAN | 🟡 Thấp | Bắt đầu TCP :2004; CAN multi-frame/CAN-FD làm sau (Open Q2). |
| Gazebo headless trên Jetson (no display) | 🟡 Thấp | `gzserver` + RViz qua VNC/X-forward, hoặc sim ở desktop. |

---

## 4. Open Questions (cần user quyết)

1. ~~Chạy Gazebo ở đâu?~~ **✅ ĐÃ CHỐT (2026-06-25):** `asiclab-desktop` = chính Jetson (không có máy riêng). Sim chạy `gzserver` headless trên Jetson; xem demo qua **Foxglove Studio** trên laptop/điện thoại qua `rosbridge` (§1.6). Theo dõi disk khi cài Gazebo (~1 GB).
2. ~~Đế kết nối TCP hay CAN?~~ **✅ ĐÃ CHỐT (2026-06-26, đọc Source_code):** **WiFi/TCP** (KHÔNG CAN). Firmware = TCP client → `192.168.137.1:2004`; node ROS2 partner = TCP server. **Câu hỏi mới (W0):** Jetson lấy IP `192.168.137.1` kiểu gì? (chạy hotspot/AP, hay sửa `DEST_IP_ADDR` firmware về IP Jetson trên LAN chung). Xem §6.
3. **Hand-eye `base_link→camera`** đo tay hay có bản vẽ cơ khí? Ảnh hưởng độ chính xác costmap + servo. → cần trước T9 (tạm dùng giá trị giả định + TODO).
4. **Kết quả T0** quyết định toàn bộ: nếu tầm ~1 m, có sẵn sàng đổi sang **LiDAR 2D (~$70–100)** không? (Khuyến nghị mạnh cho map phòng đáng tin cậy.)

---

## 5. Summary table

| ID | Task | Scope | Deps | Phase |
|----|------|-------|------|-------|
| T0 | Spike tầm depth Astra (BLOCKING) | S | — | 0 | ✅ XONG |
| T1 | Dọn disk + cài gói Foxy | S | — | 0 | ✅ XONG (Gazebo hoãn) |
| **★** | **Checkpoint 1: depth đủ dùng + disk OK** | — | T0,T1 | — | ✅ ĐẠT |
| T2 | Workspace + robot_description + static TF | M | T1 | 1 | ✅ XONG |
| T3 | Gazebo sim (base+depth+odom) | L | T2 | 1 | ⏸️ HOÃN (disk chật) |
| T4 | depthimage_to_laserscan → /scan | S | T3→bypass | 2 | ✅ XONG (camera thật) |
| **★** | **Checkpoint 1.5** | — | T3 | — | ⏸️ bỏ qua (T3 hoãn) |
| T5 | slam_toolbox → lưu map | M | T4+T8 | 3 | 🟡 CONFIG XONG+smoke-test PASS; quét đầy đủ chờ đế |
| **★** | **Checkpoint 2: quét+lưu map** | — | T5 | — | 🔲 (chờ đế) |
| T6 | NAV2 params | L | T5 | 4 | 🟡 CONFIG XONG (RPP); lifecycle chờ map thật |
| T7 | nav2_bringup: NavigateToPose | M | T6 | 4 | 🔲 (chờ đế) |
| **★** | **Checkpoint 3: nav end-to-end** | — | T7 | — | 🔲 |
| T8 | base_bridge (cmd_vel↔đế, odom) | M | (sớm) | 5 | 🟡 CODE+loopback test PASS; test đế thật chờ Q2 |
| T9 | astra_ros thật trên Jetson | M | T8 | 5 | ✅ XONG sớm (+QoS/numpy) |
| **★** | **Checkpoint 4: map thật + nav thật** | — | T9 | — | 🔲 |
| T10 | vlm_nav_orchestrator (detect→đi tới, Hybrid) | L | T9 | 6 | 🟡 V1: geometry 11/11 test + node wiring sim-free PASS; cần đế + camera_manager (6.6) |
| T11 | End-to-end + docs | M | T10 | 6 | 🔲 |
| **★** | **Checkpoint 5: Go/No-go cuối** | — | T11 | — | 🔲 |

**Critical path (cập nhật):** T0✅→T1✅→T2✅→(T3 hoãn)→T4✅→T8🟡(code+test xong)→T5🟡(config+smoke xong)→T6🟡(config xong)→T7→T10→T11.
→ **Mọi thứ làm được KHÔNG cần đế ĐÃ XONG.** Nút chặn duy nhất còn lại: **đế mecanum thật** (`/odom` + chuyển động) để quét map đầy đủ (T5) + nghiệm thu NAV2 lifecycle (T6/T7).

**Việc đã làm trước khi ghép đế (2026-06-25, phiên 2):**
- **T8 `base_bridge`**: protocol/transport/node + `fake_odom` + 13 unit-test + loopback test (ESP32 giả) cả 2 chiều PASS.
- **T5 `slam_toolbox.yaml`** + `slam.launch.py`: smoke-test sim-free (camera thật + fake_odom) → /map + tf map→odom PASS.
- **T6 `nav2_params.yaml`**: RPP controller, costmap theo tầm Astra, YAML hợp lệ.
- **astra_node**: QoS BEST_EFFORT (khớp d2l) + numpy 1-copy.

**🔴 CẬP NHẬT LỚN 2026-06-26 (đọc `Source_code/`):** Phase 5 ghép đế được THAY bằng **§6 (W0→W6)**. Topology đảo (Jetson=TCP server, ESP32=client@192.168.137.1, WiFi không CAN). **Dùng `cpp_package/wifi_node` của partner** (khớp firmware) + **port logic đã test của tôi** (watchdog STOP, FrameParser, gửi 13B) — xem lỗi 6.3. `base_node.py` (client) archive; `protocol.py`+unit-test giữ làm spec (đã xác minh khớp firmware 100%). Asset của tôi còn dùng: `slam_toolbox.yaml`, `astra_ros`(/scan), `fake_odom`, và `nav2_params.yaml` (/scan + RPP, tốt hơn pointcloud-4m của partner cho Astra cap 1m).

**Khi có đế (theo §6):** W0 (mạng 192.168.137.1) → W1 (build cpp_package) → W2 (harden) → W3 (bench) → `ros2 launch` gộp `wifi_node`+`astra_ros`+`slam_toolbox` → W4 quét map → W5/W6 NAV2.

---

*Plan cập nhật lần cuối: 2026-06-27 (phiên chiều): GỘP cpp_package + HARDEN wifi_node (watchdog/FrameParser/13B, loopback PASS) + bringup.launch.py + config Astra + tools/setup_robot_ap.sh + docs/DEMO_RUNBOOK.md → pre-hardware XONG, ghép đế chỉ còn config+run. Sáng cùng ngày: partner thêm explorer_node WFD + config sim → §6.3a rủi ro tự hành. Trước: 2026-06-26 topology WiFi/server; 2026-06-25 T8/T5/T6 sim-free.*
