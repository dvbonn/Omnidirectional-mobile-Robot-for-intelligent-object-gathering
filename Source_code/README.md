# Source_code — Firmware & App

Mã nguồn phần cứng gốc: firmware đế mecanum (ESP32‑S3), firmware tay gắp (STM32F411), cầu CAN trên Jetson, và app điều khiển đa nền tảng (.NET MAUI).

> Stack ROS2 ([`../ros2_ws/`](../ros2_ws/)) **không sửa** thư mục này — giữ nguyên làm tham chiếu. `wifi_node` trong ROS2 nói chuyện với firmware ESP32 ở đây qua giao thức byte-format trong `Mecanum_robot/main/socket_handler/include/message_type.h`.

---

## Kiến trúc phần cứng

```
        ┌── Jetson AGX Xavier (ROS2) ──┐         WiFi TCP :2004        ┌── ĐẾ ESP32-S3 ──┐
        │  wifi_node  ◀──/cmd_vel──────┼─────────────────────────────▶│ socket_handler  │
        │             ──/odom──────────┼◀────── ROBOT_STATE ──────────│ motor_control   │
        │  Can_node (ISOTP can0) ◀─────┼── CAN ──▶ tay gắp?            │ EKF · BNO055    │
        └──────────────────────────────┘                              │ PMW3901 (flow)  │
                                                                       └─────────────────┘
   App MAUI (Android/iOS/Win/Mac) ── WiFi/mDNS ──▶ đế: cấu hình, teleop, xem chart
```

---

## Thành phần

| Thư mục | Nền tảng | Vai trò |
|---|---|---|
| `Mecanum_robot/` | **ESP32‑S3** (ESP‑IDF) | Firmware đế: motor, IMU BNO055, optical‑flow PMW3901, EKF, WiFi TCP socket, CAN, NVS/SPIFFS |
| `STM32_Robot_arm/` | **STM32F411** (HAL + USB CDC) | Firmware tay gắp — điều khiển servo qua USB CDC |
| `Jetson/Can_node.cpp` | Jetson (Linux SocketCAN) | Cầu CAN ISO‑TP (`can0`, rx `0x100` / tx `0x101`) |
| `Multi_platform_app/Robot_controller/` | **.NET 9 MAUI** | App điều khiển: cấu hình WiFi/mDNS, teleop, biểu đồ motor/EKF |
| `ros2_ws/` | ROS2 | Bản gốc `cpp_package` (tham chiếu) — bản đầy đủ ở [`../ros2_ws/`](../ros2_ws/) |

---

## Build & flash

### Đế ESP32‑S3 (`Mecanum_robot/`)

```bash
# Cần ESP-IDF (v5.x) — . $IDF_PATH/export.sh
cd Source_code/Mecanum_robot
idf.py set-target esp32s3
idf.py build
idf.py -p <PORT> flash monitor
```
Cấu hình WiFi: gửi lệnh `WIFI_SET` (ssid=`MecanumRobot`, pass=`robot12345`) qua app MAUI/serial → đế tự `connect()` tới Jetson `192.168.137.1:2004`.

### Tay gắp STM32F411 (`STM32_Robot_arm/`)

Mở `STM32_Robot_arm.ioc` bằng **STM32CubeIDE**, hoặc build bằng CMake preset:
```bash
cd Source_code/STM32_Robot_arm
cmake --preset Debug && cmake --build build/Debug
# Nạp: STM32CubeProgrammer hoặc st-flash write build/Debug/*.bin 0x8000000
```

### Cầu CAN Jetson (`Jetson/Can_node.cpp`)

```bash
sudo ip link set can0 up type can bitrate 500000    # bật SocketCAN
g++ Source_code/Jetson/Can_node.cpp -o can_node && ./can_node
```

### App MAUI (`Multi_platform_app/`)

```bash
# Cần .NET 9 SDK + workload MAUI: dotnet workload install maui
cd Source_code/Multi_platform_app/Robot_controller
dotnet build -t:Run -f net9.0-android          # hoặc net9.0-windows10.0.19041.0 / ios / maccatalyst
```
Target: `net9.0-android`, `net9.0-ios`, `net9.0-maccatalyst`, `net9.0-windows` (chỉ trên Windows).
