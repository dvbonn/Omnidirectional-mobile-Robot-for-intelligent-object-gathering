#!/usr/bin/env python3
"""
fake_esp32.py - Emulate the ESP32 base to test `wifi_node` WITHOUT hardware.
===========================================================================
Plays the role of the ESP32: it is a TCP CLIENT that connects to wifi_node (server :2004),
sends telemetry like the real firmware (a burst of 4 [len][payload] frames every 200ms) and
PRINTS every received command (SET_ROBOT_VELOCITY 13B / STOP_ROBOT 1B).

Use it to sanity-check wifi_node before plugging in the real base:
  1) Terminal A:  ros2 run cpp_package wifi_node
  2) Terminal B:  python3 src/cpp_package/test/fake_esp32.py
  3) Terminal C:  ros2 topic pub -r 5 /cmd_vel geometry_msgs/msg/Twist \\
                    "{linear: {x: 0.2, y: -0.1}, angular: {z: 0.3}}"
  4) Check:       ros2 topic echo /odom   (x should increase - telemetry is flowing)
     Terminal B prints "RX VELOCITY ..." while /cmd_vel arrives; prints "RX STOP_ROBOT" when
     /cmd_vel stops for >cmd_timeout (watchdog).

Args: [host] [port] [seconds]   (default 127.0.0.1 2004 60)
Little-endian pack(1) protocol: velocity_t/state_t = float vx,vy,vtheta / x,y,theta.
"""
import socket
import struct
import sys
import threading
import time

HOST = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"
PORT = int(sys.argv[2]) if len(sys.argv) > 2 else 2004
DURATION = float(sys.argv[3]) if len(sys.argv) > 3 else 60.0

# cmd_type_t: SET_ROBOT_VELOCITY=3, STOP_ROBOT=4, MOTOR_SPEED=7, ROBOT_STATE=8,
#             BNO055_DATA=9, PMW3901_DATA=11
def telemetry_burst(x):
    """4 frames [len][payload] like the firmware sends every 200ms; ROBOT_STATE.x increases."""
    f_motor = struct.pack("<Bffff", 7, 1.0, 2.0, 3.0, 4.0)              # MOTOR_SPEED
    f_bno   = struct.pack("<BBf",   9, 3, 1.57)                         # BNO055
    f_state = struct.pack("<Bffffff", 8, 0.25, 0.0, 0.0, x, 0.0, 0.0)  # ROBOT_STATE (25B)
    f_pmw   = struct.pack("<Bff",   11, 0.1, 0.2)                       # PMW3901
    return b"".join(bytes([len(f)]) + f for f in (f_motor, f_bno, f_state, f_pmw))


def rx_loop(sock):
    """Read Jetson->ESP32 commands (raw payload, byte[0]=cmd, NO length byte)."""
    sock.setblocking(False)
    buf = b""
    nvel = 0
    while True:
        try:
            data = sock.recv(256)
            if not data:
                break
            buf += data
        except BlockingIOError:
            time.sleep(0.01)
            continue
        except OSError:
            break
        while buf:
            cmd = buf[0]
            if cmd == 3 and len(buf) >= 13:           # SET_ROBOT_VELOCITY
                _, vx, vy, vth = struct.unpack("<Bfff", buf[:13])
                nvel += 1
                print("  RX VELOCITY  vx=%.3f vy=%.3f vth=%.3f" % (vx, vy, vth), flush=True)
                buf = buf[13:]
            elif cmd == 4:                             # STOP_ROBOT
                print("  RX STOP_ROBOT  (watchdog/stop)", flush=True)
                buf = buf[1:]
            else:
                buf = buf[1:]                          # resync if misaligned


def main():
    try:
        sock = socket.create_connection((HOST, PORT), timeout=5)
    except OSError as e:
        print("x Could not connect to wifi_node %s:%d - did you run `ros2 run cpp_package wifi_node`? (%s)"
              % (HOST, PORT, e))
        sys.exit(1)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    print("Fake-ESP32 connected to wifi_node %s:%d - sending telemetry, waiting for commands..." % (HOST, PORT), flush=True)
    threading.Thread(target=rx_loop, args=(sock,), daemon=True).start()

    x = 0.0
    t0 = time.time()
    try:
        while time.time() - t0 < DURATION:
            x += 0.05
            sock.sendall(telemetry_burst(x))   # 5 Hz
            time.sleep(0.2)
    except (OSError, KeyboardInterrupt):
        pass
    finally:
        sock.close()
        print("Fake-ESP32 closed.", flush=True)


if __name__ == "__main__":
    main()
