"""
protocol.py - Encode/decode the ESP32 mecanum base protocol (PURE PYTHON, no ROS).
=================================================================================
Mirrors the byte format of the ESP32 firmware (separate repo: github.com/kamuisi/Collecting-robot
-> Source_code/Mecanum_robot, file main/socket_handler/include/message_type.h).

Kept separate from node.py so it can be UNIT-TESTED WITHOUT ROS/hardware.

Convention (confirmed from message_type.h / socket_handler.c):
  - `#pragma pack(1)`, little-endian ('<').
  - **ASYMMETRIC:**
      Jetson -> ESP32: send the payload struct DIRECTLY, byte[0]=cmd, NO length byte.
      ESP32 -> Jetson: [1 length byte][payload], length = sizeof(payload).
  - enum cmd_type_t (value = declaration order):
      WIFI_SET=0 OTA_UPDATE=1 SET_MOTOR_SPEED=2 SET_ROBOT_VELOCITY=3 STOP_ROBOT=4
      AUTO_TUNE=5 MOTOR_SPECS=6 MOTOR_SPEED=7 ROBOT_STATE=8 BNO055_DATA=9
      BNO055_RECALIBRATION=10 PMW3901_DATA=11

ASSUMPTIONS to verify against the firmware repo:
  - `ROBOT_STATE` telemetry layout: byte[0]=cmd(=8) then state_t = {velocity{vx,vy,vtheta},
    position{x,y,theta}} = 6x float32 (24B). Total payload 1+24=25B -> length byte=25.
  - Units: velocity m/s & rad/s, position m & rad (matches ROS Twist/Odometry semantics).
  If the firmware differs (e.g. adds a timestamp, or the cmd byte is absent from telemetry),
  only adjust the constants/layout in the "ROBOT_STATE" section below - the rest stays the same.
"""
import struct
from dataclasses import dataclass

# enum cmd_type_t
CMD_WIFI_SET = 0
CMD_OTA_UPDATE = 1
CMD_SET_MOTOR_SPEED = 2
CMD_SET_ROBOT_VELOCITY = 3
CMD_STOP_ROBOT = 4
CMD_AUTO_TUNE = 5
CMD_MOTOR_SPECS = 6
CMD_MOTOR_SPEED = 7
CMD_ROBOT_STATE = 8
CMD_BNO055_DATA = 9
CMD_BNO055_RECALIBRATION = 10
CMD_PMW3901_DATA = 11

# Struct layout (pack(1), little-endian)
# Jetson -> ESP32: SET_ROBOT_VELOCITY = {u8 cmd; float vx; float vy; float vtheta}
_VELOCITY_FMT = "<Bfff"          # 1 + 3*4 = 13 bytes
VELOCITY_SIZE = struct.calcsize(_VELOCITY_FMT)   # = 13

# Jetson -> ESP32: STOP_ROBOT = {u8 cmd}
_STOP_FMT = "<B"                 # 1 byte
STOP_SIZE = struct.calcsize(_STOP_FMT)           # = 1

# ESP32 -> Jetson: ROBOT_STATE payload = {u8 cmd; float vx,vy,vtheta; float x,y,theta}
_STATE_FMT = "<Bffffff"          # 1 + 6*4 = 25 bytes
STATE_PAYLOAD_SIZE = struct.calcsize(_STATE_FMT)  # = 25


# Encode: Jetson -> ESP32
def encode_set_velocity(vx: float, vy: float, vtheta: float) -> bytes:
    """SET_ROBOT_VELOCITY (13B). The ESP32 runs IK+PI itself to hold the velocity. No length byte."""
    return struct.pack(_VELOCITY_FMT, CMD_SET_ROBOT_VELOCITY, float(vx), float(vy), float(vtheta))


def encode_stop() -> bytes:
    """STOP_ROBOT (1B)."""
    return struct.pack(_STOP_FMT, CMD_STOP_ROBOT)


# Decode: ESP32 -> Jetson
@dataclass
class RobotState:
    """EKF pose telemetry (fused IMU BNO055 + optical flow PMW3901)."""
    vx: float
    vy: float
    vtheta: float
    x: float
    y: float
    theta: float


def decode_robot_state_payload(payload: bytes) -> RobotState:
    """
    Decode one ROBOT_STATE payload (NOT including the leading length byte).
    payload[0] must = CMD_ROBOT_STATE(8). Raises ValueError on wrong cmd/length.
    """
    if len(payload) != STATE_PAYLOAD_SIZE:
        raise ValueError(
            "ROBOT_STATE payload wrong length: %d (expected %d)" % (len(payload), STATE_PAYLOAD_SIZE)
        )
    cmd, vx, vy, vth, x, y, th = struct.unpack(_STATE_FMT, payload)
    if cmd != CMD_ROBOT_STATE:
        raise ValueError("Not a ROBOT_STATE: cmd=%d (expected %d)" % (cmd, CMD_ROBOT_STATE))
    return RobotState(vx=vx, vy=vy, vtheta=vth, x=x, y=y, theta=th)


class FrameParser:
    """
    Frame de-framer for the ESP32 -> Jetson byte stream: [1 length byte][payload] x N.
    TCP is a stream -> a recv() may deliver a partial frame or several frames at once.
    Feed bytes via feed(); get back the list of complete payloads (length byte stripped).

    Usage:  for payload in parser.feed(chunk): ...  # each payload is one raw message
    Dispatch by payload[0] (cmd) at the upper layer (node.py).
    """

    def __init__(self) -> None:
        self._buf = bytearray()

    def feed(self, chunk: bytes):
        """Feed new bytes; yield each complete payload (length byte not included)."""
        self._buf.extend(chunk)
        out = []
        while True:
            if len(self._buf) < 1:
                break
            length = self._buf[0]
            if length == 0:
                # garbage/desync frame -> drop 1 byte and retry
                del self._buf[0]
                continue
            if len(self._buf) < 1 + length:
                break  # payload not complete yet, wait for more
            payload = bytes(self._buf[1:1 + length])
            del self._buf[0:1 + length]
            out.append(payload)
        return out
