"""
test_protocol.py - Unit tests for the base protocol byte format (pytest, NO ROS needed).
========================================================================================
Verify that base_bridge/protocol.py encodes/decodes per the ESP32 firmware spec.
Run:  cd ros2_ws/src/base_bridge && python3 -m pytest test/test_protocol.py -v
      (or python3 test/test_protocol.py to run quickly without pytest)
"""
import os
import struct
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from base_bridge import protocol as P  # noqa: E402


# Struct sizes
def test_velocity_size_13_bytes():
    assert P.VELOCITY_SIZE == 13
    assert len(P.encode_set_velocity(0.0, 0.0, 0.0)) == 13


def test_stop_size_1_byte():
    assert P.STOP_SIZE == 1
    assert len(P.encode_stop()) == 1


def test_state_payload_size_25_bytes():
    assert P.STATE_PAYLOAD_SIZE == 25


# Encode Jetson -> ESP32
def test_encode_velocity_cmd_byte_and_values():
    raw = P.encode_set_velocity(0.25, -0.1, 1.5)
    assert raw[0] == P.CMD_SET_ROBOT_VELOCITY == 3
    cmd, vx, vy, vth = struct.unpack("<Bfff", raw)
    assert cmd == 3
    assert abs(vx - 0.25) < 1e-6
    assert abs(vy + 0.10) < 1e-6
    assert abs(vth - 1.5) < 1e-6


def test_encode_velocity_little_endian_layout():
    # vx=1.0 (0x3F800000 LE) right after the cmd byte -> check byte order
    raw = P.encode_set_velocity(1.0, 0.0, 0.0)
    assert raw == bytes([3]) + struct.pack("<f", 1.0) + struct.pack("<f", 0.0) * 2


def test_encode_stop_cmd_byte():
    assert P.encode_stop() == bytes([4])
    assert P.encode_stop()[0] == P.CMD_STOP_ROBOT == 4


# Decode ESP32 -> Jetson
def _make_state_payload(vx, vy, vth, x, y, th):
    return struct.pack("<Bffffff", P.CMD_ROBOT_STATE, vx, vy, vth, x, y, th)


def test_decode_robot_state_roundtrip():
    payload = _make_state_payload(0.1, 0.2, 0.3, 1.0, 2.0, 0.5)
    st = P.decode_robot_state_payload(payload)
    assert abs(st.vx - 0.1) < 1e-6
    assert abs(st.vy - 0.2) < 1e-6
    assert abs(st.vtheta - 0.3) < 1e-6
    assert abs(st.x - 1.0) < 1e-6
    assert abs(st.y - 2.0) < 1e-6
    assert abs(st.theta - 0.5) < 1e-6


def test_decode_robot_state_wrong_cmd_raises():
    bad = struct.pack("<Bffffff", P.CMD_BNO055_DATA, 0, 0, 0, 0, 0, 0)
    try:
        P.decode_robot_state_payload(bad)
        assert False, "must raise ValueError on wrong cmd"
    except ValueError:
        pass


def test_decode_robot_state_wrong_length_raises():
    try:
        P.decode_robot_state_payload(b"\x08\x00\x00")
        assert False, "must raise ValueError on wrong length"
    except ValueError:
        pass


# FrameParser: [length][payload] streaming
def test_frameparser_single_frame():
    payload = _make_state_payload(0, 0, 0, 0, 0, 0)
    frame = bytes([len(payload)]) + payload   # length=25
    parser = P.FrameParser()
    out = parser.feed(frame)
    assert len(out) == 1
    assert out[0] == payload


def test_frameparser_split_across_recv():
    """TCP may cut a frame mid-way -> the parser must wait until complete before yielding."""
    payload = _make_state_payload(1, 2, 3, 4, 5, 6)
    frame = bytes([len(payload)]) + payload
    parser = P.FrameParser()
    assert parser.feed(frame[:10]) == []      # not enough yet
    out = parser.feed(frame[10:])             # now complete
    assert len(out) == 1
    st = P.decode_robot_state_payload(out[0])
    assert abs(st.x - 4.0) < 1e-6


def test_frameparser_multiple_frames_one_recv():
    """Several frames bunched into one recv() -> split them all."""
    p1 = _make_state_payload(0, 0, 0, 1, 0, 0)
    p2 = _make_state_payload(0, 0, 0, 2, 0, 0)
    blob = bytes([len(p1)]) + p1 + bytes([len(p2)]) + p2
    parser = P.FrameParser()
    out = parser.feed(blob)
    assert len(out) == 2
    assert P.decode_robot_state_payload(out[0]).x == 1.0
    assert P.decode_robot_state_payload(out[1]).x == 2.0


def test_frameparser_skips_zero_length_garbage():
    payload = _make_state_payload(0, 0, 0, 9, 0, 0)
    blob = b"\x00\x00" + bytes([len(payload)]) + payload   # 2 garbage bytes length=0
    parser = P.FrameParser()
    out = parser.feed(blob)
    assert len(out) == 1
    assert P.decode_robot_state_payload(out[0]).x == 9.0


if __name__ == "__main__":
    # Quick run without pytest
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    passed = 0
    for fn in fns:
        fn()
        print("  ok %s" % fn.__name__)
        passed += 1
    print("\n%d/%d tests PASS" % (passed, len(fns)))
