"""Unit tests for geometry.py (pytest or python3 directly - NO ROS needed)."""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from vlm_nav_orchestrator import geometry as g  # noqa: E402

FX = FY = 570.3422
CX, CY = 319.5, 239.5


def test_bbox_center():
    assert g.bbox_center([100, 50, 40, 20]) == (120.0, 60.0)


def test_pixel_center_is_optical_axis():
    # pixel at the image center -> X=Y=0, Z=z
    X, Y, Z = g.pixel_to_camera_point(CX, CY, 0.8, FX, FY, CX, CY)
    assert abs(X) < 1e-9 and abs(Y) < 1e-9 and abs(Z - 0.8) < 1e-9


def test_pixel_right_gives_positive_x():
    # pixel off to the right of center -> positive X (optical: X right)
    X, _, _ = g.pixel_to_camera_point(CX + 100, CY, 1.0, FX, FY, CX, CY)
    assert X > 0
    # magnitude: (100)*1.0/570.34
    assert abs(X - 100.0 / FX) < 1e-6


def test_transform_identity():
    assert g.transform_point((1, 2, 3), np.eye(4)) == (1.0, 2.0, 3.0)


def test_transform_translation():
    T = np.eye(4); T[0, 3] = 5.0; T[1, 3] = -2.0
    assert g.transform_point((1, 1, 0), T) == (6.0, -1.0, 0.0)


def test_standoff_goal_basic():
    # robot at origin, object at (2,0), standoff 0.3 -> goal (1.7, 0), yaw 0
    gx, gy, yaw = g.standoff_goal((0.0, 0.0), (2.0, 0.0), 0.3)
    assert abs(gx - 1.7) < 1e-6 and abs(gy) < 1e-6 and abs(yaw) < 1e-6


def test_standoff_goal_faces_object():
    # object at (0,2) -> yaw ~ +90 deg
    _, _, yaw = g.standoff_goal((0.0, 0.0), (0.0, 2.0), 0.3)
    assert abs(yaw - math.pi / 2) < 1e-6


def test_standoff_within_range_just_faces():
    # object 0.2m away < standoff 0.3 -> goal = robot position, just face it
    gx, gy, yaw = g.standoff_goal((1.0, 1.0), (1.2, 1.0), 0.3)
    assert abs(gx - 1.0) < 1e-6 and abs(gy - 1.0) < 1e-6 and abs(yaw) < 1e-6


def test_yaw_to_quat():
    qx, qy, qz, qw = g.yaw_to_quat(0.0)
    assert (qx, qy, qz, qw) == (0.0, 0.0, 0.0, 1.0)
    _, _, qz, qw = g.yaw_to_quat(math.pi)
    assert abs(qz - 1.0) < 1e-6 and abs(qw) < 1e-6


def test_servo_strafes_and_advances():
    # HOLONOMIC: bbox off to the right (u=500, w=640) -> strafe right (v_y<0) AND advance (v_x>0),
    # no turning, not arrived yet (not centered)
    vx, vy, arr = g.servo_cmd(500, 640, 0.8, 0.3)
    assert vy < 0 and vx > 0 and not arr
    # bbox centered + still far -> advance straight, no strafing
    vx, vy, arr = g.servo_cmd(320, 640, 0.8, 0.3)
    assert vx > 0 and abs(vy) < 1e-6 and not arr
    # bbox centered + close enough -> arrived
    vx, vy, arr = g.servo_cmd(320, 640, 0.25, 0.3)
    assert arr and vx == 0.0


def test_servo_clamps():
    vx, vy, _ = g.servo_cmd(640, 640, 5.0, 0.3, max_lat=0.15, max_lin=0.12)
    assert -0.15 <= vy <= 0.15 and 0.0 <= vx <= 0.12


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for fn in fns:
        fn(); print("  ok %s" % fn.__name__)
    print("\n%d/%d tests PASS" % (len(fns), len(fns)))
