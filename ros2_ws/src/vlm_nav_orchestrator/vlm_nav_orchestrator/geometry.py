"""
geometry.py - Object localization + goal math (PURE PYTHON, no ROS -> unit-testable).
=====================================================================================
Used by the T10 orchestrator (detect -> drive to): 2D bbox + depth -> 3D camera point ->
(tf) -> pose on the map -> NAV2 goal (with standoff); and the centering servo command in the final phase.
Optical frame convention (REP-103): X right, Y down, Z forward.
"""
import math

import numpy as np


def bbox_center(bbox):
    """bbox [x, y, w, h] (pixels) -> (u, v) center."""
    x, y, w, h = bbox
    return (x + w / 2.0, y + h / 2.0)


def pixel_to_camera_point(u, v, z_m, fx, fy, cx, cy):
    """
    Pixel (u,v) + depth z (m) + intrinsics -> 3D point in the camera optical frame (m).
    X = (u-cx)*z/fx ; Y = (v-cy)*z/fy ; Z = z.
    """
    return ((u - cx) * z_m / fx, (v - cy) * z_m / fy, float(z_m))


def transform_point(p, T):
    """Transform point (x,y,z) through the 4x4 homogeneous matrix T (e.g. camera->map from tf2)."""
    v = np.array([p[0], p[1], p[2], 1.0])
    o = np.asarray(T) @ v
    return (float(o[0]), float(o[1]), float(o[2]))


def standoff_goal(robot_xy, object_xy, standoff):
    """
    NAV2 goal: a point 'standoff' m from the OBJECT toward the robot (do not hit the object),
    with the heading (yaw) facing the object. Returns (gx, gy, yaw).
    """
    rx, ry = robot_xy
    ox, oy = object_xy
    dx, dy = ox - rx, oy - ry
    dist = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    if dist < 1e-6 or dist <= standoff:
        # already within the standoff range -> goal = the robot position, just face the object
        return (rx, ry, yaw)
    ux, uy = dx / dist, dy / dist
    return (ox - ux * standoff, oy - uy * standoff, yaw)


def yaw_to_quat(yaw):
    """Quaternion (x,y,z,w) for a yaw about Z (for NavigateToPose)."""
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def servo_cmd(center_u, image_width, depth_z_m, target_z_m,
              k_lat=0.4, k_lin=0.4, max_lat=0.15, max_lin=0.12, center_tol=0.06):
    """
    APPROACH phase (HOLONOMIC servo - mecanum base): center the bbox by STRAFING (v_y) rather
    than turning, while advancing toward target_z (v_x). The robot keeps its heading and moves
    diagonally straight to the object (requirement: if the object is seen, drive to it, no turning).
    Returns (v_x, v_y, arrived). depth_z_m=None -> only strafe, do not advance yet.
    arrived=True once centered & close enough.
    """
    err = (center_u - image_width / 2.0) / (image_width / 2.0)   # -1..1 (>0: object off to the right)
    centered = abs(err) < center_tol
    # Object off to the right (err>0) -> strafe RIGHT -> v_y NEGATIVE (REP-103: +y is to the left).
    # Deadzone once centered -> no left/right jitter around the center.
    v_y = 0.0 if centered else max(-max_lat, min(max_lat, -k_lat * err))

    v_x = 0.0
    arrived = False
    if depth_z_m is not None:
        if depth_z_m > target_z_m:
            # Advance straight (v_x) in parallel with strafing (v_y) -> diagonal move to the object, no turn.
            v_x = max(0.0, min(max_lin, k_lin * (depth_z_m - target_z_m)))
        if centered and depth_z_m <= target_z_m:
            arrived = True
    return (v_x, v_y, arrived)
