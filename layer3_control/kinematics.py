"""
Layer 3: Kinematics - Joint angle / gripper coordinate computation
==================================================================
Placeholder: simulates inverse kinematics for the robot arm.
Must be tuned to the real hardware.
"""

import math
import logging

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [KINEMATICS] %(levelname)s: %(message)s",
    datefmt="%H:%M:%S"
)
logger = logging.getLogger("kinematics")

# ============================================================
# ROBOT ARM CONFIGURATION (tune to your hardware)
# ============================================================
# Arm link lengths (mm)
ARM_LINK_1 = 150.0  # Link 1 (shoulder -> elbow)
ARM_LINK_2 = 120.0  # Link 2 (elbow -> wrist)
ARM_LINK_3 = 80.0   # Link 3 (wrist -> gripper)

# Servo angle limits (degrees)
SERVO_MIN = 0
SERVO_MAX = 180

# Camera params (pixel -> mm conversion)
CAMERA_FOV_H = 60.0    # Horizontal camera field of view (degrees)
CAMERA_RES_W = 640      # Horizontal resolution
CAMERA_RES_H = 480      # Vertical resolution
CAMERA_HEIGHT = 300.0    # Camera height above the table (mm)


def pixel_to_world(x: int, y: int, w: int, h: int,
                   frame_w: int = CAMERA_RES_W,
                   frame_h: int = CAMERA_RES_H) -> dict:
    """
    Convert pixel coordinates (bbox) -> real-world coordinates (mm).
    Assumes the camera looks straight down (top-down view).

    Args:
        x, y, w, h: bounding box from YOLO/VLM (pixels)
        frame_w, frame_h: frame resolution

    Returns:
        dict with world_x, world_y, object_size_mm
    """
    # Object center (pixels)
    cx = x + w / 2.0
    cy = y + h / 2.0

    # Normalize to [-0.5, 0.5]
    norm_x = (cx / frame_w) - 0.5
    norm_y = (cy / frame_h) - 0.5

    # Compute the real viewing area size (mm) from FOV and height
    fov_rad = math.radians(CAMERA_FOV_H)
    view_width_mm = 2 * CAMERA_HEIGHT * math.tan(fov_rad / 2)
    view_height_mm = view_width_mm * (frame_h / frame_w)

    # Real coordinates (mm) - origin at the camera center
    world_x = norm_x * view_width_mm
    world_y = norm_y * view_height_mm

    # Object size (mm)
    object_w_mm = (w / frame_w) * view_width_mm
    object_h_mm = (h / frame_h) * view_height_mm

    result = {
        "world_x_mm": round(world_x, 1),
        "world_y_mm": round(world_y, 1),
        "object_width_mm": round(object_w_mm, 1),
        "object_height_mm": round(object_h_mm, 1),
        "distance_from_center_mm": round(math.sqrt(world_x**2 + world_y**2), 1)
    }

    logger.info(f"Pixel ({cx:.0f}, {cy:.0f}) -> World ({result['world_x_mm']}, {result['world_y_mm']}) mm")
    return result


def calculate_angles(x: int, y: int, w: int, h: int) -> dict:
    """
    Compute servo joint angles from the bbox coordinates.

    Args:
        x, y, w, h: bounding box [pixels]

    Returns:
        dict with servo angles (base, shoulder, elbow, wrist, gripper)
    """
    # Convert pixel -> world
    world = pixel_to_world(x, y, w, h)
    wx = world["world_x_mm"]
    wy = world["world_y_mm"]

    # === Inverse Kinematics (simple 2D) ===

    # Base angle (left/right rotation) - from the X position
    base_angle = 90 + math.degrees(math.atan2(wx, CAMERA_HEIGHT))
    base_angle = max(SERVO_MIN, min(SERVO_MAX, base_angle))

    # Horizontal distance to the object
    distance = math.sqrt(wx**2 + wy**2)

    # Shoulder & elbow angles (2-link IK formula)
    reach = min(distance, ARM_LINK_1 + ARM_LINK_2)  # clamp to arm reach

    # Cosine rule for a 2-link arm
    cos_elbow = (ARM_LINK_1**2 + ARM_LINK_2**2 - reach**2) / (2 * ARM_LINK_1 * ARM_LINK_2)
    cos_elbow = max(-1, min(1, cos_elbow))  # clamp
    elbow_angle = math.degrees(math.acos(cos_elbow))

    # Shoulder angle
    cos_shoulder = (ARM_LINK_1**2 + reach**2 - ARM_LINK_2**2) / (2 * ARM_LINK_1 * reach)
    cos_shoulder = max(-1, min(1, cos_shoulder))
    shoulder_angle = math.degrees(math.acos(cos_shoulder))

    # Wrist: keep the gripper vertical
    wrist_angle = 180 - shoulder_angle - elbow_angle + 90

    # Gripper: open based on the object size
    obj_size = max(world["object_width_mm"], world["object_height_mm"])
    gripper_angle = min(90, max(30, int(obj_size / 3)))  # 30-90 deg

    # Clamp everything
    angles = {
        "base": round(max(SERVO_MIN, min(SERVO_MAX, base_angle))),
        "shoulder": round(max(SERVO_MIN, min(SERVO_MAX, shoulder_angle))),
        "elbow": round(max(SERVO_MIN, min(SERVO_MAX, elbow_angle))),
        "wrist": round(max(SERVO_MIN, min(SERVO_MAX, wrist_angle))),
        "gripper": gripper_angle,
        "world_coords": world,
        "reachable": distance <= (ARM_LINK_1 + ARM_LINK_2)
    }

    logger.info(f"Servo angles: base={angles['base']} shoulder={angles['shoulder']} "
                f"elbow={angles['elbow']} wrist={angles['wrist']} gripper={angles['gripper']}")
    logger.info(f"Reachable: {angles['reachable']} (distance: {distance:.1f}mm)")

    return angles


if __name__ == "__main__":
    # Test with a mock coordinate
    print("=== Test Kinematics ===")
    test_bbox = [200, 150, 100, 80]  # x, y, w, h (pixels)
    print(f"Input bbox (pixel): {test_bbox}")
    result = calculate_angles(*test_bbox)
    print(f"Output angles: {result}")
