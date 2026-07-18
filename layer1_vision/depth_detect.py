"""
Object detection using DEPTH (simple version, no YOLO needed).
==============================================================
Idea: segment the NEAREST blob within the working range.
  1. Keep valid depth within [z_min, z_max] mm.
  2. Find the nearest depth z_near; keep the band [z_near, z_near + band] to isolate
     the closest object and drop the background.
  3. Clean up with morphology, label connected components.
  4. Pick the largest component (area >= min_area) -> bbox + center + median_z.

Shares the bbox format with vision_node: (x, y, w, h) in pixels.
"""

import cv2
import numpy as np

from .depth_utils import bbox_depth


def detect_nearest_blob(
    depth_mm: np.ndarray,
    z_min: float = 150.0,
    z_max: float = 1500.0,
    band: float = 200.0,
    min_area: int = 800,
    morph_kernel: int = 5,
):
    """
    Return dict {bbox:(x,y,w,h), center:(u,v), median_z, area} for the nearest blob,
    or None if there is no object in range.
    """
    valid = (depth_mm > z_min) & (depth_mm < z_max) & np.isfinite(depth_mm)
    if not valid.any():
        return None

    z_near = float(depth_mm[valid].min())
    mask = (valid & (depth_mm <= z_near + band)).astype(np.uint8) * 255

    if morph_kernel > 0:
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (morph_kernel, morph_kernel))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)

    n, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if n <= 1:
        return None

    # drop label 0 (background), pick the largest-area component
    areas = stats[1:, cv2.CC_STAT_AREA]
    idx = int(np.argmax(areas)) + 1
    area = int(stats[idx, cv2.CC_STAT_AREA])
    if area < min_area:
        return None

    x = int(stats[idx, cv2.CC_STAT_LEFT])
    y = int(stats[idx, cv2.CC_STAT_TOP])
    w = int(stats[idx, cv2.CC_STAT_WIDTH])
    h = int(stats[idx, cv2.CC_STAT_HEIGHT])
    cu, cv_ = centroids[idx]
    bbox = (x, y, w, h)
    return {
        "bbox": bbox,
        "center": (float(cu), float(cv_)),
        "median_z": bbox_depth(depth_mm, bbox),
        "area": area,
    }


# Stability test over N frames
if __name__ == "__main__":
    import sys

    from .cameras.astra_openni import AstraCamera

    n = int(sys.argv[1]) if len(sys.argv) > 1 else 30
    cam = AstraCamera(mode="depth")
    try:
        hits = 0
        zs = []
        for i in range(n):
            _, depth = cam.read()
            det = detect_nearest_blob(depth)
            if det:
                hits += 1
                zs.append(det["median_z"])
                if i % 5 == 0:
                    print(f"  frame {i:3d}: bbox={det['bbox']} "
                          f"z={det['median_z']:.0f}mm area={det['area']}", flush=True)
        print(f"\ndetected in {hits}/{n} frames", flush=True)
        if zs:
            arr = np.array(zs)
            print(f"median_z: mean={arr.mean():.0f}mm std={arr.std():.1f}mm "
                  f"(small std = stable)", flush=True)
    finally:
        cam.close()
