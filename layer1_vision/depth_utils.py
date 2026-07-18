"""
Depth utilities: unproject pixel -> 3D camera coordinates, read depth inside a bbox.
====================================================================================
Camera coordinate frame: X right, Y down, Z forward.
Pinhole model:
    Z = depth_mm
    X = (u - cx) * Z / fx
    Y = (v - cy) * Z / fy

Intrinsics are loaded from config/astra_intrinsics.json (Astra factory default - the
SDK does not expose FOV). Can be replaced by a hand-calibrated set.
"""

import json
import os
from dataclasses import dataclass

import numpy as np

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_INTRINSICS_PATH = os.path.join(_REPO_ROOT, "config", "astra_intrinsics.json")


@dataclass
class Intrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int = 640
    height: int = 480

    @classmethod
    def load(cls, path: str = DEFAULT_INTRINSICS_PATH) -> "Intrinsics":
        with open(path, "r") as f:
            d = json.load(f)
        return cls(
            fx=float(d["fx"]), fy=float(d["fy"]),
            cx=float(d["cx"]), cy=float(d["cy"]),
            width=int(d.get("width", 640)), height=int(d.get("height", 480)),
        )


def unproject(u: float, v: float, depth_mm: float, intr: Intrinsics):
    """
    Pixel (u,v) + depth (mm) -> (X, Y, Z) mm in the camera frame.
    Returns (nan, nan, nan) if depth is invalid (<= 0).
    """
    if depth_mm is None or depth_mm <= 0:
        return float("nan"), float("nan"), float("nan")
    z = float(depth_mm)
    x = (u - intr.cx) * z / intr.fx
    y = (v - intr.cy) * z / intr.fy
    return x, y, z


def bbox_depth(depth_mm: np.ndarray, bbox, min_valid: int = 10) -> float:
    """
    Representative depth inside a bbox = median of valid pixels (> 0).
    bbox = (x, y, w, h) in pixels (same as vision_node).
    Returns 0.0 if there are not enough valid pixels (fully invalid region).
    """
    x, y, w, h = (int(v) for v in bbox)
    H, W = depth_mm.shape[:2]
    x0, y0 = max(0, x), max(0, y)
    x1, y1 = min(W, x + w), min(H, y + h)
    if x1 <= x0 or y1 <= y0:
        return 0.0
    roi = depth_mm[y0:y1, x0:x1]
    valid = roi[(roi > 0) & np.isfinite(roi)]
    if valid.size < min_valid:
        return 0.0
    return float(np.median(valid))


def bbox_center_coord(depth_mm: np.ndarray, bbox, intr: Intrinsics):
    """
    Convenience wrapper: bbox center + median depth -> (X,Y,Z) mm.
    Returns (nan,nan,nan) if the bbox has no valid depth.
    """
    x, y, w, h = bbox
    u = x + w / 2.0
    v = y + h / 2.0
    z = bbox_depth(depth_mm, bbox)
    return unproject(u, v, z, intr)


# Unit test for the pinhole formulas
if __name__ == "__main__":
    intr = Intrinsics(fx=570.3422, fy=570.3422, cx=319.5, cy=239.5)

    # 1. Point at the optical center -> X=Y=0, Z=depth
    x, y, z = unproject(intr.cx, intr.cy, 1000.0, intr)
    assert abs(x) < 1e-6 and abs(y) < 1e-6 and abs(z - 1000.0) < 1e-6, (x, y, z)
    print(f"OK optical center: ({x:.3f},{y:.3f},{z:.1f}) mm")

    # 2. Offset of fx pixels to the right @ Z=fx mm -> X = 1mm (since (u-cx)*Z/fx = 1*fx/fx)
    x, y, z = unproject(intr.cx + 1, intr.cy, intr.fx, intr)
    assert abs(x - 1.0) < 1e-6, x
    print(f"OK horizontal offset: X={x:.4f} mm (expected 1.0)")

    # 3. Point below center (v > cy) -> positive Y (Y down)
    x, y, z = unproject(intr.cx, intr.cy + 100, 1000.0, intr)
    assert y > 0, y
    print(f"OK Y-down positive: Y={y:.2f} mm")

    # 4. invalid depth -> nan
    x, y, z = unproject(100, 100, 0, intr)
    assert all(np.isnan(v) for v in (x, y, z))
    print("OK invalid depth -> nan")

    # 5. bbox_depth median on a synthetic patch
    depth = np.zeros((480, 640), dtype=np.float32)
    depth[200:240, 300:340] = 500.0
    depth[210, 310] = 0.0  # a few invalid pixels interleaved
    z = bbox_depth(depth, (300, 200, 40, 40))
    assert abs(z - 500.0) < 1e-6, z
    print(f"OK bbox_depth median = {z:.1f} mm (ignoring 0 pixels)")

    # 6. fully invalid bbox -> 0.0
    z = bbox_depth(np.zeros((480, 640), np.float32), (0, 0, 50, 50))
    assert z == 0.0
    print("OK fully invalid bbox -> 0.0")

    print("\n=== depth_utils: all unit tests PASS ===")
