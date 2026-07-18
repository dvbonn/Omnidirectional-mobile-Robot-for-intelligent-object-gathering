"""
Structured detection log (JSONL) - the Vision -> ROS2 bridge.
=============================================================
Each line = one JSON record. The schema is a SUPERSET of the Brain/VLM prompting
schema ({object, collectible, bbox, confidence, reason}) plus 3D coordinates and
ROS2 fields (stamp, frame_id). This means:

  - The Vision layer (this demo / vision_node) and the Brain layer (VLM) share the
    SAME schema -> easy to combine.
  - A future ROS2 node only needs to read/tail the JSONL file (or call make_record)
    and publish geometry_msgs/PointStamped - no need to parse a custom format.

Coordinate convention = REP-105 *optical frame*: X right, Y down, Z forward -
MATCHES layer1_vision.depth_utils.unproject (unit: mm). Therefore:
  - position_mm : original (mm) for humans / reports.
  - position_m  : position_mm / 1000 (meters) - the ROS2 standard unit (REP-103).
  - frame_id    : optical TF frame name, default "camera_depth_optical_frame".

How ROS2 would use this record (hint, not implemented here):
    Header h; h.stamp = Time(seconds=rec["stamp"]); h.frame_id = rec["frame_id"]
    PointStamped p; p.point = Point(**rec["position_m"])      # x,y,z meters
    # then tf2 transform camera_depth_optical_frame -> base_link (static transform).
"""
import json
import math
import os
import time
from datetime import datetime

# REP-105: optical frame of the depth sensor (x right, y down, z forward).
DEFAULT_FRAME_ID = "camera_depth_optical_frame"


def _num(v, ndigits=2):
    """NaN/inf/None -> None (strict JSON rejects NaN); numbers -> rounded."""
    if v is None:
        return None
    v = float(v)
    return round(v, ndigits) if math.isfinite(v) else None


def make_record(
    frame,
    bbox,
    center_px,
    coord_mm,
    *,
    object_name=None,
    collectible=None,
    confidence=1.0,
    reason=None,
    source="astra_depth_nearest",
    frame_id=DEFAULT_FRAME_ID,
    fps=None,
    stamp=None,
):
    """
    Build one standard detection record (dict, JSON-safe - no NaN left).

    Parameters follow the prompting schema:
      object_name / collectible / bbox / confidence / reason
        -> leave as None when the Vision layer has not decided yet (the Brain VLM fills them later).
      bbox      : (x, y, w, h) pixels - same convention as vision_node/depth_detect.
      center_px : (u, v) pixel center of the bbox.
      coord_mm  : (X, Y, Z) mm in the optical frame (or None when no object).
    """
    stamp = time.time() if stamp is None else float(stamp)
    if coord_mm is None:
        X = Y = Z = None
    else:
        X, Y, Z = coord_mm
    pos_mm = {"x": _num(X), "y": _num(Y), "z": _num(Z)}
    pos_m = {k: (None if v is None else round(v / 1000.0, 4)) for k, v in pos_mm.items()}

    rec = {
        # ROS2 / time
        "stamp": round(stamp, 3),                         # epoch seconds -> ROS Time
        "stamp_iso": datetime.fromtimestamp(stamp).isoformat(timespec="milliseconds"),
        "frame": int(frame),
        "frame_id": frame_id,                             # optical TF frame
        "source": source,
        # matches the Brain/VLM prompting schema
        "object": object_name,                            # None = not identified yet (VLM fills)
        "collectible": collectible,                       # None = not decided yet (VLM fills)
        "bbox": [int(v) for v in bbox] if bbox is not None else None,  # [x,y,w,h] px
        "confidence": _num(confidence),
        "reason": reason,
        # extensions for navigation / ROS2
        "center_px": ([int(round(center_px[0])), int(round(center_px[1]))]
                      if center_px is not None else None),
        "position_mm": pos_mm,                            # optical frame, mm
        "position_m": pos_m,                              # optical frame, meters (ROS2)
    }
    if fps is not None:
        rec["fps"] = _num(fps, 1)
    return rec


class DetectionLogger:
    """
    Write records to a JSONL file (append). Line-buffered -> ROS2/`tail -f` reads in realtime.

    Usage:
        with DetectionLogger("Log/astra_demo/detections.jsonl") as dlog:
            dlog.log(make_record(...))
    """

    def __init__(self, path):
        self.path = path
        d = os.path.dirname(os.path.abspath(path))
        os.makedirs(d, exist_ok=True)
        self._fh = open(path, "a", encoding="utf-8", buffering=1)  # buffering=1: line-buffered
        self.count = 0

    def log(self, record):
        # allow_nan=False: assume the record is already NaN-free; if a NaN slips in it
        # raises early instead of producing broken JSON that ROS2 cannot parse.
        self._fh.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + "\n")
        self.count += 1

    def close(self):
        if self._fh is not None and not self._fh.closed:
            self._fh.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


# Self-test: JSON-safe schema + mm->m mapping + NaN handling
if __name__ == "__main__":
    import tempfile

    # 1. Full record: mm -> m correct, re-parseable JSON.
    r = make_record(
        frame=7, bbox=(100, 120, 40, 50), center_px=(120.4, 145.6),
        coord_mm=(-150.0, 80.0, 600.0), confidence=1.0, fps=16.3,
    )
    s = json.dumps(r, allow_nan=False)            # raises if any NaN remains
    back = json.loads(s)
    assert back["bbox"] == [100, 120, 40, 50]
    assert back["position_mm"] == {"x": -150.0, "y": 80.0, "z": 600.0}
    assert back["position_m"] == {"x": -0.15, "y": 0.08, "z": 0.6}
    assert back["frame_id"] == DEFAULT_FRAME_ID
    assert back["object"] is None and back["collectible"] is None  # VLM fills later
    print("OK full record:", s)

    # 2. No object / invalid depth (NaN) -> position = None, still valid JSON.
    r2 = make_record(frame=8, bbox=None, center_px=None,
                     coord_mm=(float("nan"),) * 3)
    s2 = json.dumps(r2, allow_nan=False)
    assert json.loads(s2)["position_m"]["z"] is None
    assert json.loads(s2)["bbox"] is None
    print("OK empty record (NaN->None):", s2)

    # 3. DetectionLogger writes & reads back.
    with tempfile.NamedTemporaryFile("r", suffix=".jsonl", delete=False) as tf:
        p = tf.name
    with DetectionLogger(p) as dlog:
        dlog.log(r)
        dlog.log(r2)
        assert dlog.count == 2
    with open(p) as f:
        lines = [json.loads(ln) for ln in f if ln.strip()]
    assert len(lines) == 2 and lines[0]["frame"] == 7
    os.unlink(p)
    print("OK DetectionLogger wrote/read 2 JSONL lines")

    print("\n=== detection_log: all self-tests PASS ===")
