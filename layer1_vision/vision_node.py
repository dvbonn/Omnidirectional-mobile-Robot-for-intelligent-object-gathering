"""
Layer 1: Vision Node - Fast perception & trigger signalling
===========================================================
- Task 1.1: Initialize camera (OpenCV webcam or Orbbec Astra)
- Task 1.2: Ultra-light YOLO (yolov8n) + 3D coordinates via Astra
- Task 1.3: Trigger logic (stable for 2s -> capture image)
- Task 1.4: Send data over the API

Run modes (CLI):
  python vision_node.py                               # Webcam + Brain API
  python vision_node.py --mock-brain                  # Webcam + skip Brain API
  python vision_node.py --source astra                # Astra color + YOLO (~30 FPS)
  python vision_node.py --source astra --astra-3d     # Astra both + YOLO + 3D coords (~2 FPS)
  python vision_node.py --image test.jpg --mock-brain # Offline still-image test
"""
from __future__ import annotations

import argparse
import json
import logging
import os
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import requests
from ultralytics import YOLO

# Astra camera + depth utils (optional - only used with --source astra)
try:
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
    from layer1_vision.cameras.astra_openni import AstraCamera
    from layer1_vision.depth_utils import Intrinsics, bbox_center_coord
    _ASTRA_AVAILABLE = True
    _ASTRA_IMPORT_ERR = None
except Exception as _e:
    _ASTRA_AVAILABLE = False
    _ASTRA_IMPORT_ERR = _e
    AstraCamera = None  # type: ignore

# Configuration
BRAIN_API_URL    = "http://localhost:8000/analyze"
CONTROL_API_URL  = "http://localhost:8001/execute"
TEMP_IMAGES_DIR  = Path(__file__).parent / "temp_images"
MODEL_DIR        = Path(__file__).parent / "model"
PROJECT_ROOT_DIR = Path(__file__).parent.parent
HEADLESS_OUT_DIR = PROJECT_ROOT_DIR / "Log" / "vision_node"

STABLE_DURATION      = 2.0   # seconds an object must stay stable before trigger
CAPTURE_COOLDOWN     = 5.0   # cooldown seconds after each capture
CONFIDENCE_THRESHOLD = 0.45  # YOLO confidence threshold
CAMERA_INDEX         = 0     # 0 = default webcam

# YOLO class IDs to detect (COCO dataset)
TARGET_CLASSES = {
    32: "sports ball",  39: "bottle",    40: "wine glass", 41: "cup",
    42: "fork",         43: "knife",     44: "spoon",      45: "bowl",
    46: "banana",       47: "apple",     48: "sandwich",   49: "orange",
    56: "chair",        60: "dining table", 62: "tv",      63: "laptop",
    64: "mouse",        65: "remote",    66: "keyboard",   67: "cell phone",
    68: "microwave",    69: "oven",      73: "book",       74: "clock",
    75: "vase",         76: "scissors",  77: "teddy bear", 28: "suitcase",
}

# Logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [VISION] %(levelname)s: %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger("vision_node")


# Task 1.1: Camera
def init_camera(camera_index: int = CAMERA_INDEX) -> cv2.VideoCapture:
    """Open the webcam and return a VideoCapture object."""
    logger.info(f"Opening camera index={camera_index}...")
    cap = cv2.VideoCapture(camera_index)

    if not cap.isOpened():
        logger.error("Cannot open camera! Check the webcam connection.")
        raise RuntimeError("Camera not available")

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    logger.info(f"Camera opened successfully: {w}x{h}")
    return cap


def make_dummy_frame(text: str = "NO CAMERA - DUMMY FRAME") -> np.ndarray:
    """Create a 640x480 gray dummy frame when no camera is present."""
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    frame[:] = (50, 50, 50)
    cv2.putText(frame, text, (80, 240),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (200, 200, 200), 2)
    return frame


# Task 1.2: YOLO
def load_yolo_model(model_name: str = "yolov8n.pt") -> YOLO:
    """Load YOLOv8 Nano locally. Never auto-downloads when offline."""
    # Search order: model/ -> project root
    candidates = [
        MODEL_DIR / model_name,
        PROJECT_ROOT_DIR / model_name,
    ]
    for model_path in candidates:
        if model_path.exists():
            logger.info(f"Loading YOLO from: {model_path}")
            # Copy into model/ if missing, so future loads are faster
            target = MODEL_DIR / model_name
            if not target.exists():
                MODEL_DIR.mkdir(parents=True, exist_ok=True)
                import shutil
                shutil.copy2(str(model_path), str(target))
                logger.info(f"Copied model into: {target}")
            return YOLO(str(model_path))

    raise FileNotFoundError(
        f"YOLO model '{model_name}' not found.\n"
        f"  Searched: {[str(c) for c in candidates]}\n"
        f"  Fix: place '{model_name}' into '{MODEL_DIR}' or '{PROJECT_ROOT_DIR}'\n"
        f"  Download (needs internet once): https://github.com/ultralytics/assets/releases"
    )


def detect_objects(model: YOLO, frame: np.ndarray) -> list[dict]:
    """
    Run YOLO on a frame, keeping only target classes.
    Returns: list of {class_id, class_name, confidence, bbox: [x,y,w,h]}
    """
    results = model(frame, verbose=False, conf=CONFIDENCE_THRESHOLD)
    detections = []
    for result in results:
        if result.boxes is None:
            continue
        for box in result.boxes:
            cls_id = int(box.cls[0])
            if cls_id not in TARGET_CLASSES:
                continue
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            detections.append({
                "class_id":   cls_id,
                "class_name": TARGET_CLASSES[cls_id],
                "confidence": round(float(box.conf[0]), 3),
                "bbox":       [int(x1), int(y1), int(x2 - x1), int(y2 - y1)],
            })
    return detections


def draw_detections(frame: np.ndarray, detections: list[dict]) -> np.ndarray:
    """Draw bounding boxes, labels, and 3D coordinates (if any) on the frame."""
    for det in detections:
        x, y, w, h = det["bbox"]
        label = f'{det["class_name"]} {det["confidence"]:.2f}'
        color = (0, 255, 0)
        cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(frame, (x, y - th - 8), (x + tw + 4, y), color, -1)
        cv2.putText(frame, label, (x + 2, y - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        coord = det.get("coord_3d")
        if coord and not any(v != v for v in coord):  # skip if NaN present
            cx, cy_c, cz = coord
            coord_txt = f"({cx:+.0f},{cy_c:+.0f},{cz:.0f})mm"
            cv2.putText(frame, coord_txt, (x + 2, y + h + 14),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 200, 255), 1)
    return frame


def enrich_with_depth(detections: list[dict], depth_mm: np.ndarray,
                      intr: Intrinsics) -> list[dict]:
    """Add a coord_3d field [X,Y,Z] (mm) to each detection from depth_mm."""
    for det in detections:
        x3, y3, z3 = bbox_center_coord(depth_mm, det["bbox"], intr)
        det["coord_3d"] = [round(x3, 1), round(y3, 1), round(z3, 1)]
    return detections


# Task 1.3: Stable Trigger
class StableTrigger:
    """
    Fires when an object is present continuously for >= stable_duration seconds.
    Has a cooldown to avoid repeated triggering.
    """

    def __init__(self, stable_duration: float = STABLE_DURATION,
                 cooldown: float = CAPTURE_COOLDOWN):
        self.stable_duration  = stable_duration
        self.cooldown         = cooldown
        self.first_seen_time  = None
        self.last_capture_time = 0.0
        self.triggered        = False

    def update(self, has_detections: bool) -> bool:
        """Returns True if a capture should happen."""
        now = time.time()
        if not has_detections:
            self.first_seen_time = None
            self.triggered = False
            return False

        if self.first_seen_time is None:
            self.first_seen_time = now
            return False

        elapsed = now - self.first_seen_time
        if elapsed >= self.stable_duration and not self.triggered:
            if now - self.last_capture_time >= self.cooldown:
                self.triggered = True
                self.last_capture_time = now
                logger.info(f"TRIGGER! Object stable for {elapsed:.1f}s -> capturing!")
                return True
        return False

    def reset(self):
        self.first_seen_time = None
        self.triggered = False


def save_frame(frame: np.ndarray, detections: list[dict]) -> str:
    """Save the frame into temp_images/. Returns: file path."""
    TEMP_IMAGES_DIR.mkdir(parents=True, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    path = TEMP_IMAGES_DIR / f"capture_{ts}.jpg"
    cv2.imwrite(str(path), frame)
    logger.info(f"Saved image: {path}")
    logger.info(f"   {len(detections)} objects: {[d['class_name'] for d in detections]}")
    return str(path)


# Task 1.4: Send over the API
def send_to_brain(image_path: str, detections: list[dict],
                  mock: bool = False) -> dict | None:
    """
    Send the image to the Brain API (Layer 2).
    If mock=True: print a simulated result to the terminal, no real POST.
    """
    if mock:
        logger.info("[MOCK] Skipping Brain API - printing mock result")
        result = {
            "object":      detections[0]["class_name"] if detections else "unknown",
            "collectible": True,
            "bbox":        detections[0]["bbox"] if detections else [0, 0, 0, 0],
            "confidence":  detections[0]["confidence"] if detections else 0.0,
            "reason":      "[MOCK] --mock-brain flag is on",
            "mock":        True,
        }
        logger.info(f"Mock result: {json.dumps(result, ensure_ascii=False)}")
        return result

    logger.info(f"Sending image to Brain API: {BRAIN_API_URL}")
    try:
        with open(image_path, "rb") as img_file:
            files = {"file": (os.path.basename(image_path), img_file, "image/jpeg")}
            data  = {"detections": json.dumps(detections)}   # use json.dumps instead of eval
            resp  = requests.post(BRAIN_API_URL, files=files, data=data, timeout=30)

        if resp.status_code == 200:
            result = resp.json()
            logger.info(f"Brain response: {result}")
            return result
        else:
            logger.error(f"Brain API error: {resp.status_code} - {resp.text}")
            return None

    except requests.ConnectionError:
        logger.warning("Cannot reach Brain API. Is the server running?")
        return None
    except requests.Timeout:
        logger.warning("Brain API timeout (>30s)")
        return None
    except Exception as e:
        logger.error(f"Error sending image: {e}")
        return None


def forward_to_control(brain_response: dict) -> dict | None:
    """Forward the command to the Control Node if the object is collectible."""
    if not brain_response or not brain_response.get("collectible", False):
        logger.info("Object not collectible, skipping Control.")
        return None

    logger.info(f"Sending command to Control: {CONTROL_API_URL}")
    try:
        resp = requests.post(CONTROL_API_URL, json=brain_response, timeout=10)
        if resp.status_code == 200:
            result = resp.json()
            logger.info(f"Control response: {result}")
            return result
        else:
            logger.error(f"Control API error: {resp.status_code}")
            return None
    except requests.ConnectionError:
        logger.warning("Cannot reach Control API.")
        return None
    except Exception as e:
        logger.error(f"Error sending control command: {e}")
        return None


# Still-image test mode
def run_single_image(image_path: str, model: YOLO, mock_brain: bool):
    """Run the pipeline once on a still image, then exit."""
    logger.info(f"Running on still image: {image_path}")
    frame = cv2.imread(image_path)
    if frame is None:
        logger.error(f"Failed to read image: {image_path}")
        sys.exit(1)

    detections = detect_objects(model, frame)
    logger.info(f"YOLO found {len(detections)} objects: "
                f"{[d['class_name'] for d in detections]}")

    # Save and send
    saved_path = save_frame(frame, detections)
    brain_result = send_to_brain(saved_path, detections, mock=mock_brain)

    if brain_result:
        if not brain_result.get("mock"):
            forward_to_control(brain_result)
    else:
        logger.warning("No result from Brain (server not running?). "
                       "Use --mock-brain to test offline.")

    # Show the image with bounding boxes
    display = draw_detections(frame.copy(), detections)
    cv2.imshow("Vision Test - Press any key to close", display)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    logger.info("Still-image test complete.")


# Main Loop
def main():
    parser = argparse.ArgumentParser(
        description="Robot Vision Node",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python vision_node.py                               # Webcam + Brain API
  python vision_node.py --mock-brain                  # Webcam + skip Brain API
  python vision_node.py --source astra                # Astra color, ~30 FPS
  python vision_node.py --source astra --astra-3d     # Astra both, YOLO + 3D coords
  python vision_node.py --image photo.jpg --mock-brain # Offline test
""",
    )
    parser.add_argument("--image",      metavar="PATH",
                        help="Still-image path. Run once and exit.")
    parser.add_argument("--mock-brain", action="store_true",
                        help="Skip Brain API, print mock result to terminal.")
    parser.add_argument("--source",     choices=["webcam", "astra"], default="webcam",
                        help="Camera source: 'webcam' (default) or 'astra'.")
    parser.add_argument("--astra-3d",   action="store_true",
                        help="(--source astra) Enable depth to attach 3D coords (~2 FPS).")
    parser.add_argument("--headless",   action="store_true",
                        help="Force headless: print coords + save frames (auto-detected via $DISPLAY).")
    parser.add_argument("--frames",     type=int, default=300,
                        help="(headless) number of frames before exit.")
    parser.add_argument("--trigger-log", metavar="PATH", default=None,
                        help="F14: write StableTrigger timeline (CSV) for plotting.")
    args = parser.parse_args()
    # Auto-detect headless when there is no X (headless SSH/Jetson)
    args.headless = args.headless or not os.environ.get("DISPLAY")

    logger.info("=" * 50)
    logger.info("ROBOT COLLECTING - VISION NODE")
    logger.info("=" * 50)
    if args.mock_brain:
        logger.info("Mode: --mock-brain (skipping Brain API)")

    # Task 1.2: Load YOLO
    model = load_yolo_model()

    # Still-image mode
    if args.image:
        run_single_image(args.image, model, mock_brain=args.mock_brain)
        return

    # Select camera source
    if args.source == "astra":
        if not _ASTRA_AVAILABLE:
            logger.error(f"AstraCamera not available: {_ASTRA_IMPORT_ERR!r}")
            logger.error("Check the driver at tools/orbbec/openni2/")
            sys.exit(1)
        astra_mode = "both" if args.astra_3d else "color"
        logger.info(f"Using Astra ({astra_mode} mode).")
        if astra_mode == "both":
            logger.info("  both mode: YOLO + 3D coords, ~2 FPS due to USB 2.0 limit")
        intr = Intrinsics.load() if args.astra_3d else None
        _run_astra_loop(model, astra_mode, intr, args)
    else:
        _run_webcam_loop(model, args)


def _run_webcam_loop(model: YOLO, args):
    cap = init_camera()
    trigger = StableTrigger()
    tlog = TriggerLogger(args.trigger_log, trigger) if args.trigger_log else None
    _log_ready("webcam", args)
    fps, t_prev, idx = 0.0, time.time(), 0
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                logger.error("Failed to read frame!")
                break
            detections = detect_objects(model, frame)
            display_frame = draw_detections(frame.copy(), detections)
            _run_trigger(frame, detections, trigger, args, tlog)
            _draw_hud(display_frame, detections, trigger)
            fps, t_prev = _smooth_fps(fps, t_prev)
            if not args.headless:
                cv2.putText(display_frame, f"FPS:{fps:.0f}", (550, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
            if _show_or_save(display_frame, detections, args.headless, idx, fps):
                break
            idx += 1
            if args.headless and idx >= args.frames:
                break
    except KeyboardInterrupt:
        logger.info("Ctrl+C")
    finally:
        cap.release()
        cv2.destroyAllWindows()
        if tlog:
            tlog.close()
        _log_done("webcam", idx, args)


def _run_astra_loop(model: YOLO, mode: str, intr: Intrinsics | None, args):
    """Main loop using Astra. mode='color' (30 FPS) or 'both' (2 FPS)."""
    cam = AstraCamera(mode=mode)
    trigger = StableTrigger()
    tlog = TriggerLogger(args.trigger_log, trigger) if args.trigger_log else None
    _log_ready("Astra", args)
    fps, t_prev, idx = 0.0, time.time(), 0
    try:
        while True:
            bgr, depth_mm = cam.read()
            if bgr is None:
                logger.warning("No color frame (depth-only mode?)")
                continue
            detections = detect_objects(model, bgr)
            if depth_mm is not None and intr is not None:
                detections = enrich_with_depth(detections, depth_mm, intr)
            display_frame = draw_detections(bgr.copy(), detections)
            _run_trigger(bgr, detections, trigger, args, tlog)
            _draw_hud(display_frame, detections, trigger)
            fps, t_prev = _smooth_fps(fps, t_prev)
            if not args.headless:
                mode_lbl = "3D" if depth_mm is not None else "RGB"
                cv2.putText(display_frame, f"{mode_lbl} {fps:.1f}FPS", (460, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1)
            if _show_or_save(display_frame, detections, args.headless, idx, fps):
                break
            idx += 1
            if args.headless and idx >= args.frames:
                break
    except KeyboardInterrupt:
        logger.info("Ctrl+C")
    finally:
        cam.close()
        cv2.destroyAllWindows()
        if tlog:
            tlog.close()
        _log_done("Astra", idx, args)


def _smooth_fps(fps: float, t_prev: float):
    """Per-frame EMA FPS; returns (new_fps, t_now)."""
    now = time.time()
    inst = 1.0 / max(now - t_prev, 1e-6)
    return (0.9 * fps + 0.1 * inst if fps else inst), now


def _log_ready(src: str, args):
    if args.headless:
        logger.info(f"Ready ({src}) - HEADLESS: print coords + save frames to {HEADLESS_OUT_DIR}/")
        logger.info(f"   running {args.frames} frames then exit (Ctrl+C to stop early)")
    else:
        logger.info(f"Ready ({src}). Press 'q' to quit.")


def _log_done(src: str, idx: int, args):
    msg = f"Vision Node ({src}) stopped after {idx} frames."
    if args.headless:
        msg += f" Frames saved to {HEADLESS_OUT_DIR}/"
    logger.info(msg)


# F14: log StableTrigger timeline for plotting
class TriggerLogger:
    """Write one CSV row per frame: t, has_det, top_conf, elapsed_s, state, trigger_event.
    state in IDLE / DETECTING / TRIGGERED / COOLDOWN - enough to build figure F14."""

    HEADER = "t,has_det,top_conf,elapsed_s,state,trigger_event\n"

    def __init__(self, path: str, trigger: StableTrigger):
        self.trigger = trigger
        self.f = open(path, "w")
        self.f.write(self.HEADER)
        self.t0 = time.time()
        logger.info(f"F14: writing trigger timeline -> {path}")

    def log(self, detections: list[dict], should_capture: bool):
        tr = self.trigger
        now = time.time()
        has = len(detections) > 0
        conf = max((d["confidence"] for d in detections), default=0.0)
        elapsed = (now - tr.first_seen_time) if tr.first_seen_time else 0.0
        in_cool = tr.last_capture_time > 0 and (now - tr.last_capture_time) < tr.cooldown
        if should_capture:
            state = "TRIGGERED"
        elif has and elapsed >= tr.stable_duration and in_cool:
            state = "COOLDOWN"        # object still stable but waiting out cooldown
        elif has:
            state = "DETECTING"
        elif in_cool:
            state = "COOLDOWN"
        else:
            state = "IDLE"
        self.f.write(f"{now - self.t0:.3f},{int(has)},{conf:.3f},"
                     f"{elapsed:.3f},{state},{int(should_capture)}\n")
        self.f.flush()

    def close(self):
        try:
            self.f.close()
        except Exception:
            pass


def _run_trigger(frame: np.ndarray, detections: list[dict],
                 trigger: StableTrigger, args, tlog: "TriggerLogger | None" = None):
    should_capture = trigger.update(len(detections) > 0)
    if tlog is not None:                     # log BEFORE reset to keep elapsed at the trigger frame
        tlog.log(detections, should_capture)
    if should_capture:
        image_path = save_frame(frame, detections)
        brain_result = send_to_brain(image_path, detections, mock=args.mock_brain)
        if brain_result and not args.mock_brain:
            forward_to_control(brain_result)
        trigger.reset()


def _show_or_save(display_frame: np.ndarray, detections: list[dict],
                  headless: bool, frame_idx: int, fps: float) -> bool:
    """Display (GUI) or print coords + save frame (headless). Returns True if we should exit."""
    if headless:
        if detections:
            for d in detections:
                c = d.get("coord_3d")
                coord_txt = (f"X={c[0]:+7.0f} Y={c[1]:+7.0f} Z={c[2]:6.0f} mm"
                             if c and not any(v != v for v in c) else "Z=  n/a")
                logger.info(f"frame {frame_idx:4d} | {d['class_name']:>12} "
                            f"{d['confidence']:.2f} | {coord_txt} | {fps:4.1f} FPS")
        elif frame_idx % 15 == 0:
            logger.info(f"frame {frame_idx:4d} | (no object)                 | {fps:4.1f} FPS")
        if frame_idx % 30 == 0:
            HEADLESS_OUT_DIR.mkdir(parents=True, exist_ok=True)
            p = HEADLESS_OUT_DIR / f"frame_{frame_idx:04d}.jpg"
            cv2.imwrite(str(p), display_frame)
        return False
    cv2.imshow("Robot Vision - Q to quit", display_frame)
    return (cv2.waitKey(1) & 0xFF) == ord("q")


def _draw_hud(display_frame: np.ndarray, detections: list[dict],
              trigger: StableTrigger):
    if trigger.first_seen_time and not trigger.triggered:
        elapsed = time.time() - trigger.first_seen_time
        progress = min(elapsed / STABLE_DURATION, 1.0)
        bar_w = int(200 * progress)
        cv2.rectangle(display_frame, (10, 10), (210, 30), (50, 50, 50), -1)
        cv2.rectangle(display_frame, (10, 10), (10 + bar_w, 30), (0, 200, 255), -1)
        cv2.putText(display_frame, f"Detecting... {elapsed:.1f}s/{STABLE_DURATION}s",
                    (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)
    else:
        cv2.putText(display_frame, f"Objects: {len(detections)}", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)


if __name__ == "__main__":
    main()
