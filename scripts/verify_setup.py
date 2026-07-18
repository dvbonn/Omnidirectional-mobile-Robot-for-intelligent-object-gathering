"""
verify_setup.py - Check the whole setup before running the system.

Run this before deploying to the Jetson or when hitting startup errors:
    python verify_setup.py

Exit code:
    0 = all OK (can run offline)
    1 = there are issues to fix
"""

import json
import platform
import shutil
import subprocess
import sys
from pathlib import Path

PROJECT_DIR = Path(__file__).parent.parent
IS_WINDOWS  = platform.system() == "Windows"

PASS = "  [OK]  "
FAIL = "  [ERR] "
WARN = "  [!]   "

errors   = []
warnings = []


def check(label: str, ok: bool, error_msg: str = "", warn_msg: str = ""):
    if ok:
        print(f"{PASS}{label}")
    elif warn_msg:
        print(f"{WARN}{label} - {warn_msg}")
        warnings.append(f"{label}: {warn_msg}")
    else:
        print(f"{FAIL}{label} - {error_msg}")
        errors.append(f"{label}: {error_msg}")


def section(title: str):
    print(f"\n{'-' * 55}")
    print(f"  {title}")
    print(f"{'-' * 55}")


# 1. Python packages
section("1. PYTHON PACKAGES")

required = {
    "cv2":              "opencv-python",
    "ultralytics":      "ultralytics",
    "requests":         "requests",
    "fastapi":          "fastapi",
    "uvicorn":          "uvicorn",
    "flask":            "flask",
    "huggingface_hub":  "huggingface-hub",
    "multipart":        "python-multipart",
}

for module, pkg in required.items():
    try:
        __import__(module)
        check(f"import {module}", True)
    except ImportError:
        check(f"import {module}", False, f"pip install {pkg}")


# 2. YOLO model
section("2. YOLO MODEL")

yolo_candidates = [
    PROJECT_DIR / "layer1_vision" / "model" / "yolov8n.pt",
    PROJECT_DIR / "yolov8n.pt",
]
yolo_found = next((p for p in yolo_candidates if p.exists()), None)

if yolo_found:
    size_mb = yolo_found.stat().st_size / (1024 * 1024)
    check(f"yolov8n.pt ({size_mb:.1f} MB) at {yolo_found.relative_to(PROJECT_DIR)}", True)
else:
    check(
        "yolov8n.pt",
        False,
        f"Not found. Place the file at: layer1_vision/model/yolov8n.pt\n"
        f"{'':12}Download: https://github.com/ultralytics/assets/releases"
    )


# 3. VLM model (Qwen2.5-VL GGUF)
section("3. VLM MODEL (Qwen2.5-VL)")

config_path = PROJECT_DIR / "layer2_brain" / "config.json"
if config_path.exists():
    cfg = json.loads(config_path.read_text(encoding="utf-8"))
    models_dir  = PROJECT_DIR / "layer2_brain" / "models"
    model_file  = models_dir / cfg["vlm"]["model_file"]
    mmproj_file = models_dir / cfg["vlm"]["mmproj_file"]

    if model_file.exists():
        size_gb = model_file.stat().st_size / (1024 ** 3)
        check(f"{model_file.name} ({size_gb:.2f} GB)", True)
    else:
        check(
            cfg["vlm"]["model_file"],
            False,
            f"Not downloaded. Run: python layer2_brain/setup_vlm.py"
        )

    if mmproj_file.exists():
        size_mb = mmproj_file.stat().st_size / (1024 * 1024)
        check(f"{mmproj_file.name} ({size_mb:.0f} MB)", True)
    else:
        check(
            cfg["vlm"]["mmproj_file"],
            False,
            f"Not downloaded. Run: python layer2_brain/setup_vlm.py"
        )
else:
    check("layer2_brain/config.json", False, "Config file does not exist!")


# 4. llama-server binary
section("4. LLAMA-SERVER BINARY")

llama_dir = PROJECT_DIR / "llama.cpp"
bin_candidates = [
    llama_dir / "llama-server",
    llama_dir / "llama-server.exe",
    llama_dir / "bin" / "llama-server",
    llama_dir / "bin" / "llama-server.exe",
    llama_dir / "build" / "bin" / "llama-server",
    llama_dir / "build" / "bin" / "Release" / "llama-server.exe",
]
llama_bin = next((p for p in bin_candidates if p.exists()), None)

if llama_bin:
    check(f"llama-server at {llama_bin.relative_to(PROJECT_DIR)}", True)
    # Check it runs
    try:
        r = subprocess.run([str(llama_bin), "--version"],
                           capture_output=True, text=True, timeout=5)
        version = (r.stdout + r.stderr).strip().splitlines()
        v = next((l for l in version if "version" in l.lower()), "OK")
        check(f"  version: {v[:60]}", True)
    except Exception as e:
        check("  llama-server --version", False,
              warn_msg=f"Could not run: {e}")
else:
    in_path = shutil.which("llama-server")
    if in_path:
        check(f"llama-server (PATH: {in_path})", True)
    else:
        check(
            "llama-server",
            False,
            "Binary not found.\n"
            f"{'':12}Download: https://github.com/ggml-org/llama.cpp/releases\n"
            f"{'':12}Extract into: llama.cpp/llama-server (Linux) or llama.cpp/llama-server.exe (Windows)"
        )


# 5. Disk space
section("5. DISK SPACE")

try:
    usage = shutil.disk_usage(str(PROJECT_DIR))
    free_gb = usage.free / (1024 ** 3)
    total_gb = usage.total / (1024 ** 3)
    ok_space = free_gb >= 2.0
    check(
        f"Disk free: {free_gb:.1f} GB / {total_gb:.1f} GB",
        ok_space,
        warn_msg="" if ok_space else "Warning: at least 2 GB free is recommended"
    )
except Exception:
    print(f"{WARN}Could not read disk usage")


# 6. Camera (optional)
section("6. CAMERA (optional)")

try:
    import cv2
    cap = cv2.VideoCapture(0)
    if cap.isOpened():
        check("Camera index 0", True)
        cap.release()
    else:
        check("Camera index 0", False,
              warn_msg="No camera found. Use --image <file> to test offline.")
except Exception:
    print(f"{WARN}Could not check the camera (opencv not installed?)")


# 7. Serial port (optional - robot hardware)
section("7. SERIAL PORT (optional - robot arm)")

try:
    import serial.tools.list_ports
    ports = list(serial.tools.list_ports.comports())
    if ports:
        for p in ports:
            check(f"Serial: {p.device} - {p.description[:40]}", True)
    else:
        check("Serial ports", True,
              warn_msg="No serial ports found. Normal if the robot is not plugged in.")
except ImportError:
    print(f"{WARN}pyserial not installed (pip install pyserial) - needed to connect the real robot")


# 8. Result
print(f"\n{'=' * 55}")
print(f"  CHECK RESULT")
print(f"{'=' * 55}")

if not errors and not warnings:
    print("  [OK] Everything is ready! Can run FULLY OFFLINE.")
    print("  Startup order:")
    launcher = "start_vlm_server.bat" if IS_WINDOWS else "./start_vlm_server.sh"
    print(f"    1. {launcher}")
    print(f"    2. python layer2_brain/brain_server.py")
    print(f"    3. python layer3_control/control_node.py")
    print(f"    4. python layer1_vision/vision_node.py")
elif not errors:
    print(f"  [OK] Can run (with {len(warnings)} warnings):")
    for w in warnings:
        print(f"    [!] {w}")
else:
    print(f"  [ERR] Fix {len(errors)} errors before running:")
    for e in errors:
        print(f"    [X] {e}")
    if warnings:
        print(f"\n  Warnings ({len(warnings)}):")
        for w in warnings:
            print(f"    [!] {w}")

print()
sys.exit(0 if not errors else 1)
