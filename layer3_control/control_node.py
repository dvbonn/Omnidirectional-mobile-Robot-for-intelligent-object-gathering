"""
Layer 3: Control Node - Hardware control
=========================================
Receives commands from the Brain (Layer 2), computes kinematics,
and sends commands down to the control board (Arduino/ESP32).
Placeholder: simulated via logs, not yet wired to real hardware.
"""

import json
import logging
import time
from flask import Flask, request, jsonify
from kinematics import calculate_angles

# ============================================================
# LOGGING
# ============================================================
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [CONTROL] %(levelname)s: %(message)s",
    datefmt="%H:%M:%S"
)
logger = logging.getLogger("control_node")

# ============================================================
# FLASK SERVER
# ============================================================
app = Flask(__name__)

# Robot state
robot_state = {
    "status": "idle",       # idle, moving, gripping, error
    "last_command": None,
    "last_angles": None,
    "task_count": 0
}


@app.route("/health", methods=["GET"])
def health():
    """Health check endpoint."""
    return jsonify({
        "status": "ok",
        "service": "control_node",
        "robot_state": robot_state["status"],
        "tasks_completed": robot_state["task_count"]
    })


@app.route("/execute", methods=["POST"])
def execute():
    """
    Receive a command from the Brain, compute kinematics, send to the board.

    Expected JSON input:
    {
        "object": "bottle",
        "collectible": true,
        "bbox": [x, y, w, h],
        "confidence": 0.85
    }
    """
    try:
        data = request.get_json()
        if not data:
            return jsonify({"error": "No JSON data received"}), 400

        logger.info(f"Received command: {json.dumps(data, ensure_ascii=False)}")
        robot_state["status"] = "processing"
        robot_state["last_command"] = data

        # Check for a bbox
        bbox = data.get("bbox")
        if not bbox or len(bbox) != 4:
            return jsonify({"error": "Missing or invalid bbox [x, y, w, h]"}), 400

        # Check whether it is collectible
        if not data.get("collectible", False):
            logger.info("Object not collectible, skipping.")
            robot_state["status"] = "idle"
            return jsonify({
                "status": "skipped",
                "reason": "Object not collectible"
            })

        # Compute kinematics
        x, y, w, h = bbox
        angles = calculate_angles(x, y, w, h)
        robot_state["last_angles"] = angles

        if not angles.get("reachable", False):
            logger.warning("Object is out of reach!")
            robot_state["status"] = "idle"
            return jsonify({
                "status": "out_of_reach",
                "angles": angles,
                "message": "Object is out of reach"
            })

        # === SIMULATE HARDWARE CONTROL ===
        result = simulate_hardware_control(data["object"], angles)

        robot_state["status"] = "idle"
        robot_state["task_count"] += 1

        return jsonify(result)

    except Exception as e:
        logger.error(f"Command handling error: {e}")
        robot_state["status"] = "error"
        return jsonify({"error": str(e)}), 500


@app.route("/status", methods=["GET"])
def status():
    """Return the robot's current state."""
    return jsonify(robot_state)


@app.route("/reset", methods=["POST"])
def reset():
    """Reset the robot to its initial state."""
    robot_state["status"] = "idle"
    robot_state["last_command"] = None
    robot_state["last_angles"] = None
    logger.info("Robot reset.")
    return jsonify({"status": "reset", "message": "Robot reset to home position"})


# ============================================================
# PLACEHOLDER: HARDWARE SIMULATION
# ============================================================
def simulate_hardware_control(object_name: str, angles: dict) -> dict:
    """
    Simulate the hardware control steps.
    In practice this is where serial commands are sent down to the Arduino/ESP32.

    Sequence:
    1. Move the base into position
    2. Lower the arm (shoulder + elbow)
    3. Open the gripper
    4. Grip the object
    5. Lift it up
    6. Move to the drop zone
    7. Release the object
    8. Return to the home position
    """
    steps = [
        ("[1] Rotate base",        f"base -> {angles['base']} deg", 0.5),
        ("[2] Lower arm",          f"shoulder -> {angles['shoulder']} deg, elbow -> {angles['elbow']} deg", 0.8),
        ("[3] Open gripper",       f"gripper -> {angles['gripper']} deg", 0.3),
        ("[4] Grip object",        "gripper -> 10 deg (closed)", 0.5),
        ("[5] Lift up",            "shoulder -> 45 deg", 0.6),
        ("[6] Rotate to drop zone","base -> 0 deg", 0.5),
        ("[7] Release object",     "gripper -> 90 deg (open)", 0.3),
        ("[8] Return home",        "all servos -> home", 0.5),
    ]

    logger.info(f"\n{'=' * 40}")
    logger.info(f"START COLLECTING: {object_name}")
    logger.info(f"{'=' * 40}")

    for step_name, detail, delay in steps:
        robot_state["status"] = "moving"
        logger.info(f"  {step_name}: {detail}")
        time.sleep(delay)  # simulate motion time

    logger.info(f"Finished collecting: {object_name}")
    logger.info(f"{'=' * 40}\n")

    return {
        "status": "completed",
        "object": object_name,
        "angles_used": angles,
        "message": f"Successfully collected {object_name}"
    }


# ============================================================
# Notes on connecting real hardware
# ============================================================
"""
# To connect a real Arduino/ESP32, replace simulate_hardware_control with:

import serial

ser = serial.Serial('COM3', 9600)  # change COM port accordingly

def send_to_board(angles: dict):
    command = f"B{angles['base']}S{angles['shoulder']}E{angles['elbow']}"
    command += f"W{angles['wrist']}G{angles['gripper']}\\n"
    ser.write(command.encode())
    response = ser.readline().decode().strip()
    return response
"""


def main():
    logger.info("=" * 50)
    logger.info("ROBOT COLLECTING - CONTROL NODE")
    logger.info("=" * 50)
    logger.info("Server running at http://localhost:8001")
    logger.info("Endpoints:")
    logger.info("  POST /execute  - Receive a collect command")
    logger.info("  GET  /health   - Health check")
    logger.info("  GET  /status   - Robot state")
    logger.info("  POST /reset    - Reset the robot")

    app.run(host="0.0.0.0", port=8001, debug=False)


if __name__ == "__main__":
    main()
