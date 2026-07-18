"""
Layer 2: Brain Server - VLM reasoning brain
===========================================
- Task 2.2: FastAPI server that receives images
- Task 2.3: Call Qwen2.5-VL via llama.cpp and return JSON

Quick test:
  python brain_server.py            # Start the server
  http://localhost:8000/docs        # Swagger UI (test /analyze in the browser)
  http://localhost:8000/health      # Health check
"""

import os
import json
import base64
import logging
import re
import time
import shutil
from pathlib import Path
from datetime import datetime

# Disable HuggingFace Hub network calls if the model already exists locally
_models_dir = Path(__file__).parent / "models"
if _models_dir.exists() and any(_models_dir.glob("*.gguf")):
    os.environ.setdefault("HF_HUB_OFFLINE", "1")
    os.environ.setdefault("TRANSFORMERS_OFFLINE", "1")

from fastapi.responses import JSONResponse, RedirectResponse

import requests as http_requests
import uvicorn
from fastapi import FastAPI, File, UploadFile, Form, HTTPException

# ============================================================
# CONFIGURATION
# ============================================================
CONFIG_PATH = Path(__file__).parent / "config.json"

with open(CONFIG_PATH, "r", encoding="utf-8") as f:
    CONFIG = json.load(f)

LLAMA_CPP_URL = (
    f"http://{CONFIG['llama_cpp_server']['host']}:{CONFIG['llama_cpp_server']['port']}"
)
UPLOAD_DIR = Path(__file__).parent / "uploads"
UPLOAD_DIR.mkdir(exist_ok=True)

# ============================================================
# LOGGING
# ============================================================
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [BRAIN] %(levelname)s: %(message)s",
    datefmt="%H:%M:%S"
)
logger = logging.getLogger("brain_server")

# ============================================================
# FASTAPI APP
# ============================================================
app = FastAPI(
    title="Robot Brain - VLM Server",
    description="VLM Brain for Robot Collecting. Get img -> Analyze -> Return matrix.",
    version="1.0.0"
)


@app.get("/", include_in_schema=False)
async def root():
    """Redirect to Swagger UI for quick testing."""
    return RedirectResponse(url="/docs")


@app.get("/health")
async def health_check():
    """Health check - verify the brain server and the llama.cpp server."""
    llama_status = "unknown"
    try:
        resp = http_requests.get(f"{LLAMA_CPP_URL}/health", timeout=3)
        if resp.status_code == 200:
            llama_status = "connected"
        else:
            llama_status = f"error (status {resp.status_code})"
    except http_requests.ConnectionError:
        llama_status = "disconnected"
    except Exception as e:
        llama_status = f"error: {str(e)}"

    return {
        "status": "ok",
        "service": "brain_server",
        "llama_cpp": llama_status,
        "model": CONFIG["vlm"]["model_name"],
        "timestamp": datetime.now().isoformat()
    }


@app.post("/analyze")
async def analyze_image(
    file: UploadFile = File(...),
    detections: str = Form(default="[]")
):
    """
    Receive an image from Layer 1 and send it to Qwen2.5-VL for analysis.

    - **file**: image file (JPEG/PNG)
    - **detections**: JSON string with the YOLO detection list (optional)

    Returns: JSON with object, collectible, bbox, confidence, reason
    """
    # Validate file
    if not file.content_type or not file.content_type.startswith("image/"):
        raise HTTPException(status_code=400, detail="File must be an image (image/jpeg, image/png)")

    # Save the image temporarily
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    ext = Path(file.filename).suffix if file.filename else ".jpg"
    save_path = UPLOAD_DIR / f"brain_{timestamp}{ext}"

    try:
        contents = await file.read()
        with open(save_path, "wb") as f:
            f.write(contents)
        logger.info(f"Received image: {file.filename} ({len(contents)} bytes)")
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error saving image: {str(e)}")

    # Parse YOLO detections (json.loads instead of eval - safer)
    try:
        yolo_detections = json.loads(detections) if detections else []
        if not isinstance(yolo_detections, list):
            yolo_detections = []
    except (json.JSONDecodeError, ValueError):
        yolo_detections = []

    # Call the VLM
    try:
        result = await call_vlm(save_path, yolo_detections)
        logger.info(f"VLM result: {json.dumps(result, ensure_ascii=False)}")
        return JSONResponse(content=result)
    except ConnectionError:
        logger.warning("llama.cpp server is not running, using mock response")
        mock = generate_mock_response(yolo_detections)
        return JSONResponse(content=mock)
    except Exception as e:
        logger.error(f"VLM error: {e}")
        raise HTTPException(status_code=500, detail=f"VLM analysis error: {str(e)}")
    finally:
        # Cleanup
        if save_path.exists():
            save_path.unlink()


async def call_vlm(image_path: Path, yolo_detections: list) -> dict:
    """
    Call Qwen2.5-VL via the llama.cpp server API.
    Uses the /completion endpoint with a base64 image.
    """
    # Encode the image as base64
    with open(image_path, "rb") as f:
        image_b64 = base64.b64encode(f.read()).decode("utf-8")

    # Build the prompt
    system_prompt = CONFIG["prompt"]["system"]
    user_prompt = CONFIG["prompt"]["user_template"]

    # Add YOLO info if present
    if yolo_detections:
        yolo_info = "\n\nInformation from YOLO (for reference):\n"
        for det in yolo_detections:
            if isinstance(det, dict):
                yolo_info += f"- {det.get('class_name', 'unknown')}: bbox={det.get('bbox', [])}, conf={det.get('confidence', 0)}\n"
        user_prompt += yolo_info

    # Call the llama.cpp chat completion API
    payload = {
        "messages": [
            {
                "role": "system",
                "content": system_prompt
            },
            {
                "role": "user",
                "content": [
                    {
                        "type": "image_url",
                        "image_url": {
                            "url": f"data:image/jpeg;base64,{image_b64}"
                        }
                    },
                    {
                        "type": "text",
                        "text": user_prompt
                    }
                ]
            }
        ],
        "temperature": CONFIG["vlm"]["temperature"],
        "max_tokens": CONFIG["vlm"]["max_tokens"],
        "stream": False
    }

    logger.info("Sending image to Qwen2.5-VL...")
    start_time = time.time()

    try:
        response = http_requests.post(
            f"{LLAMA_CPP_URL}/v1/chat/completions",
            json=payload,
            timeout=60
        )
    except http_requests.ConnectionError:
        raise ConnectionError("Cannot connect to llama.cpp server")

    elapsed = time.time() - start_time
    logger.info(f"VLM responded in {elapsed:.1f}s")

    if response.status_code != 200:
        logger.error(f"llama.cpp error: {response.status_code} - {response.text}")
        raise RuntimeError(f"llama.cpp error: {response.status_code}")

    # Parse response
    resp_json = response.json()
    raw_text = resp_json["choices"][0]["message"]["content"]
    logger.info(f"Raw VLM response: {raw_text}")

    # Extract JSON from the response text
    result = parse_vlm_response(raw_text)
    result["processing_time_s"] = round(elapsed, 2)
    result["raw_response"] = raw_text

    return result


def parse_vlm_response(text: str) -> dict:
    """
    Extract JSON from the VLM output.
    The VLM may return pure JSON or JSON embedded in text.
    """
    # Try to parse directly
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        pass

    # Find a JSON block inside the text
    json_patterns = [
        r'\{[^{}]*"object"[^{}]*\}',
        r'```json\s*(\{.*?\})\s*```',
        r'\{.*?\}',
    ]

    for pattern in json_patterns:
        matches = re.findall(pattern, text, re.DOTALL)
        for match in matches:
            try:
                parsed = json.loads(match)
                if "object" in parsed:
                    return parsed
            except json.JSONDecodeError:
                continue

    # Fallback: build a basic response from the text
    logger.warning("Could not parse JSON from the VLM, building a fallback response")
    return {
        "object": "unknown",
        "collectible": False,
        "bbox": [0, 0, 0, 0],
        "confidence": 0.0,
        "reason": f"Could not parse VLM response: {text[:200]}"
    }


def generate_mock_response(yolo_detections: list) -> dict:
    """
    Build a mock response when llama.cpp is not running.
    Uses info from the YOLO detections if available.
    """
    if yolo_detections and isinstance(yolo_detections, list) and len(yolo_detections) > 0:
        det = yolo_detections[0] if isinstance(yolo_detections[0], dict) else {}
        return {
            "object": det.get("class_name", "unknown_object"),
            "collectible": True,
            "bbox": det.get("bbox", [100, 100, 50, 50]),
            "confidence": det.get("confidence", 0.5),
            "reason": "[MOCK] llama.cpp not running - response based on YOLO detection",
            "mock": True
        }

    return {
        "object": "unknown_object",
        "collectible": False,
        "bbox": [0, 0, 0, 0],
        "confidence": 0.0,
        "reason": "[MOCK] llama.cpp not running - no detection data",
        "mock": True
    }


# ============================================================
# MAIN
# ============================================================
def main():
    host = CONFIG["brain_server"]["host"]
    port = CONFIG["brain_server"]["port"]

    logger.info("=" * 50)
    logger.info("ROBOT COLLECTING - BRAIN SERVER")
    logger.info("=" * 50)
    logger.info(f"Model      : {CONFIG['vlm']['model_name']}")
    logger.info(f"llama.cpp  : {LLAMA_CPP_URL}")
    logger.info(f"Brain API  : http://localhost:{port}")
    logger.info(f"Swagger UI : http://localhost:{port}/docs  <- test here")
    logger.info(f"Health     : http://localhost:{port}/health")
    logger.info("-" * 50)

    uvicorn.run(app, host=host, port=port, log_level="warning")


if __name__ == "__main__":
    main()
