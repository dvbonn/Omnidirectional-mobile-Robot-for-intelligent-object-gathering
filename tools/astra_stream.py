#!/usr/bin/env python3
"""
Stream the Astra camera as MJPEG to a browser - view live over SSH (no monitor needed).
=======================================================================================
Runs a web server (stdlib, no extra install). One thread reads the camera + overlay
(bbox + X/Y/Z coordinates + FPS), clients view it via multipart MJPEG.

Run:
  python3 tools/astra_stream.py                 # port 8080
  python3 tools/astra_stream.py --port 8090 --mode both

View:
  - VS Code Remote auto-forwards the port -> open http://localhost:8080 on YOUR machine.
  - Or on the same LAN: http://<jetson-ip>:8080
"""
import argparse
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from layer1_vision.cameras.astra_openni import AstraCamera          # noqa: E402
from layer1_vision.depth_detect import detect_nearest_blob          # noqa: E402
from layer1_vision.depth_utils import Intrinsics, unproject         # noqa: E402
from tools.astra_coord_demo import annotate, colorize_depth         # noqa: E402


class FrameStore:
    """Holds the latest JPEG, thread-safe; wakes clients when a new frame arrives."""
    def __init__(self):
        self._jpeg = None
        self._cond = threading.Condition()
        self.running = True

    def update(self, jpeg_bytes):
        with self._cond:
            self._jpeg = jpeg_bytes
            self._cond.notify_all()

    def get(self, last_id):
        """Wait until a new JPEG (different from last_id) exists, return (jpeg, new_id)."""
        with self._cond:
            while self.running and self._jpeg is last_id:
                self._cond.wait(timeout=1.0)
            return self._jpeg, self._jpeg

    def stop(self):
        with self._cond:
            self.running = False
            self._cond.notify_all()


store = FrameStore()
LATEST = {"coord": None, "fps": 0.0}


def capture_loop(mode, z_max, jpeg_quality):
    """Thread that reads the camera continuously, draws the overlay, encodes JPEG into the store."""
    intr = Intrinsics.load()
    cam = AstraCamera(mode=mode)
    fps = 0.0
    t_prev = time.time()
    enc = [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality]
    try:
        while store.running:
            bgr, depth = cam.read()
            # depth/both modes both have depth; if color-only, show the color image
            if depth is not None:
                det = detect_nearest_blob(depth)
                coord = (float("nan"),) * 3
                if det is not None:
                    cu, cv_ = det["center"]
                    coord = unproject(cu, cv_, det["median_z"], intr)
                base = colorize_depth(depth, z_max)
            else:
                det, coord, base = None, (float("nan"),) * 3, bgr

            now = time.time()
            inst = 1.0 / max(now - t_prev, 1e-6)
            fps = 0.9 * fps + 0.1 * inst if fps else inst
            t_prev = now
            LATEST["coord"], LATEST["fps"] = coord, fps

            canvas = annotate(base, det, coord, fps)
            ok, buf = cv2.imencode(".jpg", canvas, enc)
            if ok:
                store.update(buf.tobytes())
    finally:
        cam.close()
        store.stop()


INDEX_HTML = b"""<!doctype html><html><head><meta charset=utf-8>
<title>Astra live</title><style>
body{background:#111;color:#eee;font-family:sans-serif;text-align:center;margin:0;padding:16px}
img{max-width:100%;border:2px solid #2a2}h3{font-weight:400}</style></head>
<body><h3>Astra - depth + coordinate 3D (live)</h3>
<img src="/stream.mjpg"><p>Close the tab to stop the stream.</p>
</body></html>"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass  # silence per-request logging

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.send_header("Content-Length", str(len(INDEX_HTML)))
            self.end_headers()
            self.wfile.write(INDEX_HTML)
            return

        if self.path == "/stream.mjpg":
            self.send_response(200)
            self.send_header("Age", "0")
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=FRAME")
            self.end_headers()
            last = None
            try:
                while store.running:
                    jpeg, last = store.get(last)
                    if jpeg is None:
                        continue
                    self.wfile.write(b"--FRAME\r\n")
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", str(len(jpeg)))
                    self.end_headers()
                    self.wfile.write(jpeg)
                    self.wfile.write(b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                pass  # client closed the tab
            return

        self.send_error(404)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--mode", default="depth", choices=["depth", "color", "both"])
    ap.add_argument("--z-max", type=float, default=2000.0)
    ap.add_argument("--quality", type=int, default=80, help="JPEG quality 1-100")
    args = ap.parse_args()

    t = threading.Thread(target=capture_loop,
                         args=(args.mode, args.z_max, args.quality), daemon=True)
    t.start()

    server = ThreadingHTTPServer((args.host, args.port), Handler)
    print(f"Stream Astra (mode={args.mode}) at:", flush=True)
    print(f"  - VS Code Remote: open http://localhost:{args.port}  (port forwarded)", flush=True)
    print(f"  - Same LAN:       http://<jetson-ip>:{args.port}", flush=True)
    print("Ctrl-C to stop.", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping...", flush=True)
    finally:
        store.stop()
        server.shutdown()
        t.join(timeout=3)
        print("Stopped.", flush=True)


if __name__ == "__main__":
    main()
