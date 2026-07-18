"""
Orbbec Astra camera module (0x2bc5:0x0401) via Orbbec OpenNI2 v2.3.0 (ctypes).
==============================================================================
The Astra is a legacy device - it ONLY works with Orbbec's OpenNI2 fork, NOT with
the system OpenNI2 (apt) nor pyorbbecsdk. The persistent driver stack lives in the
repo at tools/orbbec/openni2/.

HARDWARE LIMIT (verified on this machine):
    Over USB 2.0 the Astra CANNOT stream depth + color SIMULTANEOUSLY - starting a
    second stream while another is running fails/hangs. So this module supports 3 modes:
      - "depth" : depth only        -> read() -> (None, depth_mm)   ~30 FPS
      - "color" : color only        -> read() -> (bgr,  None)       ~30 FPS
      - "both"  : toggle stop/start -> read() -> (bgr,  depth_mm)   ~2  FPS (slow!)
    Object detection + 3D coords (Phase 0) only need DEPTH -> use mode="depth".
    YOLO (Phase 1) needs RGB -> mode="color" (or "both" when coords are also needed).

Usage:
    from layer1_vision.cameras.astra_openni import AstraCamera

    with AstraCamera(mode="depth") as cam:
        bgr, depth_mm = cam.read()      # depth_mm: HxW float32 (mm), 0.0 = invalid

Technical notes (AArch64, Orbbec OpenNI2 v2.3.0):
- ONI_API_VERSION = 2 (NOT 2002/2003).
- libOpenNI2.so looks for OpenNI2/Drivers/ RELATIVE to CWD -> chdir() while loading the lib.
- OniFrame layout: dataSize @offset 0 (uint32), pData @offset 8 (void*).
- The color sensor returns RGB888; this module returns BGR for cv2/ultralytics compatibility.
"""

import ctypes
import os
import struct

import numpy as np

# Persistent driver stack path
_REPO_ROOT   = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_REPO_ORBBEC = os.path.join(_REPO_ROOT, "tools", "orbbec", "openni2")
ORBBEC_DIR = _REPO_ORBBEC if os.path.exists(os.path.join(_REPO_ORBBEC, "libOpenNI2.so")) \
    else "/tmp/orbbec_openni2"
LIB_PATH = os.path.join(ORBBEC_DIR, "libOpenNI2.so")

# OpenNI2 constants
ONI_STATUS_OK    = 0
ONI_API_VERSION  = 2
ONI_SENSOR_DEPTH = 1
ONI_SENSOR_COLOR = 2

FRAME_OFF_DATASIZE = 0   # uint32
FRAME_OFF_PDATA    = 8   # void*

DEFAULT_WIDTH = 640
VALID_MODES = ("depth", "color", "both")


class AstraCamera:
    """Open the Astra and read frames. See the module docstring for USB 2.0 limits + modes."""

    def __init__(self, mode: str = "depth", width: int = DEFAULT_WIDTH):
        if mode not in VALID_MODES:
            raise ValueError(f"mode must be one of {VALID_MODES}, got '{mode}'")
        self.mode = mode
        self.width = width
        self._lib = None
        self._device = ctypes.c_void_p()
        self._depth_stream = ctypes.c_void_p()
        self._color_stream = ctypes.c_void_p()
        self._opened = False
        self._active_stream = None   # currently running stream (for grab(): toggle without reopening device)
        self._open()

    # Initialization
    def _load_lib(self):
        if not os.path.exists(LIB_PATH):
            raise FileNotFoundError(
                f"{LIB_PATH} not found.\n"
                f"  Restore the driver stack into tools/orbbec/openni2/."
            )
        prev = os.getcwd()
        os.chdir(ORBBEC_DIR)          # the lib looks for OpenNI2/Drivers/ relative to CWD
        try:
            lib = ctypes.CDLL(LIB_PATH)
        finally:
            os.chdir(prev)
        return lib

    def _create_stream(self, sensor_type):
        stream = ctypes.c_void_p()
        if self._lib.oniDeviceCreateStream(
            self._device, sensor_type, ctypes.byref(stream)
        ) != ONI_STATUS_OK:
            raise RuntimeError(f"Failed to create stream (sensor={sensor_type})")
        return stream

    def _open(self):
        self._lib = self._load_lib()

        if self._lib.oniInitialize(ONI_API_VERSION) != ONI_STATUS_OK:
            raise RuntimeError("oniInitialize failed")

        dev_list = ctypes.c_void_p()
        count = ctypes.c_int(0)
        self._lib.oniGetDeviceList(ctypes.byref(dev_list), ctypes.byref(count))
        if count.value == 0:
            self._lib.oniShutdown()
            raise RuntimeError("Astra not found (does lsusb show 2bc5:0401?)")

        rc = self._lib.oniDeviceOpen(None, ctypes.byref(self._device))
        self._lib.oniReleaseDeviceList(dev_list, count)
        if rc != ONI_STATUS_OK:
            self._lib.oniShutdown()
            raise RuntimeError(f"oniDeviceOpen failed: rc={rc}")

        # Create the needed streams. Only ever start ONE stream at a time
        # (USB 2.0 limit). For "both", read() toggles start/stop.
        if self.mode in ("depth", "both"):
            self._depth_stream = self._create_stream(ONI_SENSOR_DEPTH)
        if self.mode in ("color", "both"):
            self._color_stream = self._create_stream(ONI_SENSOR_COLOR)

        if self.mode == "depth":
            self._start(self._depth_stream)
        elif self.mode == "color":
            self._start(self._color_stream)
        # "both": do not pre-start - toggle inside read()

        self._opened = True

    # Checked start/stop helpers
    def _start(self, stream):
        if self._lib.oniStreamStart(stream) != ONI_STATUS_OK:
            raise RuntimeError("oniStreamStart failed (the other stream may be running - USB 2.0)")

    def _stop(self, stream):
        self._lib.oniStreamStop(stream)

    # Read one frame from a running stream
    def _read_stream(self, stream, sensor_type):
        frame_p = ctypes.c_void_p()
        rc = self._lib.oniStreamReadFrame(stream, ctypes.byref(frame_p))
        if rc != ONI_STATUS_OK or not frame_p.value:
            return None
        addr = frame_p.value
        raw = bytes((ctypes.c_uint8 * 16).from_address(addr))
        ds = struct.unpack_from("<I", raw, FRAME_OFF_DATASIZE)[0]
        pdata = struct.unpack_from("<Q", raw, FRAME_OFF_PDATA)[0]
        w = self.width
        if sensor_type == ONI_SENSOR_DEPTH:
            h = ds // 2 // w
            buf = (ctypes.c_uint16 * (w * h)).from_address(pdata)
            arr = np.frombuffer(buf, dtype=np.uint16).copy().reshape(h, w)
        else:
            h = ds // 3 // w
            buf = (ctypes.c_uint8 * ds).from_address(pdata)
            arr = np.frombuffer(buf, dtype=np.uint8).copy().reshape(h, w, 3)
        self._lib.oniFrameRelease(ctypes.c_void_p(frame_p.value))
        return arr

    def read(self):
        """
        Read one frame according to mode.
        Returns (bgr, depth_mm):
            bgr      : HxWx3 uint8 BGR  (None if mode="depth")
            depth_mm : HxW   float32 mm, 0.0 = invalid  (None if mode="color")
        Raises RuntimeError on read failure.
        """
        if not self._opened:
            raise RuntimeError("Camera not open / already closed")

        if self.mode == "depth":
            d = self._read_stream(self._depth_stream, ONI_SENSOR_DEPTH)
            if d is None:
                raise RuntimeError("Depth read failed")
            return None, d.astype(np.float32)

        if self.mode == "color":
            c = self._read_stream(self._color_stream, ONI_SENSOR_COLOR)
            if c is None:
                raise RuntimeError("Color read failed")
            return c[:, :, ::-1].copy(), None

        # mode == "both": toggle depth -> color (slow, ~2 FPS)
        self._start(self._depth_stream)
        d = self._read_stream(self._depth_stream, ONI_SENSOR_DEPTH)
        self._stop(self._depth_stream)

        self._start(self._color_stream)
        c = self._read_stream(self._color_stream, ONI_SENSOR_COLOR)
        self._stop(self._color_stream)

        if d is None or c is None:
            raise RuntimeError("Frame read failed (both mode)")
        return c[:, :, ::-1].copy(), d.astype(np.float32)

    def grab(self, want):
        """Read one 'depth' or 'color' frame, switching the running stream via stop/start -
        WITHOUT closing/reopening the device (avoids the oniShutdown/oniInitialize cycle that
        hangs the Astra over USB2). Only one stream runs at a time. Requires the camera opened
        in mode='both' (both streams already created). Staying on the same 'want' repeatedly
        means no toggle -> full frame rate (~7Hz depth)."""
        if not self._opened:
            raise RuntimeError("Camera not open / already closed")
        if want == "depth":
            target, sensor = self._depth_stream, ONI_SENSOR_DEPTH
        elif want == "color":
            target, sensor = self._color_stream, ONI_SENSOR_COLOR
        else:
            raise ValueError(f"want must be 'depth' or 'color', got '{want}'")
        if not target.value:
            raise RuntimeError(f"Stream '{want}' not created - open the camera in mode='both'.")
        if self._active_stream is not target:
            if self._active_stream is not None and self._active_stream.value:
                self._stop(self._active_stream)
            self._start(target)
            self._active_stream = target
        arr = self._read_stream(target, sensor)
        if arr is None:
            raise RuntimeError(f"{want} read failed")
        if want == "depth":
            return None, arr.astype(np.float32)
        return arr[:, :, ::-1].copy(), None

    # Cleanup
    def close(self):
        if self._lib is None:
            return
        try:
            for s_attr in ("_depth_stream", "_color_stream"):
                s = getattr(self, s_attr)
                if s.value:
                    self._lib.oniStreamStop(s)
                    self._lib.oniStreamDestroy(s)
                    setattr(self, s_attr, ctypes.c_void_p())
            if self._device.value:
                self._lib.oniDeviceClose(self._device)
                self._device = ctypes.c_void_p()
            self._lib.oniShutdown()
        finally:
            self._opened = False

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()


# Smoke test: 100 continuous depth frames + 5 "both" frames
if __name__ == "__main__":
    import sys
    import time

    n = int(sys.argv[1]) if len(sys.argv) > 1 else 100

    print(f">> mode=depth, {n} continuous frames", flush=True)
    cam = AstraCamera(mode="depth")
    try:
        t0 = time.time()
        depth = None
        for i in range(n):
            _, depth = cam.read()
        dt = time.time() - t0
        valid = depth[depth > 0]
        print(f"OK read {n}/{n} frames without crash | {n/dt:.1f} FPS", flush=True)
        print(f"   depth {depth.shape} center="
              f"{depth[depth.shape[0]//2, depth.shape[1]//2]:.0f}mm | "
              f"valid={valid.size}/{depth.size} | "
              f"range [{valid.min() if valid.size else 0:.0f}-{depth.max():.0f}]mm", flush=True)
    finally:
        cam.close()

    print(">> mode=both, 5 frames (toggle, slow)", flush=True)
    cam = AstraCamera(mode="both")
    try:
        for i in range(5):
            bgr, depth = cam.read()
        print(f"OK both: bgr={bgr.shape} depth={depth.shape}", flush=True)
    finally:
        cam.close()
        print("OK close() in finally", flush=True)
