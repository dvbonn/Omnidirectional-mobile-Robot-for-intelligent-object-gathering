#!/usr/bin/env python3
"""
Test Orbbec Astra camera — Orbbec OpenNI2 v2.3.0 via ctypes.
Driver stack: /tmp/orbbec_openni2/libOpenNI2.so + OpenNI2/Drivers/liborbbec.so
"""
import ctypes, os, sys, struct
import numpy as np

# Persistent driver stack lives in the repo (tools/orbbec/openni2/).
# Fallback to the old /tmp location if the repo copy is missing.
_REPO_ORBBEC = os.path.join(os.path.dirname(os.path.abspath(__file__)), "orbbec", "openni2")
ORBBEC_DIR  = _REPO_ORBBEC if os.path.exists(os.path.join(_REPO_ORBBEC, "libOpenNI2.so")) else "/tmp/orbbec_openni2"
LIB_PATH    = os.path.join(ORBBEC_DIR, "libOpenNI2.so")
ONI_STATUS_OK = 0
ONI_SENSOR_DEPTH = 1
ONI_SENSOR_COLOR = 2

# OniFrame memory layout (Orbbec OpenNI2 v2.3.0, AArch64):
#   offset  0: uint32 dataSize
#   offset  4: uint32 _pad
#   offset  8: void*  pData        ← depth/color pixel buffer
#   offset 16: int    frameIndex
#   ...
FRAME_OFF_DATASIZE = 0
FRAME_OFF_PDATA    = 8


def _load_lib():
    prev = os.getcwd()
    os.chdir(ORBBEC_DIR)           # lib looks for OpenNI2/Drivers/ relative to CWD
    lib = ctypes.CDLL(LIB_PATH)
    os.chdir(prev)
    return lib


def _read_frame(lib, stream, sensor_type):
    """Read one frame; return (width, height, numpy_array) or None."""
    frame_p = ctypes.c_void_p()
    rc = lib.oniStreamReadFrame(stream, ctypes.byref(frame_p))
    if rc != ONI_STATUS_OK or not frame_p.value:
        return None
    addr    = frame_p.value
    raw     = bytes((ctypes.c_uint8 * 16).from_address(addr))
    ds      = struct.unpack_from("<I", raw, FRAME_OFF_DATASIZE)[0]
    pdata   = struct.unpack_from("<Q", raw, FRAME_OFF_PDATA)[0]
    if sensor_type == ONI_SENSOR_DEPTH:
        w, h = 640, ds // 2 // 640
        buf  = (ctypes.c_uint16 * (w * h)).from_address(pdata)
        arr  = np.frombuffer(buf, dtype=np.uint16).copy().reshape(h, w)
    else:
        w, h = 640, ds // 3 // 640
        buf  = (ctypes.c_uint8 * ds).from_address(pdata)
        arr  = np.frombuffer(buf, dtype=np.uint8).copy().reshape(h, w, 3)
    lib.oniFrameRelease(ctypes.c_void_p(frame_p.value))
    return w, h, arr


def test_astra():
    if not os.path.exists(LIB_PATH):
        print(f"FAIL: {LIB_PATH} not found")
        print("  Run setup: ensure /tmp/orbbec_openni2/ has libOpenNI2.so + OpenNI2/Drivers/liborbbec.so")
        return False

    lib = _load_lib()
    rc  = lib.oniInitialize(2)
    if rc != ONI_STATUS_OK:
        print(f"FAIL oniInitialize: rc={rc}")
        return False
    print("OK  oniInitialize — Orbbec OpenNI2 v2.3.0")

    # Enumerate
    dev_list = ctypes.c_void_p()
    count    = ctypes.c_int(0)
    lib.oniGetDeviceList(ctypes.byref(dev_list), ctypes.byref(count))
    print(f"OK  Devices found: {count.value}")
    if count.value == 0:
        print("FAIL: no devices")
        lib.oniShutdown()
        return False

    # Open default device
    device = ctypes.c_void_p()
    rc = lib.oniDeviceOpen(None, ctypes.byref(device))
    lib.oniReleaseDeviceList(dev_list, count)
    if rc != ONI_STATUS_OK:
        print(f"FAIL oniDeviceOpen: rc={rc}")
        lib.oniShutdown()
        return False
    print("OK  Device opened (Orbbec Astra 0x2bc5:0x0401)")

    ok = True

    # --- Depth ---
    stream = ctypes.c_void_p()
    if lib.oniDeviceCreateStream(device, ONI_SENSOR_DEPTH, ctypes.byref(stream)) == ONI_STATUS_OK:
        lib.oniStreamStart(stream)
        result = _read_frame(lib, stream, ONI_SENSOR_DEPTH)
        if result:
            w, h, arr = result
            valid = arr[arr > 0]
            print(f"OK  Depth  {w}x{h} | center={arr[h//2,w//2]}mm "
                  f"| valid={len(valid)}/{w*h} pixels "
                  f"| range [{valid.min() if len(valid) else 0}–{arr.max()}]mm")
        else:
            print("WARN Depth frame read failed")
            ok = False
        lib.oniStreamStop(stream)
        lib.oniStreamDestroy(stream)
    else:
        print("WARN Depth stream unavailable")
        ok = False

    # --- Color ---
    stream2 = ctypes.c_void_p()
    if lib.oniDeviceCreateStream(device, ONI_SENSOR_COLOR, ctypes.byref(stream2)) == ONI_STATUS_OK:
        lib.oniStreamStart(stream2)
        result2 = _read_frame(lib, stream2, ONI_SENSOR_COLOR)
        if result2:
            w, h, arr = result2
            print(f"OK  Color  {w}x{h} | mean RGB=({arr[:,:,0].mean():.0f},"
                  f"{arr[:,:,1].mean():.0f},{arr[:,:,2].mean():.0f})")
        else:
            print("WARN Color frame read failed")
        lib.oniStreamStop(stream2)
        lib.oniStreamDestroy(stream2)
    else:
        print("WARN Color stream unavailable")

    lib.oniDeviceClose(device)
    lib.oniShutdown()

    if ok:
        print("\n=== Astra camera setup SUCCESSFUL ===")
    return ok


if __name__ == "__main__":
    sys.exit(0 if test_astra() else 1)
