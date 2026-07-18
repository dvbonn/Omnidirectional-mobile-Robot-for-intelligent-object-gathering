"""
test_grab_switch.py - Verify mode switching WITHOUT reopening the device (Astra USB2 anti-hang).
================================================================================================
Reproduces exactly what camera_manager does: open the device once in 'both', then TOGGLE
depth<->color for several cycles via grab() (only stop/start the stream, NO close/reopen).
Before the fix, each mode switch called oniShutdown()->oniInitialize() -> hang/corruption.
After the fix, it must survive many cycles.

Run (only needs the Astra plugged in over USB, NO ROS, NO base; stop any launch first):
    python3 layer1_vision/cameras/test_grab_switch.py
Expected: prints "ALL ... cycles OK" and both depth/color have valid sizes.
If it hangs on a cycle -> grab()/USB still has an issue, send the output back.
"""
import time

from astra_openni import AstraCamera

CYCLES = 6          # number of depth<->color cycles (the real camera_manager switches every few seconds)
FRAMES_PER = 5      # frames read per window


def main():
    print(">> Open device once in mode='both' (create 2 streams, not started yet)...", flush=True)
    cam = AstraCamera(mode="both")
    try:
        for c in range(CYCLES):
            # DEPTH window (like the exploration phase) - stay on depth for several frames, no toggle
            t0 = time.time()
            depth = None
            for _ in range(FRAMES_PER):
                _, depth = cam.grab("depth")
            fps_d = FRAMES_PER / (time.time() - t0)
            vd = depth[depth > 0]

            # COLOR window (like the approach phase) - switch to color (one stop/start)
            t0 = time.time()
            bgr = None
            for _ in range(FRAMES_PER):
                bgr, _ = cam.grab("color")
            fps_c = FRAMES_PER / (time.time() - t0)

            print(
                f"  cycle {c+1}/{CYCLES}: depth {depth.shape} {fps_d:.1f}FPS "
                f"valid={vd.size} | color {bgr.shape} {fps_c:.1f}FPS", flush=True)
        print(f"ALL {CYCLES} depth<->color cycles OK - mode switching does NOT hang/corrupt.", flush=True)
    finally:
        cam.close()
        print("close() clean (device closed, projector off).", flush=True)


if __name__ == "__main__":
    main()
