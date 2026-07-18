#!/usr/bin/env python3
"""
probe_vram.py — Do bo nho GPU (VRAM) llama-server cap phat cho tung n_gpu_layers.

Jetson dung UNIFIED MEMORY (khong co VRAM roi). "VRAM" o day = phan bo nho GPU
(cudaMalloc) cho model offload + KV cache + compute buffer, do qua delta cua
torch.cuda.mem_get_info() (device-global free) trong CUNG mot tien trinh probe
=> context CUDA cua probe khong doi nen tu trie tieu.

    python3 scripts/probe_vram.py --image <anh.jpg> --out docs/_vram_probe.jsonl --ngls -1 32 24 16 8 0
"""
import argparse, json, signal, time
from pathlib import Path
import requests
import torch

import importlib.util
_spec = importlib.util.spec_from_file_location("bvc", str(Path(__file__).with_name("bench_vlm_config.py")))
bvc = importlib.util.module_from_spec(_spec); _spec.loader.exec_module(bvc)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--image", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--ngls", nargs="+", type=int, default=[-1, 32, 24, 16, 8, 0])
    args = ap.parse_args()

    import base64
    img_b64 = base64.b64encode(Path(args.image).read_bytes()).decode()
    payload = bvc.build_payload(img_b64)

    torch.cuda.init()
    _ = torch.zeros(1, device="cuda")          # ep tao context probe
    torch.cuda.synchronize()
    free0, total = torch.cuda.mem_get_info()    # baseline: total - probe_ctx
    print(f"GPU total={total/2**20:.0f}MiB | free baseline (probe ctx loaded)={free0/2**20:.0f}MiB", flush=True)

    open(args.out, "w").close()
    for ngl in args.ngls:
        lp = bvc.PROJECT / "Log" / f"vramprobe_ngl{ngl}.log"
        proc, logf = bvc.start_server(ngl, lp)
        rec = {"ngl": ngl, "label": "all(-1)" if ngl < 0 else str(ngl)}
        try:
            if not bvc.wait_health():
                rec["error"] = "health timeout"
            else:
                # 1 inference de cap phat KV + compute buffer
                try:
                    requests.post(f"{bvc.URL}/v1/chat/completions", json=payload, timeout=300)
                except Exception as e:
                    rec["infer_warn"] = str(e)
                torch.cuda.synchronize()
                free1, _ = torch.cuda.mem_get_info()
                vram = (free0 - free1) / 2**20
                rec["vram_used_mib"] = round(vram, 1)
                rec["gpu_free_after_mib"] = round(free1/2**20, 1)
                print(f"ngl={rec['label']:>7}: VRAM cap phat = {vram:.1f} MiB", flush=True)
        finally:
            proc.send_signal(signal.SIGINT)
            try: proc.wait(timeout=15)
            except Exception: proc.kill()
            logf.close()
            time.sleep(3)  # cho GPU mem giai phong
            torch.cuda.empty_cache()
        with open(args.out, "a", encoding="utf-8") as f:
            f.write(json.dumps(rec, ensure_ascii=False) + "\n")

    print("PROBE DONE", flush=True)

if __name__ == "__main__":
    main()
