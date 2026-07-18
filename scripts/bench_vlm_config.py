#!/usr/bin/env python3
"""
bench_vlm_config.py — Benchmark Qwen2.5-VL-3B Q4_K_M cho MOT cau hinh n_gpu_layers.

Goi truc tiep llama-server (/v1/chat/completions, base64) — KHONG qua brain_server,
KHONG ghi anh ra disk (eMMC chat). Tu khoi dong + tat llama-server.

Dung:
    python3 scripts/bench_vlm_config.py --ngl 16 --runs 3 --image <anh.jpg> \
        --out docs/_vlm_sweep_raw.jsonl

Ghi 1 dong JSON ket qua vao --out (append). In JSON ket qua ra stdout.
Thermal guard: neu CPU/GPU > --temp-limit (mac dinh 80C) -> bo qua, danh dau aborted.
"""
import argparse, base64, json, os, re, signal, subprocess, sys, threading, time
from pathlib import Path

import requests

PROJECT = Path(__file__).resolve().parent.parent
SERVER  = PROJECT / "llama.cpp/build/bin/llama-server"
MODEL   = PROJECT / "layer2_brain/models/Qwen2.5-VL-3B-Instruct-Q4_K_M.gguf"
MMPROJ  = PROJECT / "layer2_brain/models/mmproj-Qwen2.5-VL-3B-Instruct-f16.gguf"
CONFIG  = json.loads((PROJECT / "layer2_brain/config.json").read_text(encoding="utf-8"))
PORT    = int(os.environ.get("PORT", "8080"))
URL     = f"http://127.0.0.1:{PORT}"

THERM = {  # zone-type -> sysfs temp file
    p.name: p / "temp"
    for p in Path("/sys/class/thermal").glob("thermal_zone*")
}

def read_temps():
    out = {}
    for zdir in Path("/sys/class/thermal").glob("thermal_zone*"):
        try:
            t = (zdir / "type").read_text().strip()
            v = int((zdir / "temp").read_text().strip()) / 1000.0
            out[t] = round(v, 1)
        except Exception:
            pass
    return out

def hot(temps, limit):
    for k, v in temps.items():
        if ("CPU" in k or "GPU" in k) and v >= limit:
            return k, v
    return None

def mem_snapshot():
    """System used MB (MemTotal-MemAvailable) tu /proc/meminfo."""
    info = {}
    for line in Path("/proc/meminfo").read_text().splitlines():
        k, _, rest = line.partition(":")
        info[k] = int(rest.strip().split()[0])  # kB
    used_kb = info["MemTotal"] - info.get("MemAvailable", info["MemFree"])
    return used_kb / 1024.0

def proc_rss_mb(pid):
    try:
        for line in Path(f"/proc/{pid}/status").read_text().splitlines():
            if line.startswith("VmRSS:"):
                return int(line.split()[1]) / 1024.0
    except Exception:
        pass
    return 0.0

def start_server(ngl, logpath):
    env = dict(os.environ)
    env["LD_LIBRARY_PATH"] = "/usr/local/cuda/lib64:" + env.get("LD_LIBRARY_PATH", "")
    cmd = [
        str(SERVER), "--model", str(MODEL), "--mmproj", str(MMPROJ),
        "--host", "127.0.0.1", "--port", str(PORT),
        "--ctx-size", str(CONFIG["vlm"]["context_size"]),
        "--n-gpu-layers", str(ngl),
        "--threads", str(CONFIG["vlm"]["threads"]),
    ]
    logf = open(logpath, "wb")
    proc = subprocess.Popen(cmd, stdout=logf, stderr=subprocess.STDOUT, env=env)
    return proc, logf

def wait_health(timeout=240):
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            r = requests.get(f"{URL}/health", timeout=3)
            if r.status_code == 200 and r.json().get("status") in ("ok", "loading model") :
                if r.json().get("status") == "ok":
                    return True
        except Exception:
            pass
        time.sleep(1.5)
    return False

def parse_gpu_mem(logpath):
    """Sum cac 'CUDA0 ... buffer size = X MiB' -> VRAM MiB. Layer offloaded."""
    txt = Path(logpath).read_text(errors="ignore")
    cuda_buffers = [float(x) for x in re.findall(r"CUDA0[^\n]*?buffer size\s*=\s*([\d.]+)\s*MiB", txt)]
    vram = round(sum(cuda_buffers), 1)
    m = re.search(r"offloaded\s+(\d+)\s*/\s*(\d+)\s+layers? to GPU", txt)
    offloaded = m.group(0) if m else None
    n_off = int(m.group(1)) if m else None
    n_tot = int(m.group(2)) if m else None
    return {"vram_mib": vram, "cuda_buffers_mib": cuda_buffers,
            "layers_offloaded": offloaded, "n_offloaded": n_off, "n_layers_total": n_tot}

def build_payload(img_b64):
    return {
        "messages": [
            {"role": "system", "content": CONFIG["prompt"]["system"]},
            {"role": "user", "content": [
                {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{img_b64}"}},
                {"type": "text", "text": CONFIG["prompt"]["user_template"]},
            ]},
        ],
        "temperature": CONFIG["vlm"]["temperature"],
        "max_tokens": CONFIG["vlm"]["max_tokens"],
        "stream": False,
    }

def json_ok(text):
    try:
        d = json.loads(text); return "object" in d
    except Exception:
        pass
    for pat in [r'\{[^{}]*"object"[^{}]*\}', r'```json\s*(\{.*?\})\s*```', r'\{.*?\}']:
        for mm in re.findall(pat, text, re.DOTALL):
            try:
                if "object" in json.loads(mm):
                    return True
            except Exception:
                continue
    return False

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ngl", type=int, required=True)
    ap.add_argument("--runs", type=int, default=3)
    ap.add_argument("--warmup", type=int, default=1)
    ap.add_argument("--image", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--temp-limit", type=float, default=80.0)
    ap.add_argument("--req-timeout", type=int, default=300)
    args = ap.parse_args()

    label = "all(-1)" if args.ngl < 0 else str(args.ngl)
    log_p = PROJECT / "Log" / f"llama_ngl{args.ngl}.log"
    log_p.parent.mkdir(exist_ok=True)
    img_b64 = base64.b64encode(Path(args.image).read_bytes()).decode()

    pre = read_temps(); h = hot(pre, args.temp_limit)
    result = {"ngl": args.ngl, "label": label, "image": os.path.basename(args.image),
              "temps_before": pre}
    if h:
        result.update({"aborted": True, "reason": f"NONG truoc khi chay: {h[0]}={h[1]}C"})
        _write(args.out, result); print(json.dumps(result, ensure_ascii=False)); return

    mem_idle = mem_snapshot()
    print(f"[ngl={label}] khoi dong llama-server... (RAM idle {mem_idle:.0f}MB)", flush=True)
    proc, logf = start_server(args.ngl, log_p)
    try:
        if not wait_health():
            proc.send_signal(signal.SIGINT); time.sleep(2)
            result.update({"aborted": True, "reason": "llama-server khong /health OK trong 240s",
                           "server_log_tail": Path(log_p).read_text(errors='ignore')[-1500:]})
            _write(args.out, result); print(json.dumps(result, ensure_ascii=False)); return

        logf.flush()
        gpu = parse_gpu_mem(log_p)
        print(f"[ngl={label}] server READY | VRAM~{gpu['vram_mib']}MiB | {gpu['layers_offloaded']}", flush=True)

        payload = build_payload(img_b64)

        # RAM sampler chay nen trong khi infer
        peak = {"sys_mb": mem_idle, "rss_mb": 0.0, "stop": False}
        def sampler():
            while not peak["stop"]:
                peak["sys_mb"] = max(peak["sys_mb"], mem_snapshot())
                peak["rss_mb"] = max(peak["rss_mb"], proc_rss_mb(proc.pid))
                time.sleep(0.4)
        th = threading.Thread(target=sampler, daemon=True); th.start()

        # warmup
        for _ in range(args.warmup):
            try:
                requests.post(f"{URL}/v1/chat/completions", json=payload, timeout=args.req_timeout)
            except Exception as e:
                print(f"[ngl={label}] warmup loi: {e}", flush=True)

        times, ok = [], 0
        for i in range(args.runs):
            ht = hot(read_temps(), args.temp_limit)
            if ht:
                result["thermal_abort_midrun"] = f"{ht[0]}={ht[1]}C"
                print(f"[ngl={label}] NONG giua chung -> dung: {ht}", flush=True)
                break
            t0 = time.perf_counter()
            try:
                r = requests.post(f"{URL}/v1/chat/completions", json=payload, timeout=args.req_timeout)
                dt = time.perf_counter() - t0
                times.append(dt)
                txt = r.json()["choices"][0]["message"]["content"] if r.status_code == 200 else ""
                if r.status_code == 200 and json_ok(txt):
                    ok += 1
                print(f"[ngl={label}] run {i+1}/{args.runs}: {dt:.2f}s json_ok={json_ok(txt)}", flush=True)
            except Exception as e:
                dt = time.perf_counter() - t0
                times.append(dt)
                print(f"[ngl={label}] run {i+1} loi sau {dt:.1f}s: {e}", flush=True)

        peak["stop"] = True; time.sleep(0.5)

        if times:
            result.update({
                "runs": len(times),
                "latency_avg_s": round(sum(times)/len(times), 2),
                "latency_min_s": round(min(times), 2),
                "latency_max_s": round(max(times), 2),
                "json_ok": ok, "json_ok_rate": f"{ok}/{len(times)} ({round(ok/len(times)*100)}%)",
            })
        result.update({
            "vram_mib": gpu["vram_mib"], "gpu_detail": gpu,
            "ram_peak_used_mb": round(peak["sys_mb"], 0),
            "ram_idle_mb": round(mem_idle, 0),
            "ram_delta_mb": round(peak["sys_mb"] - mem_idle, 0),
            "llama_rss_peak_mb": round(peak["rss_mb"], 0),
            "temps_after": read_temps(),
        })
    finally:
        proc.send_signal(signal.SIGINT)
        try: proc.wait(timeout=15)
        except Exception: proc.kill()
        logf.close()
        time.sleep(2)  # cho GPU mem giai phong truoc config sau

    _write(args.out, result)
    print(json.dumps(result, ensure_ascii=False))

def _write(out, obj):
    with open(out, "a", encoding="utf-8") as f:
        f.write(json.dumps(obj, ensure_ascii=False) + "\n")

if __name__ == "__main__":
    main()
