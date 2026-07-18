#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import json, re, os
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.ticker import ScalarFormatter

ROOT = Path(__file__).resolve().parent.parent
DOCS = ROOT / "docs"
LOG = ROOT / "Log"
OUT = DOCS / "figures"
OUT.mkdir(exist_ok=True)

# ---- "Thesis" style (serif, faint grid, color-blind-safe Okabe-Ito palette) ----
plt.rcParams.update({
    "font.family": "serif",
    "font.size": 11, "axes.titlesize": 12, "axes.labelsize": 11,
    "xtick.labelsize": 10, "ytick.labelsize": 10, "legend.fontsize": 9,
    "figure.dpi": 300, "savefig.dpi": 300, "savefig.bbox": "tight",
    "axes.grid": True, "grid.alpha": 0.3, "grid.linewidth": 0.5,
    "axes.axisbelow": True,
})
C = {"blue": "#0072B2", "orange": "#E69F00", "green": "#009E73",
     "verm": "#D55E00", "sky": "#56B4E9", "gray": "#999999",
     "purple": "#CC79A7", "yellow": "#F0E442"}

def load(name):
    return json.loads((DOCS / name).read_text(encoding="utf-8"))

def save(fig, name):
    p = OUT / name
    fig.savefig(p)
    plt.close(fig)
    print("  ->", p.relative_to(ROOT))


# ============================ Figure 4.1 - YOLO ============================
def fig_yolo():
    d = load("bang_4_1_yolo_performance.json")["ket_qua"]
    labels = ["PC\n(CPU)", "Jetson\nCPU", "Jetson\nCUDA 15W", "Jetson\nCUDA MAXN"]
    keys = ["pc_cpu", "jetson_cpu_only", "jetson_cuda"]
    fps = [d["pc_cpu"]["fps_trung_binh"], d["jetson_cpu_only"]["fps_trung_binh"],
           d["jetson_cuda"]["fps_trung_binh"], 42.4]
    lat = [d["pc_cpu"]["do_tre_trung_binh_ms"], d["jetson_cpu_only"]["do_tre_trung_binh_ms"],
           d["jetson_cuda"]["do_tre_trung_binh_ms"], 23.6]
    lmin = [d["pc_cpu"]["do_tre_min_ms"], d["jetson_cpu_only"]["do_tre_min_ms"],
            d["jetson_cuda"]["do_tre_min_ms"], 23.1]
    lmax = [d["pc_cpu"]["do_tre_max_ms"], d["jetson_cpu_only"]["do_tre_max_ms"],
            d["jetson_cuda"]["do_tre_max_ms"], 24.5]
    cols = [C["gray"], C["orange"], C["sky"], C["blue"]]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(9, 3.6))
    x = np.arange(len(labels))
    b = ax1.bar(x, fps, color=cols, edgecolor="black", linewidth=0.5)
    ax1.set_yscale("log"); ax1.set_ylabel("FPS (log)")
    ax1.set_title("(a) Thông lượng YOLOv8n")
    ax1.set_xticks(x); ax1.set_xticklabels(labels)
    for r, v in zip(b, fps):
        ax1.text(r.get_x()+r.get_width()/2, v*1.1, f"{v:g}", ha="center", va="bottom", fontsize=9)
    ax1.set_ylim(0.9, max(fps)*2.6)   # headroom on top so labels do not overlap the frame

    yerr = [np.array(lat)-np.array(lmin), np.array(lmax)-np.array(lat)]
    ax2.bar(x, lat, color=cols, edgecolor="black", linewidth=0.5,
            yerr=yerr, capsize=4, error_kw={"elinewidth": 0.8})
    ax2.set_yscale("log"); ax2.set_ylabel("Độ trễ (ms, log)")
    ax2.set_title("(b) Độ trễ suy luận (min–max)")
    ax2.set_xticks(x); ax2.set_xticklabels(labels)
    for xi, v, vm in zip(x, lat, lmax):
        ax2.text(xi, vm*1.12, f"{v:g}", ha="center", va="bottom", fontsize=9)  # place labels above the error bar
    ax2.set_ylim(6, max(lmax)*2.3)   # headroom on top so labels do not overlap the frame
    fig.tight_layout()
    save(fig, "fig_4_1_yolo.png")


# ===================== Figure 4.2 - VLM latency + memory =====================
def fig_vlm():
    d = load("bang_4_2_vlm_ngpu_sweep.json")["ket_qua"]["jetson"]
    d = sorted(d, key=lambda r: (36 if r["n_gpu"] == -1 else r["n_gpu"]))
    xpos = np.arange(len(d))
    xlab = [("−1" if r["n_gpu"] == -1 else str(r["n_gpu"])) for r in d]
    lat = [r["do_tre_avg_s"] for r in d]
    lmin = [r["do_tre_min_s"] for r in d]
    lmax = [r["do_tre_max_s"] for r in d]
    vram = [r["vram_mib"] for r in d]
    rss = [r["ram_rss_mb"] for r in d]

    fig, ax = plt.subplots(figsize=(6.6, 4))
    l1, = ax.plot(xpos, lat, color=C["blue"], marker="o", ms=4, lw=1.0,
                  label="Độ trễ VLM (s)")
    ax.set_xlabel("Số layer offload lên GPU (n_gpu_layers)")
    ax.set_ylabel("Độ trễ trung bình (s)", color=C["blue"])
    ax.tick_params(axis="y", labelcolor=C["blue"])
    ax.set_xticks(xpos); ax.set_xticklabels(xlab)
    ax.spines["top"].set_visible(False)

    ax2 = ax.twinx()
    ax2.grid(False)
    l2, = ax2.plot(xpos, vram, color=C["verm"], marker="o", ms=4, lw=1.0, label="VRAM GPU (MiB)")
    l3, = ax2.plot(xpos, rss, color=C["green"], marker="o", ms=4, lw=1.0, ls="--", label="RAM RSS (MB)")
    ax2.set_ylabel("Bộ nhớ (MiB / MB)")
    ax2.spines["top"].set_visible(False)

    lines = [l1, l2, l3]
    ax.legend(lines, [ln.get_label() for ln in lines], loc="center right", framealpha=0.9)
    ax.set_title("Độ trễ & bộ nhớ Qwen2.5-VL-3B Q4_K_M theo n_gpu_layers")
    fig.tight_layout()
    save(fig, "fig_4_2_vlm_latency_memory.png")

    # 4.2b - stacked footprint (RAM RSS + VRAM ~ constant)
    fig, ax = plt.subplots(figsize=(6.6, 3.8))
    ax.bar(xpos, rss, color=C["green"], edgecolor="black", linewidth=0.5, label="RAM RSS (CPU)")
    ax.bar(xpos, vram, bottom=rss, color=C["verm"], edgecolor="black", linewidth=0.5, label="VRAM (GPU)")
    tot = np.array(rss) + np.array(vram)
    for xi, t in zip(xpos, tot):
        ax.text(xi, t+60, f"{t/1024:.2f}\nGB", ha="center", va="bottom", fontsize=8)
    ax.set_xticks(xpos); ax.set_xticklabels(xlab)
    ax.set_xlabel("n_gpu_layers"); ax.set_ylabel("Bộ nhớ (MB)")
    ax.set_ylim(0, max(tot)*1.18)
    ax.set_title("Tổng footprint ≈ hằng số — bộ nhớ chỉ dịch CPU↔GPU")
    ax.legend(loc="upper right", ncol=2, framealpha=0.9)
    fig.tight_layout()
    save(fig, "fig_4_2b_vlm_memory_stacked.png")


# ===================== Figure 4.3 - Accuracy of the 3 methods =====================
def fig_accuracy():
    d = load("bang_4_3_accuracy_results.json")["ket_qua"]
    labels = ["YOLO-only", "VLM-only", "YOLO+VLM"]
    acc = [d["yolo_only"]["accuracy_pct"], d["vlm_only"]["accuracy_pct"], d["de_xuat"]["accuracy_pct"]]
    cor = [d["yolo_only"]["correct"], d["vlm_only"]["correct"], d["de_xuat"]["correct"]]
    tot = d["yolo_only"]["total"]
    cols = [C["gray"], C["blue"], C["green"]]
    fig, ax = plt.subplots(figsize=(5.4, 4))
    x = np.arange(len(labels))
    b = ax.bar(x, acc, color=cols, edgecolor="black", linewidth=0.5)
    ax.axhline(50, color=C["verm"], ls="--", lw=1, label="Ngẫu nhiên 50%")
    for r, a, c in zip(b, acc, cor):
        ax.text(r.get_x()+r.get_width()/2, a+1, f"{a:.1f}%\n({c}/{tot})",
                ha="center", va="bottom", fontsize=9)
    ax.set_xticks(x); ax.set_xticklabels(labels)
    ax.set_ylabel("Độ chính xác collectibility (%)")
    ax.set_ylim(0, 100)
    ax.set_title(f"Độ chính xác trên {tot} ảnh tự chụp Astra")
    ax.legend(loc="upper right")
    fig.tight_layout()
    save(fig, "fig_4_3_accuracy.png")


# ===================== Figure 4.4 - End-to-end breakdown =====================
def fig_e2e():
    d = load("bang_4_4_end_to_end.json")["ket_qua"]
    yolo_s = d["khi_co_trigger"]["yolo_ms"]/1000.0
    vlm_s = d["khi_co_trigger"]["vlm_avg_s"]
    total = yolo_s + vlm_s
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(9, 3.4),
                                   gridspec_kw={"width_ratios": [1.4, 1]})
    # (a) stacked breakdown
    ax1.barh([0], [yolo_s], color=C["sky"], edgecolor="black", linewidth=0.5, label=f"YOLO ({yolo_s*1000:.1f} ms)")
    ax1.barh([0], [vlm_s], left=[yolo_s], color=C["blue"], edgecolor="black", linewidth=0.5, label=f"VLM ({vlm_s:.2f} s)")
    ax1.set_yticks([0]); ax1.set_yticklabels(["Có trigger"])
    ax1.set_xlabel("Thời gian (s)")
    ax1.set_title(f"(a) Phân rã end-to-end ≈ {total:.2f} s")
    ax1.text(yolo_s+vlm_s/2, 0, f"VLM ≈ {vlm_s/total*100:.1f}%", ha="center", va="center",
             color="white", fontsize=10, fontweight="bold")
    ax1.legend(loc="lower right", framealpha=0.9)
    ax1.grid(axis="y")
    ax1.set_ylim(-0.8, 0.8)   # top/bottom margin so the bar does not touch the frame
    # (b) trigger vs no-trigger (log)
    notrig = d["khi_khong_trigger"]["end_to_end_avg_s"]
    vals = [total, notrig]
    b = ax2.bar([0, 1], vals, color=[C["blue"], C["green"]], edgecolor="black", linewidth=0.5)
    ax2.set_yscale("log"); ax2.set_ylabel("End-to-end (s, log)")
    ax2.set_xticks([0, 1]); ax2.set_xticklabels(["Có\ntrigger", "Không\ntrigger"])
    ax2.set_title("(b) Lợi ích của cơ chế trigger")
    for xi, v in zip([0, 1], vals):
        ax2.text(xi, v*1.15, f"{v:.3g} s", ha="center", va="bottom", fontsize=9)
    ax2.set_ylim(0.01, max(vals)*4)   # headroom on top so labels do not overlap the frame
    fig.tight_layout()
    save(fig, "fig_4_4_end_to_end.png")


# ===================== Figure 4.5 - Edge time-series =====================
def parse_tegrastats(path):
    t, gpu, cpu, gr3d = [], [], [], []
    pat_g = re.compile(r"GPU@([\d.]+)C"); pat_c = re.compile(r"CPU@([\d.]+)C")
    pat_3 = re.compile(r"GR3D_FREQ (\d+)%")
    pat_ts = re.compile(r"^\d\d-\d\d-\d{4} (\d\d):(\d\d):(\d\d)")
    t0 = None
    for ln in path.read_text(errors="ignore").splitlines():
        mts = pat_ts.match(ln); mg = pat_g.search(ln); mc = pat_c.search(ln); m3 = pat_3.search(ln)
        if not (mts and mg and mc and m3):
            continue
        h, m, s = int(mts.group(1)), int(mts.group(2)), int(mts.group(3))
        sec = h*3600 + m*60 + s
        if t0 is None:
            t0 = sec
        t.append((sec - t0)/3600.0)   # hours since the start of the session
        gpu.append(float(mg.group(1))); cpu.append(float(mc.group(1))); gr3d.append(int(m3.group(1)))
    return np.array(t), np.array(gpu), np.array(cpu), np.array(gr3d)

def fig_edge():
    path = LOG / "tegrastats_bench.log"
    if not path.exists():
        print("  (skip 4.5: missing", path, ")"); return
    t, gpu, cpu, gr3d = parse_tegrastats(path)

    # --- ESTIMATED power (NOT measured directly): inferred from the real GR3D util ---
    # P = P_idle + (P_max - P_idle) * GR3D/100 ; AGX Xavier MAXN: idle ~9W, full ~30W
    P_IDLE, P_MAX = 9.0, 30.0
    power_est = P_IDLE + (P_MAX - P_IDLE) * (gr3d / 100.0)
    load_avg = power_est[gr3d > 50].mean() if (gr3d > 50).any() else power_est.mean()

    fig, (axA, axB) = plt.subplots(2, 1, figsize=(8, 6.6), sharex=True,
                                   gridspec_kw={"height_ratios": [1.25, 1]})
    # (a) temperature + GR3D util (measured)
    axA.plot(t, gpu, color=C["verm"], lw=1.0, label="Nhiệt GPU (°C)")
    axA.plot(t, cpu, color=C["blue"], lw=1.0, label="Nhiệt CPU (°C)")
    axA.axhline(80, color="red", ls="--", lw=1, label="Ngưỡng tự dừng 80°C")
    axA.set_ylabel("Nhiệt độ (°C)"); axA.set_ylim(30, 85)
    axA.spines["top"].set_visible(False)
    axA2 = axA.twinx(); axA2.grid(False)
    axA2.fill_between(t, gr3d, color=C["gray"], alpha=0.22, step="mid", label="GR3D util (%)")
    axA2.set_ylabel("GPU util GR3D (%)"); axA2.set_ylim(0, 105)
    axA2.spines["top"].set_visible(False)
    h1, l1 = axA.get_legend_handles_labels(); h2, l2 = axA2.get_legend_handles_labels()
    axA.legend(h1+h2, l1+l2, loc="upper center", ncol=2, framealpha=0.9, fontsize=8.5)
    axA.set_title("(a) Nhiệt độ & mức sử dụng GPU — đo thật")

    # (b) ESTIMATED power
    axB.fill_between(t, power_est, color=C["orange"], alpha=0.30, step="mid")
    axB.plot(t, power_est, color=C["verm"], lw=0.9)
    axB.axhline(load_avg, color=C["blue"], ls="--", lw=1, label=f"TB khi tải ≈ {load_avg:.1f} W")
    axB.axhline(P_IDLE, color=C["gray"], ls=":", lw=1, label=f"Nhàn rỗi ≈ {P_IDLE:.0f} W")
    axB.set_xlabel("Thời gian chạy (giờ)")
    axB.set_ylabel("Công suất ƯỚC LƯỢNG (W)")
    axB.set_ylim(0, P_MAX*1.12)
    axB.spines["top"].set_visible(False)
    axB.legend(loc="upper center", ncol=2, framealpha=0.9, fontsize=8.5)
    axB.set_title("(b) Công suất ƯỚC LƯỢNG từ GR3D util — KHÔNG đo trực tiếp")
    axB.text(0.985, 0.07, f"Mô hình: P = {P_IDLE:.0f} + {P_MAX-P_IDLE:.0f}×(GR3D/100) W  ·  suy từ util đo thật",
             transform=axB.transAxes, ha="right", va="bottom", fontsize=7.4,
             style="italic", color="#666",
             bbox=dict(boxstyle="round,pad=0.3", fc="white", ec="#cccccc"))

    fig.suptitle("Hành vi edge — Jetson AGX Xavier MAXN", fontsize=13, fontweight="bold")
    fig.tight_layout()
    save(fig, "fig_4_5_edge_timeseries.png")


if __name__ == "__main__":
    print("Exporting figures ->", OUT.relative_to(ROOT))
    fig_yolo(); fig_vlm(); fig_accuracy(); fig_e2e(); fig_edge()
    print("Done.")
