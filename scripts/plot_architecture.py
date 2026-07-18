#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch, Polygon

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "docs" / "figures"
OUT.mkdir(parents=True, exist_ok=True)

plt.rcParams.update({
    "font.family": "sans-serif",
    "font.sans-serif": ["DejaVu Sans"],
    "figure.dpi": 300, "savefig.dpi": 300, "savefig.bbox": "tight",
})

# Color-blind-safe palette (Okabe-Ito): light fill + bold edge
FILL = {"cam": "#ECECEC", "yolo": "#CDE7F5", "gate": "#FBF3B0",
        "vlm": "#BFE3D5", "ctrl": "#F6DDC4", "skip": "#EAEAEA"}
EDGE = {"cam": "#666666", "yolo": "#0072B2", "gate": "#B59A00",
        "vlm": "#009E73", "ctrl": "#D55E00", "skip": "#999999"}

fig, ax = plt.subplots(figsize=(12, 5.4))
ax.set_xlim(0, 100); ax.set_ylim(0, 52); ax.axis("off")

def box(cx, cy, w, h, title, sub, key):
    ax.add_patch(FancyBboxPatch(
        (cx - w/2, cy - h/2), w, h,
        boxstyle="round,pad=0.4,rounding_size=1.2",
        linewidth=1.6, edgecolor=EDGE[key], facecolor=FILL[key]))
    ax.text(cx, cy + h*0.16, title, ha="center", va="center",
            fontsize=11.5, fontweight="bold", color="#111111")
    if sub:
        ax.text(cx, cy - h*0.22, sub, ha="center", va="center",
                fontsize=8.6, color="#333333")
    return dict(l=cx-w/2, r=cx+w/2, t=cy+h/2, b=cy-h/2, cx=cx, cy=cy)

def arrow(p1, p2, color="#222222", lw=1.7, cs=None, label=None, lpos=None, ls="-"):
    kw = dict(arrowstyle="-|>", mutation_scale=16, lw=lw, color=color, linestyle=ls)
    if cs: kw["connectionstyle"] = cs
    ax.add_patch(FancyArrowPatch(p1, p2, **kw))
    if label:
        lx, ly = lpos if lpos else ((p1[0]+p2[0])/2, (p1[1]+p2[1])/2)
        ax.text(lx, ly, label, ha="center", va="center", fontsize=8.6,
                color=color, fontweight="bold",
                bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="none", alpha=0.85))

YM = 35  # main-row axis
# ---- main blocks ----
cam  = box(9,  YM, 15, 12, "Camera Astra", "RGB 640×480\n+ Depth", "cam")
yolo = box(31, YM, 17, 12, "YOLOv8n", "Phát hiện vật\n24–42 FPS", "yolo")
# trigger gate (diamond)
gx, gy, gh = 51, YM, 8.5
ax.add_patch(Polygon([(gx, gy+gh), (gx+gh, gy), (gx, gy-gh), (gx-gh, gy)],
                     closed=True, facecolor=FILL["gate"], edgecolor=EDGE["gate"], linewidth=1.6))
ax.text(gx, gy+1.2, "Stable\nTrigger?", ha="center", va="center", fontsize=10, fontweight="bold")
ax.text(gx, gy-3.0, "≥2s · cd 5s", ha="center", va="center", fontsize=7.6, color="#555")
gate = dict(l=gx-gh, r=gx+gh, t=gy+gh, b=gy-gh, cx=gx, cy=gy)
vlm  = box(74, YM, 19, 12, "VLM Qwen2.5-VL-3B", "Chính sách thu gom\n+ tiếng Việt · ~2.4s", "vlm")
ctrl = box(93, YM, 11, 12, "Điều khiển", "Mecanum\n+ gripper", "ctrl")

# ---- main-row arrows ----
arrow((cam["r"], YM), (yolo["l"], YM))
arrow((yolo["r"], YM), (gate["l"], YM))
arrow((gate["r"], YM), (vlm["l"], YM), color=EDGE["vlm"],
      label="YES", lpos=((gate["r"]+vlm["l"])/2, YM+3.2))
arrow((vlm["r"], YM), (ctrl["l"], YM), color=EDGE["ctrl"],
      label="JSON", lpos=((vlm["r"]+ctrl["l"])/2, YM+3.0))

# ---- NO-trigger branch (skip the VLM) - orthogonal routing ----
skip = box(51, 11, 34, 9, "collectible = False",
           "Khung trống / không có vật → bỏ qua, KHÔNG gọi VLM", "skip")
# IN: straight down from the trigger gate to the skip block
arrow((gate["cx"], gate["b"]), (gate["cx"], skip["t"]),
      color=EDGE["skip"], label="NO", lpos=(gate["cx"] - 3, 21))
# OUT: right-angle elbow (horizontal then up 90 deg) at x=93, avoiding the JSON callout
arrow((skip["r"], skip["cy"]), (ctrl["cx"], ctrl["b"]),
      color=EDGE["skip"], cs="angle,angleA=0,angleB=90,rad=0")

# ---- layer labels ----
for x, txt in [(20, "Tầng 1 · Thị giác"), (74, "Tầng 2 · Brain (VLM)"), (93, "Tầng 3 · Điều khiển")]:
    ax.text(x, 47.5, txt, ha="center", va="center", fontsize=9.5,
            style="italic", color="#444")

# ---- callout JSON output ----
ax.text(74, 23.8, "{object, collectible, bbox,\nconfidence, reason}\nJSON OK 100%",
        ha="center", va="center", fontsize=8.0, color="#0a5", linespacing=1.3,
        bbox=dict(boxstyle="round,pad=0.35", fc="#F2FBF7", ec=EDGE["vlm"], lw=0.8))
ax.annotate("", xy=(74, 27.2), xytext=(74, vlm["b"]),
            arrowprops=dict(arrowstyle="-", lw=0.8, color=EDGE["vlm"]))

ax.set_title("YOLO (cổng trigger) + VLM (suy luận ngữ nghĩa, offline)",
             fontsize=15, fontweight="bold", pad=14)

# key takeaway caption
ax.text(50, 2.0,
        "Tốc độ của YOLO + suy luận tiếng Việt của VLM — không trả giá bằng tải tính toán liên tục nhờ cơ chế trigger.",
        ha="center", va="center", fontsize=8.8, style="italic", color="#555")

fig.tight_layout()
p = OUT / "fig_architecture.png"
fig.savefig(p); plt.close(fig)
print("->", p.relative_to(ROOT))
