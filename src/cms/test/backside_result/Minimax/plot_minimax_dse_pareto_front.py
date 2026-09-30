#!/usr/bin/env python3
# Minimax DSE Pareto plot from the Optuna study:
#   gray cloud   = every valid (COMPLETE) trial
#   green points = non-dominated Pareto frontier (staircase line)
#   labels + side table = the frontier configs (pitch/driver/fanout/LCB/layers)
import optuna
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

I = "/home/wali2/backside/OpenROAD/src/cms/test/backside_result/Minimax"
DB = f"sqlite:///{I}/dse/minimax_dse.db"
study = optuna.load_study(study_name="minimax_mesh_dse", storage=DB)

done = [t for t in study.trials if t.state.name == "COMPLETE"]
# dedupe identical configs (cached re-evals)
seen, uniq = set(), []
for t in done:
    key = tuple(sorted(t.params.items()))
    if key not in seen:
        seen.add(key)
        uniq.append(t)
pts = [(t.values[1], t.values[0], t.params) for t in uniq]   # (power, skew, params)

# non-dominated set (minimize both)
front = []
for pw, sk, prm in pts:
    if not any(pw2 <= pw and sk2 <= sk and (pw2 < pw or sk2 < sk)
               for pw2, sk2, _ in pts):
        front.append((pw, sk, prm))
front.sort()

def tag(p):
    return (f"{p['PITCH']:>5} {p['MBUF']:>4} {p['FMAX']:>4} "
            f"{p['SBUF']:>4} {p['CLKLAYERS']:>6}")

fig, ax = plt.subplots(figsize=(10, 6.6))
# frontier staircase
fx = [q[0] for q in front]; fy = [q[1] for q in front]
ax.plot(fx, fy, "-", color="#9e9e9e", lw=1.3, zorder=3)
# 18 distinct SATURATED colors (tab10 + Dark2) -- readable as text too
cmap = list(plt.cm.tab10.colors) + list(plt.cm.Dark2.colors)
for i, (pw, sk, prm) in enumerate(front):
    ax.scatter([pw], [sk], s=170, marker="o", color=cmap[i % len(cmap)], edgecolors="k",
               linewidths=0.7, zorder=4,
               label="Pareto frontier" if i == 0 else None)
    ax.annotate(f"{(chr(65+i) if i<26 else "A"+chr(39+i))}", (pw, sk), textcoords="offset points",
                xytext=(-12, 4), fontsize=9, fontweight="bold", color="black")

ax.set_xlabel("Clock power (mW)")
ax.set_ylabel("Worst-case skew, max-min (ps)")
ax.set_title("Minimax - Skew vs Power (Bayesian DSE, backside mesh)",
             fontsize=13, fontweight="bold")
ax.grid(alpha=0.3)


# per-row colored legend table (row color = point color) on a white panel
from matplotlib.patches import FancyBboxPatch
y0, dy = 0.97, 0.0195
panel_h = (len(front) + 2) * dy
ax.add_patch(FancyBboxPatch((0.545, y0 - panel_h + dy), 0.445, panel_h,
                            boxstyle="round,pad=0.008", transform=ax.transAxes,
                            fc="white", ec="#9e9e9e", alpha=0.93, zorder=5))
hdr = (f"{'':2s} {'skew':>7} {'power':>7}  {'Pitch':>5} {'Mbuf':>4} "
       f"{'Fmax':>4} {'Sbuf':>4} {'Layers':>6}")
ax.text(0.985, y0, hdr, transform=ax.transAxes, fontsize=8.4,
        va="top", ha="right", family="monospace", fontweight="bold", zorder=6)
for i, q in enumerate(front):
    ax.text(0.985, y0 - (i + 1) * dy,
            f"{(chr(65+i) if i<26 else "A"+chr(39+i))}: {q[1]:5.1f}ps {q[0]:5.2f}mW  {tag(q[2])}",
            transform=ax.transAxes, fontsize=8.4, va="top", ha="right",
            family="monospace", color=cmap[i % len(cmap)], fontweight="bold", zorder=6)

out = f"{I}/minimax_dse_pareto_front.pdf"
plt.tight_layout()
plt.savefig(out, bbox_inches="tight", dpi=130)
plt.savefig(out.replace(".pdf", ".png"), bbox_inches="tight", dpi=130)
print("wrote", out)
for i, (pw, sk, prm) in enumerate(front):
    print(f"{(chr(65+i) if i<26 else "A"+chr(39+i))}: skew={sk:5.1f}ps pw={pw:5.2f}mW  {tag(prm)}")
