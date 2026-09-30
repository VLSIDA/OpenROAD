#!/usr/bin/env python3
# Ibex: BS+LCB vs FS+LCB(M5/M6) Pareto frontiers on one plot.
# Same architecture (mesh + LCB sink tier) on both sides -- the gap between
# the two staircases is the pure MEDIUM effect, architecture-optimal on each side.
import optuna
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

I = "/home/wali2/backside/OpenROAD/src/cms/test/backside_result/Ibex"

def load(study, db):
    st = optuna.load_study(study_name=study, storage=db)
    done = [t for t in st.trials if t.state.name == "COMPLETE" and t.values]
    seen, uniq = set(), []
    for t in done:
        k = tuple(sorted(t.params.items()))
        if k not in seen:
            seen.add(k); uniq.append(t)
    pts = [(t.values[1], t.values[0], t.params) for t in uniq]  # (pw, skew)
    front = [q for q in pts if not any(
        p2 <= q[0] and s2 <= q[1] and (p2 < q[0] or s2 < q[1])
        for p2, s2, _ in pts)]
    front.sort(key=lambda q: (q[0], q[1]))
    return pts, front

bs_pts, bs_front = load("ibex_mesh_dse", f"sqlite:///{I}/dse/ibex_dse.db")
fs_pts, fs_front = load("ibex_fslcb_m5m6_dse",
                        f"sqlite:///{I}/dse_fslcb_m5m6/ibex_fslcb_m5m6_dse.db")

fig, ax = plt.subplots(figsize=(9.5, 6.2))
# clouds
ax.scatter([p for p, s, _ in bs_pts], [s for p, s, _ in bs_pts],
           s=14, color="#9ecae1", alpha=0.45, lw=0, label=None)
ax.scatter([p for p, s, _ in fs_pts], [s for p, s, _ in fs_pts],
           s=14, color="#fdae6b", alpha=0.45, lw=0, label=None)
# frontier staircases (post-step)
def stair(front, color, label, marker):
    xs = [q[0] for q in front]; ys = [q[1] for q in front]
    ax.step(xs, ys, where="post", color=color, lw=2.0, zorder=4)
    ax.scatter(xs, ys, s=64, marker=marker, color=color, edgecolors="k",
               linewidths=0.6, zorder=5, label=label)
stair(bs_front, "#1a56a0", "Backside mesh + LCB (BM1/BM2)", "o")
stair(fs_front, "#d95f0e", "Frontside mesh + LCB (M5/M6)", "s")

# annotate the matched point J
ax.annotate("Point J", (1.013, 7.05), textcoords="offset points",
            xytext=(8, -14), fontsize=9, fontweight="bold", color="#1a56a0")

ax.set_xlabel("Clock power (mW)")
ax.set_ylabel("Worst-case skew, max-min (ps)")
ax.set_title("Ibex — Skew vs Power: backside vs frontside mesh, same LCB architecture",
             fontsize=12, fontweight="bold")
ax.set_xlim(0.7, 2.0)
ax.set_ylim(0, 40)
ax.grid(alpha=0.3)
ax.legend(loc="upper right", fontsize=10)

out = f"{I}/ibex_bs_vs_fslcb_pareto.pdf"
plt.tight_layout()
plt.savefig(out, bbox_inches="tight")
plt.savefig(out.replace(".pdf", ".png"), bbox_inches="tight", dpi=140)
print("wrote", out)
print(f"BS: {len(bs_pts)} valid trials, {len(bs_front)} frontier pts")
print(f"FS+LCB M5/M6: {len(fs_pts)} valid trials, {len(fs_front)} frontier pts")
