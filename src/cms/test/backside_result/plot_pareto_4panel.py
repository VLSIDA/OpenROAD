#!/usr/bin/env python3
# 4-panel Pareto figure (skew vs power) from the checkerboard backside studies.
# Recovered from the 2026-08-09 session scratchpad (pareto_4panel.py) and made
# permanent here.  Reads the four Optuna checkerboard DSE databases and writes
# pareto_4panel.{pdf,png} next to this script.
#
# Run:  /home/wali2/backside/dse_venv/bin/python plot_pareto_4panel.py
import os
import optuna
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

# IEEE / ISCAS require embedded Type 1 or TrueType fonts.  Matplotlib's default
# is Type 3, which the IEEE PDF eXpress check rejects.  Type 42 = TrueType.
plt.rcParams["pdf.fonttype"] = 42
plt.rcParams["ps.fonttype"] = 42

B = os.path.dirname(os.path.abspath(__file__))
OUT = f"{B}/pareto_4panel"
CFG = [
    ("Ibex",    "Ibex",    "ibex_checker_mesh_dse",    "dse_checker/ibex_checker_dse.db"),
    ("JPEG",    "Jpeg",    "jpeg_checker_mesh_dse",    "dse_checker/jpeg_checker_dse.db"),
    ("Minimax", "Minimax", "minimax_checker_mesh_dse", "dse_checker/minimax_checker_dse.db"),
    ("FlooNoC", "Floonoc", "floonoc_checker_mesh_dse", "dse_checker/floonoc_checker_dse.db"),
]

fig, axes = plt.subplots(1, 4, figsize=(16, 3.6))
for ax, (name, dirn, study, db) in zip(axes, CFG):
    st = optuna.load_study(study_name=study, storage=f"sqlite:///{B}/{dirn}/{db}")
    done = [t for t in st.trials if t.state.name == "COMPLETE" and t.values]
    seen, pts = set(), []
    for t in done:
        k = tuple(sorted(t.params.items()))
        if k not in seen:
            seen.add(k)
            pts.append((t.values[1], t.values[0]))
    front = sorted(q for q in pts if not any(
        p2 <= q[0] and s2 <= q[1] and (p2, s2) != q for p2, s2 in pts))
    ax.scatter([p for p, s in pts], [s for p, s in pts], s=9,
               color="#9ecae1", alpha=0.55, lw=0, label="valid trials")
    ax.step([p for p, s in front], [s for p, s in front], where="post",
            color="#1a56a0", lw=1.8, zorder=3)
    ax.scatter([p for p, s in front], [s for p, s in front], s=34,
               color="#1a56a0", edgecolors="k", lw=0.5, zorder=4,
               label="Pareto frontier")
    ax.set_title(name, fontsize=12, fontweight="bold")
    ax.set_xlabel("Clock power (mW)", fontsize=10)
    ax.grid(alpha=0.3)
    ax.tick_params(labelsize=9)
    ax.set_ylim(0, min(max(s for _, s in front) * 1.5, 60))
axes[0].set_ylabel("Worst-case skew (ps)", fontsize=10)
axes[0].legend(loc="upper right", fontsize=8, framealpha=0.9)
plt.tight_layout()
plt.savefig(OUT + ".pdf", bbox_inches="tight")
plt.savefig(OUT + ".png", bbox_inches="tight", dpi=150)
print("wrote", OUT + ".pdf")
