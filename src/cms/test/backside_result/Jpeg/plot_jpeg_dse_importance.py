#!/usr/bin/env python3
# Knob-importance bar chart (fANOVA) for the Ibex backside-mesh DSE:
# grouped horizontal bars, one group per knob, one bar per objective.
import optuna
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

I = "/home/wali2/backside/OpenROAD/src/cms/test/backside_result/Jpeg"
DB = f"sqlite:///{I}/dse/jpeg_dse.db"
study = optuna.load_study(study_name="jpeg_mesh_dse", storage=DB)
ncomplete = sum(t.state.name == "COMPLETE" for t in study.trials)

imp_skew = optuna.importance.get_param_importances(study, target=lambda t: t.values[0])
imp_pw   = optuna.importance.get_param_importances(study, target=lambda t: t.values[1])

LABEL = {"PITCH": "Mesh pitch", "MBUF": "Mesh-driver size",
         "FMAX": "LCB fanout", "SBUF": "LCB size",
         "CLKLAYERS": "Clock layer window"}
# order knobs by mean importance, most important on top
knobs = sorted(SPACE := imp_skew.keys(),
               key=lambda k: -(imp_skew[k] + imp_pw[k]) / 2)
sk = [imp_skew[k] * 100 for k in knobs]
pw = [imp_pw[k] * 100 for k in knobs]

y = np.arange(len(knobs))[::-1]
h = 0.38
fig, ax = plt.subplots(figsize=(8, 4.6))
b1 = ax.barh(y + h / 2, sk, height=h, color="#1f77b4", label="Skew")
b2 = ax.barh(y - h / 2, pw, height=h, color="#d62728", label="Power")
for bars in (b1, b2):
    for r in bars:
        ax.text(r.get_width() + 0.8, r.get_y() + r.get_height() / 2,
                f"{r.get_width():.1f}%", va="center", fontsize=8.5)
ax.set_yticks(y)
ax.set_yticklabels([LABEL[k] for k in knobs], fontsize=10)
ax.set_xlabel("Importance (fANOVA, % of objective variance explained)")
ax.set_title("JPEG backside mesh - knob importance"
             f"\n(fANOVA over {ncomplete} valid DSE trials)",
             fontsize=12, fontweight="bold")
ax.legend(fontsize=10, loc="lower right")
ax.set_xlim(0, max(max(sk), max(pw)) * 1.15)
ax.grid(axis="x", alpha=0.3)
out = f"{I}/jpeg_dse_importance.pdf"
plt.tight_layout()
plt.savefig(out, bbox_inches="tight", dpi=130)
plt.savefig(out.replace(".pdf", ".png"), bbox_inches="tight", dpi=130)
print("wrote", out)
