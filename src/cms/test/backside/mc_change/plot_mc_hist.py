#!/usr/bin/env python3
"""Paper figure: MC skew distributions of the three full-network Ibex clock
networks under the literature variation profile (Table: mc_change).

Reads the chunk mt0s, computes per-chip skew (max-min of t_sink_*), and plots
overlaid histograms.  Outputs ibex_mc_hist.pdf/.png next to this script.
"""
import glob
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))

SERIES = [  # (folder, chunk-glob, label, color)   fixed categorical order
    ("full_bslcb",    "bslcb_lit_c*.mt0",    "BS+LCB",    "#2a78d6"),
    ("full_fslcb",    "fslcb_lit_c*.mt0",    "FS+LCB",    "#eb6834"),
    ("full_fsdirect", "fsdirect_lit_c*.mt0", "FS Direct", "#1baf7a"),
]


def skews_ps(folder, pattern):
    vals = []
    first = True
    for mt0 in sorted(glob.glob(os.path.join(HERE, folder, pattern))):
        names, keep_nominal = None, first
        first = False
        with open(mt0) as f:
            for line in f:
                t = line.split()
                if not t:
                    continue
                if names is None:
                    if t[0] == "index":
                        names = t
                        tcols = [i for i, n in enumerate(names)
                                 if n.startswith("t_sink_")]
                    continue
                try:
                    row = [float(x) for x in t]
                except ValueError:
                    continue
                if len(row) != len(names):
                    continue
                if row[0] == 1 and not keep_nominal:
                    continue  # duplicate nominal sample in later chunks
                ts = np.array([row[i] for i in tcols])
                vals.append((ts.max() - ts.min()) * 1e12)
    return np.array(vals)


fig, ax = plt.subplots(figsize=(3.6, 2.3), dpi=300)
bins = np.arange(4.0, 46.0, 0.5)

for folder, pat, label, color in SERIES:
    s = skews_ps(folder, pat)
    ax.hist(s, bins=bins, density=True, histtype="stepfilled",
            facecolor=color, alpha=0.40, edgecolor=color, linewidth=1.2,
            label=f"{label}")
    # direct label with the headline stats above each distribution
    y = np.histogram(s, bins=bins, density=True)[0].max()
    ax.annotate(f"{label}\n{s.mean():.2f} $\\pm$ {s.std(ddof=1):.2f} ps",
                xy=(s.mean(), y), xytext=(s.mean(), y * 1.06),
                ha="center", va="bottom", fontsize=6.5, color="#333333",
                linespacing=1.2)

ax.set_xlabel("Clock skew (ps)", fontsize=8)
ax.set_ylabel("Probability density", fontsize=8)
ax.tick_params(labelsize=7)
ax.set_xlim(4, 46)
ax.set_ylim(top=ax.get_ylim()[1] * 1.28)
ax.spines[["top", "right"]].set_visible(False)
ax.grid(axis="y", linewidth=0.3, alpha=0.35)
ax.set_axisbelow(True)
ax.legend(fontsize=6.5, frameon=False, loc="upper right")

fig.tight_layout(pad=0.3)
fig.savefig(os.path.join(HERE, "ibex_mc_hist.pdf"))
fig.savefig(os.path.join(HERE, "ibex_mc_hist.png"))
print("wrote ibex_mc_hist.pdf/.png")
