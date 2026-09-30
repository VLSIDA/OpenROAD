#!/usr/bin/env python3
"""Summaries and plots for run_mcvar.py outputs.

  mc_stats.py summary DIR [DIR ...]        per-network stats table
  mc_stats.py hist DIR [DIR ...]           skew histogram PNG per dir
  mc_stats.py sweep DIR:LABEL [...]        wire-R sigma sweep table + plot
                                           (LABEL = frontside sigma, e.g. 0.05)

Stats exclude the nominal sample (is_nominal=1), which is reported on its
own line as the deterministic reference.
"""
import csv
import json
import sys
from pathlib import Path

import numpy as np

METRICS = [("skew_ps", "worst-case sink skew"),
           ("max_slew_ps", "max sink slew"),
           ("latency_ps", "clock latency")]


def load(d: Path):
    rows = list(csv.DictReader(open(d / "samples.csv")))
    meta = json.loads((d / "run_meta.json").read_text())
    rand = {m: np.array([float(r[m]) for r in rows if r["is_nominal"] == "0"])
            for m, _ in METRICS}
    nom = {m: [float(r[m]) for r in rows if r["is_nominal"] == "1"]
           for m, _ in METRICS}
    n_nom = len(nom["skew_ps"])
    return rows, meta, rand, {m: v[0] if v else np.nan for m, v in nom.items()}, n_nom


def summary(dirs):
    for d in dirs:
        d = Path(d)
        rows, meta, rand, nom, n_nom = load(d)
        n = len(rand["skew_ps"])
        print(f"\n== {d.name}  ({meta['network']}, seed={meta['seed']}, "
              f"sigma_r_front={meta['sigma_r_front']:.4g}, "
              f"sigma_r_back={meta['sigma_r_back']:.4g}, "
              f"sheets={meta['sheets']}, N={n} random + {n_nom} nominal) ==")
        print(f"{'metric':24s} {'nominal':>9s} {'mu':>9s} {'sigma':>9s} "
              f"{'mu+3sig':>9s} {'max':>9s}")
        for m, label in METRICS:
            x = rand[m]
            print(f"{label:24s} {nom[m]:9.3f} {x.mean():9.3f} "
                  f"{x.std(ddof=1):9.3f} {x.mean()+3*x.std(ddof=1):9.3f} "
                  f"{x.max():9.3f}")
        v = np.array([float(r["vdd"]) for r in rows if r["is_nominal"] == "0"])
        print(f"{'global Vdd draw (V)':24s} {'0.700':>9s} {v.mean():9.4f} "
              f"{v.std(ddof=1):9.4f}")
        half = rand["skew_ps"][: n // 2].std(ddof=1)
        full = rand["skew_ps"].std(ddof=1)
        drift = abs(half - full) / full if full else 0
        print(f"convergence: skew sigma first-half {half:.3f} vs all "
              f"{full:.3f} ps ({'ok' if drift < 0.2 else 'still drifting'})")


def hist(dirs):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    for d in dirs:
        d = Path(d)
        _, meta, rand, nom, _ = load(d)
        x = rand["skew_ps"]
        fig, ax = plt.subplots(figsize=(6, 4))
        ax.hist(x, bins=min(80, max(15, len(x) // 25)), color="#4878b0",
                edgecolor="white", linewidth=0.3)
        mu, sd = x.mean(), x.std(ddof=1)
        ax.axvline(nom["skew_ps"], color="k", ls="--", lw=1, label="nominal")
        ax.axvline(mu, color="#c44e52", lw=1.2, label=f"mu = {mu:.2f} ps")
        ax.axvline(mu + 3 * sd, color="#c44e52", ls=":", lw=1.2,
                   label=f"mu+3sig = {mu+3*sd:.2f} ps")
        ax.set_xlabel("worst-case sink skew (ps)")
        ax.set_ylabel("samples")
        ax.set_title(f"{meta['network']}  N={len(x)}  "
                     f"sigma_r_front={meta['sigma_r_front']:.3g}")
        ax.legend(fontsize=8)
        fig.tight_layout()
        out = d / "skew_hist.png"
        fig.savefig(out, dpi=140)
        print(f"wrote {out}")


def sweep(specs):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    pts = []
    for spec in specs:
        dstr, _, label = spec.partition(":")
        d = Path(dstr)
        _, meta, rand, _, _ = load(d)
        sig = float(label) if label else meta["sigma_r_front"]
        x = rand["skew_ps"]
        pts.append((sig, x.mean(), x.std(ddof=1), len(x)))
    pts.sort()
    print(f"{'sig_R_front':>11s} {'sig_R_back':>10s} {'N':>6s} "
          f"{'skew mu':>9s} {'sigma':>8s} {'mu+3sig':>9s}")
    for sig, mu, sd, n in pts:
        print(f"{sig:11.3f} {sig/2:10.3f} {n:6d} {mu:9.3f} {sd:8.3f} "
              f"{mu+3*sd:9.3f}")
    fig, ax = plt.subplots(figsize=(5.5, 4))
    s = np.array([p[0] for p in pts]) * 100
    y = np.array([p[1] + 3 * p[2] for p in pts])
    ax.plot(s, y, "o-", color="#4878b0")
    ax.set_xlabel("frontside wire-R sigma (%)  [backside = half]")
    ax.set_ylabel("mu + 3 sigma worst-case skew (ps)")
    ax.set_title("BS+LCB skew vs wire-R variability")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig("sweep_skew_vs_sigma_r.png", dpi=140)
    print("wrote sweep_skew_vs_sigma_r.png")


if __name__ == "__main__":
    if len(sys.argv) < 3 or sys.argv[1] not in ("summary", "hist", "sweep"):
        sys.exit(__doc__)
    {"summary": summary, "hist": hist, "sweep": sweep}[sys.argv[1]](sys.argv[2:])
