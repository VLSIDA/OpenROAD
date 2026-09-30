#!/usr/bin/env python3
"""Compute skew statistics from a Monte Carlo .mt0 (one row per iteration).

Per iteration: skew = max(t_sink_*) - min(t_sink_*).
Reports mean/sigma/min/max/3-sigma skew and a convergence check.

Accepts multiple mt0 files (parallel chunks from run_mc.py); sample 1 of
every chunk is HSPICE's nominal run, so it is kept only from the first file.

Usage: mc_skew.py <file.mt0> [more.mt0 ...]
"""
import sys

import numpy as np


def parse_mt0(path):
    """MEASFORM=1: a names line starting with 'index', then one row/iteration."""
    names, rows = None, []
    with open(path) as f:
        for line in f:
            toks = line.split()
            if not toks:
                continue
            if names is None:
                if toks[0] == "index":
                    names = toks
                continue
            vals = []
            for t in toks:
                try:
                    vals.append(float(t))
                except ValueError:
                    vals.append(np.nan)  # 'failed' measures
            if len(vals) == len(names):
                rows.append(vals)
    if names is None:
        sys.exit("no 'index ...' names line found - not a MEASFORM=1 mt0?")
    return names, np.array(rows)


def main():
    names, arr = parse_mt0(sys.argv[1])
    for extra in sys.argv[2:]:
        n2, a2 = parse_mt0(extra)
        if n2 != names:
            sys.exit(f"{extra}: measure columns differ from {sys.argv[1]}")
        arr = np.vstack([arr, a2[a2[:, 0] != 1]])  # drop duplicate nominal
    tcols = [k for k, n in enumerate(names) if n.startswith("t_sink_")]
    if not tcols:
        sys.exit("no t_sink_* measures found")
    t = arr[:, tcols] * 1e12  # ps
    nfail = int(np.isnan(t).sum())
    skew = np.nanmax(t, axis=1) - np.nanmin(t, axis=1)
    n = len(skew)
    mu, sd = skew.mean(), skew.std(ddof=1) if n > 1 else 0.0

    print(f"iterations: {n}   sinks: {len(tcols)}   failed measures: {nfail}")
    print(f"skew  mean = {mu:8.3f} ps")
    print(f"skew  sigma= {sd:8.3f} ps")
    print(f"skew  min  = {skew.min():8.3f} ps")
    print(f"skew  max  = {skew.max():8.3f} ps")
    print(f"mean + 3*sigma = {mu + 3 * sd:8.3f} ps")
    if n >= 20:
        half = skew[: n // 2].std(ddof=1)
        print(f"convergence: sigma(first half) = {half:.3f} ps "
              f"vs sigma(all) = {sd:.3f} ps "
              f"({'ok' if abs(half - sd) < 0.2 * sd else 'still drifting'})")

    slews = [k for k, nm in enumerate(names) if nm.startswith("slew_sink_")]
    if slews:
        sl = arr[:, slews] * 1e12
        print(f"slew  mean = {np.nanmean(sl):8.3f} ps   "
              f"worst = {np.nanmax(sl):8.3f} ps")


if __name__ == "__main__":
    main()
