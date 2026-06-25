#!/usr/bin/env python3
"""Mesh buffer arrival-time / local-skew heatmaps.

Parses Vclk_* sources from a mesh SPICE deck (same source as
mesh_skew_local.py), bins each buffer to its (ix, iy) grid cell, and
renders two PNGs:

  <spice>_arrival_heatmap.png   per-buffer arrival time (ns)
  <spice>_local_skew_heatmap.png  per-buffer 2-hop NSEW local skew (ps)

Usage:
    python3 mesh_skew_heatmap.py <mesh.spice> [--hops 2] [--annotate]
"""

import argparse
import csv
import re
import sys

import numpy as np
import matplotlib.pyplot as plt

VCLK_RE = re.compile(
    r"^Vclk_\d+\s+\S+\s+0\s+PULSE\([^)]*\)\s*\$\s*"
    r"(mesh_buf_(\d+)_(\d+))/\S+\s+arrival=([\d.eE+\-]+)ns"
)


def parse_spice(path):
    buffers = []
    seen = set()
    with open(path) as f:
        for line in f:
            if not line.startswith("Vclk_"):
                continue
            m = VCLK_RE.match(line)
            if not m:
                continue
            name = m.group(1)
            if name in seen:
                continue
            seen.add(name)
            buffers.append((name, int(m.group(2)), int(m.group(3)),
                            float(m.group(4))))
    return buffers


def infer_pitch(vals):
    uniq = sorted(set(vals))
    if len(uniq) < 2:
        return 1
    return min(b - a for a, b in zip(uniq[:-1], uniq[1:]) if b > a)


def build_grid(buffers):
    xs = [b[1] for b in buffers]
    ys = [b[2] for b in buffers]
    px, py = infer_pitch(xs), infer_pitch(ys)
    x0, y0 = min(xs), min(ys)
    nx = round((max(xs) - x0) / px) + 1
    ny = round((max(ys) - y0) / py) + 1
    arr = np.full((ny, nx), np.nan)
    cells = {}
    for name, x, y, t in buffers:
        ix = round((x - x0) / px)
        iy = round((y - y0) / py)
        arr[iy, ix] = t
        cells[(ix, iy)] = t
    return arr, cells, (nx, ny), (px, py), (x0, y0)


def local_skew(cells, nx, ny, hops):
    out = np.full((ny, nx), np.nan)
    offs = [(h, 0) for h in range(-hops, hops + 1) if h] + \
           [(0, h) for h in range(-hops, hops + 1) if h]
    for (ix, iy), t in cells.items():
        vals = [t]
        for dx, dy in offs:
            v = cells.get((ix + dx, iy + dy))
            if v is not None:
                vals.append(v)
        out[iy, ix] = (max(vals) - min(vals)) * 1000.0  # ps
    return out


def load_sinks(path):
    """Read sinks.csv. Returns dict kind -> list[(x_dbu, y_dbu)].
    Backwards compatible with files that lack a 'kind' column (all 'mesh')."""
    out = {"mesh": [], "gated": []}
    with open(path) as f:
        r = csv.DictReader(f)
        for row in r:
            kind = row.get("kind", "mesh") or "mesh"
            out.setdefault(kind, []).append(
                (int(row["x_dbu"]), int(row["y_dbu"])))
    return out


def render(arr, title, cbar_label, out_path, annotate=False, cmap="viridis",
           sinks_ig=None):
    """White background, black mesh grid, triangle buffer at each intersection
    colored by value. `arr` is a (ny, nx) array, NaN where no buffer."""
    ny, nx = arr.shape
    fig, ax = plt.subplots(figsize=(max(6, nx * 0.5), max(5, ny * 0.5)))
    fig.patch.set_facecolor("white")
    ax.set_facecolor("white")

    # Black grid: one line per column and per row, drawn through buffer sites.
    for ix in range(nx):
        ax.plot([ix, ix], [-0.5, ny - 0.5], color="black",
                linewidth=0.6, zorder=1)
    for iy in range(ny):
        ax.plot([-0.5, nx - 0.5], [iy, iy], color="black",
                linewidth=0.6, zorder=1)

    # Triangle marker at each present buffer.
    xs, ys, vs = [], [], []
    for iy in range(ny):
        for ix in range(nx):
            v = arr[iy, ix]
            if not np.isnan(v):
                xs.append(ix)
                ys.append(iy)
                vs.append(v)
    size = max(40, min(400, 6000 / max(nx, ny)))
    # sinks_ig: dict kind -> list[(ix_float, iy_float)] in grid units.
    # blue=mesh, red=gated (downstream of a clock-gate, not on mesh).
    if sinks_ig:
        styles = {
            "mesh":  {"c": "#1f77ff", "label": "mesh sinks"},
            "gated": {"c": "#d62728", "label": "gated sinks"},
        }
        for kind, pts in sinks_ig.items():
            if not pts:
                continue
            sx = [p[0] for p in pts]
            sy = [p[1] for p in pts]
            st = styles.get(kind, {"c": "#888", "label": kind})
            ax.scatter(sx, sy, s=4, c=st["c"], alpha=0.7,
                       edgecolors="none", zorder=2, label=st["label"])
        ax.legend(loc="upper right", fontsize=8, framealpha=0.9)

    sc = ax.scatter(xs, ys, c=vs, marker="^", s=size, cmap=cmap,
                    edgecolors="black", linewidths=0.4, zorder=3)

    cbar = fig.colorbar(sc, ax=ax, shrink=0.85)
    cbar.set_label(cbar_label)
    ax.set_xlabel("ix (grid column)")
    ax.set_ylabel("iy (grid row)")
    ax.set_title(title)
    ax.set_aspect("equal")
    ax.set_xlim(-0.5, nx - 0.5)
    ax.set_ylim(-0.5, ny - 0.5)
    if annotate and len(xs) <= 400:
        for x, y, v in zip(xs, ys, vs):
            ax.text(x, y - 0.25, f"{v:.2f}", ha="center", va="top",
                    fontsize=6, color="black", zorder=4)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"Wrote: {out_path}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("spice")
    ap.add_argument("--hops", type=int, default=2)
    ap.add_argument("--annotate", action="store_true",
                    help="Print value in each cell (only if grid is small).")
    ap.add_argument("--sinks", default=None,
                    help="CSV of name,x_dbu,y_dbu to overlay as blue dots.")
    args = ap.parse_args()

    buffers = parse_spice(args.spice)
    if not buffers:
        print(f"ERROR: no Vclk_* sources in {args.spice}", file=sys.stderr)
        sys.exit(1)

    arr_ns, cells, (nx, ny), (px, py), (x0, y0) = build_grid(buffers)
    skew_ps = local_skew(cells, nx, ny, args.hops)

    sinks_ig = None
    if args.sinks:
        raw = load_sinks(args.sinks)
        sinks_ig = {kind: [((x - x0) / px, (y - y0) / py) for (x, y) in pts]
                    for kind, pts in raw.items()}
        n_total = sum(len(v) for v in sinks_ig.values())
        n_mesh = len(sinks_ig.get("mesh", []))
        n_gated = len(sinks_ig.get("gated", []))
        print(f"Sinks overlay: {n_total} total ({n_mesh} mesh, "
              f"{n_gated} gated) from {args.sinks}")

    arrivals = np.array([b[3] for b in buffers])
    gskew_ps = (arrivals.max() - arrivals.min()) * 1000.0
    print(f"Buffers: {len(buffers)}   grid: {nx} x {ny}   "
          f"pitch (dbu): {px} x {py}")
    print(f"Arrival (ns)  min={arrivals.min():.4f}  "
          f"max={arrivals.max():.4f}  global skew = {gskew_ps:.3f} ps")
    print(f"Local skew (ps)  min={np.nanmin(skew_ps):.3f}  "
          f"mean={np.nanmean(skew_ps):.3f}  max={np.nanmax(skew_ps):.3f}")

    base = args.spice
    render(arr_ns,
           f"Mesh buffer arrival time (ns)\nglobal skew = {gskew_ps:.2f} ps",
           "arrival (ns)",
           base + "_arrival_heatmap.png",
           annotate=args.annotate,
           cmap="Reds",
           sinks_ig=sinks_ig)
    render(skew_ps,
           f"Local skew, NSEW hops={args.hops} (ps)",
           "local skew (ps)",
           base + "_local_skew_heatmap.png",
           annotate=args.annotate,
           cmap="magma",
           sinks_ig=sinks_ig)


if __name__ == "__main__":
    main()
