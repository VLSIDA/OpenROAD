#!/usr/bin/env python3
"""
Local-neighborhood skew of mesh buffer input delays.

Reads a mesh SPICE file emitted by write_mesh_spice and, for each mesh
buffer, computes the skew across the NSEW stencil:

    self,  N1, N2,  S1, S2,  E1, E2,  W1, W2   (9 points, hops=2)

Source of input delays is the per-buffer `Vclk_*` line, e.g.

    Vclk_5 clk_in_mesh_buf_54924_37404_A 0 PULSE(0 0.7 0.155414n ...) \\
       $ mesh_buf_54924_37404/A arrival=0.155414ns

Buffer (x, y) in DBU is taken from the instance name `mesh_buf_<x>_<y>`,
which matches the grid intersection it was placed at — so binning is
exact (no rounding off origin/getOrigin).

Output: CSV with one row per anchor buffer. Missing neighbors written
as NA. Skew is max-min across (self + valid neighbors), in ps.

Usage:
    python3 mesh_skew_local.py <mesh.spice> [-o out.csv] [--hops 2]
"""

import argparse
import csv
import re
import sys
from collections import defaultdict

VCLK_RE = re.compile(
    r"^Vclk_\d+\s+\S+\s+0\s+PULSE\([^)]*\)\s*\$\s*"
    r"(mesh_buf_(\d+)_(\d+))/\S+\s+arrival=([\d.eE+\-]+)ns"
)


def parse_spice(path):
    """Return list of (buf_name, x_dbu, y_dbu, arrival_ns)."""
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
            x = int(m.group(2))
            y = int(m.group(3))
            t = float(m.group(4))
            if name in seen:
                continue
            seen.add(name)
            buffers.append((name, x, y, t))
    return buffers


def infer_pitch(values):
    """Pitch = smallest positive diff between sorted unique values.

    Robust against missing intersections (blockage holes) — the
    spacing between *adjacent* present coords is still the true pitch
    as long as at least one pair of adjacent grid lines both have a
    buffer."""
    uniq = sorted(set(values))
    if len(uniq) < 2:
        return 1
    diffs = [b - a for a, b in zip(uniq[:-1], uniq[1:]) if b > a]
    return min(diffs)


def build_index(buffers):
    """Map (ix, iy) -> (name, x, y, arrival_ns), with ix/iy from pitch."""
    xs = [b[1] for b in buffers]
    ys = [b[2] for b in buffers]
    px = infer_pitch(xs)
    py = infer_pitch(ys)
    x_min = min(xs)
    y_min = min(ys)

    grid = {}
    for name, x, y, t in buffers:
        ix = round((x - x_min) / px)
        iy = round((y - y_min) / py)
        grid[(ix, iy)] = (name, x, y, t)
    return grid, px, py, x_min, y_min


def neighbors(grid, ix, iy, hops):
    """Return list of (offset_label, arrival_ns or None) for the
    NSEW stencil out to `hops` in each direction."""
    out = []
    for h in range(1, hops + 1):
        out.append((f"N{h}", grid.get((ix, iy + h))))
        out.append((f"S{h}", grid.get((ix, iy - h))))
        out.append((f"E{h}", grid.get((ix + h, iy))))
        out.append((f"W{h}", grid.get((ix - h, iy))))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("spice", help="mesh SPICE file (with Vclk_* sources)")
    ap.add_argument("-o", "--output", default=None,
                    help="output CSV (default: <spice>_local_skew.csv)")
    ap.add_argument("--hops", type=int, default=2,
                    help="NSEW reach in grid steps (default: 2)")
    args = ap.parse_args()

    buffers = parse_spice(args.spice)
    if not buffers:
        print(f"ERROR: no Vclk_* sources found in {args.spice}", file=sys.stderr)
        sys.exit(1)

    grid, px, py, x_min, y_min = build_index(buffers)
    print(f"Parsed {len(buffers)} mesh buffer input delays.")
    print(f"Inferred pitch: X={px} dbu  Y={py} dbu")
    print(f"Grid span: X=[{x_min}, {max(b[1] for b in buffers)}]  "
          f"Y=[{y_min}, {max(b[2] for b in buffers)}]")

    out_path = args.output or (args.spice + "_local_skew.csv")

    header = ["buf_name", "x_dbu", "y_dbu", "ix", "iy", "t_self_ns"]
    for h in range(1, args.hops + 1):
        header += [f"t_N{h}_ns", f"t_S{h}_ns", f"t_E{h}_ns", f"t_W{h}_ns"]
    header += ["n_valid_neighbors", "local_skew_ps"]

    rows = []
    all_skews = []
    worst = None  # (skew_ps, name)
    for (ix, iy), (name, x, y, t) in grid.items():
        nbrs = neighbors(grid, ix, iy, args.hops)
        arrivals = [t]
        nbr_cells = {}
        for label, entry in nbrs:
            if entry is None:
                nbr_cells[label] = "NA"
            else:
                nbr_cells[label] = f"{entry[3]:.6f}"
                arrivals.append(entry[3])
        skew_ps = (max(arrivals) - min(arrivals)) * 1000.0
        all_skews.append(skew_ps)
        if worst is None or skew_ps > worst[0]:
            worst = (skew_ps, name, ix, iy)
        row = [name, x, y, ix, iy, f"{t:.6f}"]
        for h in range(1, args.hops + 1):
            for d in ("N", "S", "E", "W"):
                row.append(nbr_cells[f"{d}{h}"])
        row.append(len(arrivals) - 1)
        row.append(f"{skew_ps:.4f}")
        rows.append(row)

    rows.sort(key=lambda r: (r[4], r[3]))  # iy, ix

    with open(out_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)

    n = len(all_skews)
    mean_skew = sum(all_skews) / n
    max_skew = max(all_skews)
    min_skew = min(all_skews)

    global_skew_ps = (
        max(b[3] for b in buffers) - min(b[3] for b in buffers)
    ) * 1000.0

    # Interior-only stats (anchors with the full 4*hops neighbors present).
    full_count = 4 * args.hops
    interior = [
        float(r[-1]) for r in rows if int(r[-2]) == full_count
    ]
    if interior:
        int_min = min(interior)
        int_mean = sum(interior) / len(interior)
        int_max = max(interior)
    else:
        int_min = int_mean = int_max = float("nan")

    report_path = out_path.rsplit(".csv", 1)[0] + ".report.txt"
    with open(report_path, "w") as f:
        f.write("Mesh buffer input-delay local skew report\n")
        f.write("=========================================\n")
        f.write(f"Source SPICE      : {args.spice}\n")
        f.write(f"CSV               : {out_path}\n")
        f.write(f"Hops (NSEW reach) : {args.hops} (stencil = self + "
                f"{full_count} neighbors when interior)\n")
        f.write(f"Buffers parsed    : {len(buffers)}\n")
        f.write(f"Pitch X (dbu)     : {px}\n")
        f.write(f"Pitch Y (dbu)     : {py}\n")
        f.write(f"X span (dbu)      : [{x_min}, {max(b[1] for b in buffers)}]\n")
        f.write(f"Y span (dbu)      : [{y_min}, {max(b[2] for b in buffers)}]\n")
        f.write("\n")
        f.write("Local skew over (self + NSEW neighbors), ps:\n")
        f.write(f"  all anchors      ({n:>5d})  "
                f"min {min_skew:8.3f}  mean {mean_skew:8.3f}  "
                f"max {max_skew:8.3f}\n")
        f.write(f"  interior anchors ({len(interior):>5d})  "
                f"min {int_min:8.3f}  mean {int_mean:8.3f}  "
                f"max {int_max:8.3f}\n")
        f.write("\n")
        f.write(f"Worst anchor      : {worst[1]} "
                f"(ix={worst[2]}, iy={worst[3]}) -> {worst[0]:.3f} ps\n")
        f.write(f"Global skew (max-min across all buffers): "
                f"{global_skew_ps:.3f} ps\n")

    print()
    print(f"Wrote: {out_path}")
    print(f"Wrote: {report_path}")
    print(f"Anchors: {n}  (interior with full {full_count}-neighbor stencil: "
          f"{len(interior)})")
    print(f"Local skew (ps) all      -- min: {min_skew:.3f}  "
          f"mean: {mean_skew:.3f}  max: {max_skew:.3f}")
    print(f"Local skew (ps) interior -- min: {int_min:.3f}  "
          f"mean: {int_mean:.3f}  max: {int_max:.3f}")
    print(f"Worst-anchor: {worst[1]} (ix={worst[2]}, iy={worst[3]}) "
          f"-> {worst[0]:.3f} ps")
    print(f"Global mesh input-delay skew: {global_skew_ps:.3f} ps")


if __name__ == "__main__":
    main()
