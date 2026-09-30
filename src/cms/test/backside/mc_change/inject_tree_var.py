#!/usr/bin/env python3
"""Inject Khan-style uniform tree variation into a MESH-ONLY MC deck.

Adds, per mesh-buffer clock source (Vclk_* PULSE):
  arrival += aunif(0, 25p)   independent per source  (tree delay +/-25 ps)
  slew    += aunif(0, 10p)   independent per source  (tree slew  +/-10 ps)

Run make_mc.py FIRST (device/wire variation + vddval), then this script.
Only valid for mesh-only decks (per-buffer PULSE sources); full-network
decks simulate the tree and must NOT get this injection.

Usage: inject_tree_var.py <deck_mc.sp> [-o out.sp]
"""
import argparse
import re
import sys
from pathlib import Path

DLY = "25p"
SLW = "10p"

ap = argparse.ArgumentParser()
ap.add_argument("deck", type=Path)
ap.add_argument("-o", "--out", type=Path)
args = ap.parse_args()
out = args.out or args.deck.with_name(args.deck.stem + "_inj.sp")

pulse_re = re.compile(
    r"^(Vclk\S*\s+\S+\s+\S+\s+PULSE\(0 'vddval' )([0-9.e-]+)n ([0-9.e-]+)n ([0-9.e-]+)n (.*)$")

lines, params, k = [], [], 0
min_slew = 1e9
for line in args.deck.read_text().splitlines():
    m = pulse_re.match(line)
    if m:
        head, dly, rise, fall, tail = m.groups()
        params += [f".param ad{k}=aunif(0,{DLY})", f".param as{k}=aunif(0,{SLW})"]
        line = (f"{head}'{dly}n+ad{k}' 'max(0.5p,{rise}n+as{k})' "
                f"'max(0.5p,{fall}n+as{k})' {tail}")
        min_slew = min(min_slew, float(rise))
        k += 1
    lines.append(line)
    if line.strip().startswith(".option modmonte"):
        insert_at = len(lines)

if k == 0:
    sys.exit("no vddval PULSE sources found - run make_mc.py on the deck first")
lines[insert_at:insert_at] = params
out.write_text("\n".join(lines) + "\n")
print(f"injected {k} sources (arrival +/-{DLY}, slew +/-{SLW}); "
      f"min nominal slew {min_slew:.4f} ns -> {out}")
