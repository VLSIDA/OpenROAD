#!/usr/bin/env python3
"""Run an MC deck as parallel single-process HSPICE chunks and merge results.

Workaround for hspice -mp being broken on Ubuntu (its wrapper needs bash as
/bin/sh).  Each chunk gets a different seed; sample 1 of every chunk is
HSPICE's built-in nominal run, so merged useful samples =
1 nominal + jobs*(per_chunk-1) random.

Usage: run_mc.py <deck_mc.sp> [-n TOTAL] [-j JOBS] [--seed BASE]
Then:  mc_skew.py <deck>_c*.mt0
"""
import argparse
import re
import subprocess
import sys
from pathlib import Path


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("mc_deck", type=Path, help="deck produced by make_mc.py")
    ap.add_argument("-n", "--total", type=int, default=1000,
                    help="total random samples wanted")
    ap.add_argument("-j", "--jobs", type=int, default=8)
    ap.add_argument("--seed", type=int, default=1234)
    args = ap.parse_args()

    txt = args.mc_deck.read_text()
    if "sweep monte=" not in txt:
        sys.exit("deck has no 'sweep monte=' - run make_mc.py first")
    per = -(-args.total // args.jobs) + 1  # +1 pays for the nominal sample

    outdir = args.mc_deck.resolve().parent
    stem = args.mc_deck.stem
    procs = []
    for k in range(args.jobs):
        chunk = re.sub(r"sweep monte=\d+", f"sweep monte={per}", txt)
        chunk = re.sub(r"\bseed=\d+", f"seed={args.seed + k}", chunk)
        cdeck = outdir / f"{stem}_c{k}.sp"
        cdeck.write_text(chunk)
        procs.append(subprocess.Popen(
            ["hspice", "-i", cdeck.name, "-o", f"{stem}_c{k}"],
            cwd=outdir, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
    print(f"{args.jobs} jobs x {per} samples "
          f"(= 1 nominal + {args.jobs * (per - 1)} random) ... ", flush=True)

    fails = 0
    for k, p in enumerate(procs):
        p.wait()
        mt0 = outdir / f"{stem}_c{k}.mt0"
        if p.returncode != 0 or not mt0.exists():
            fails += 1
            print(f"chunk {k}: FAILED (see {stem}_c{k}.lis)")
    if fails:
        sys.exit(f"{fails}/{args.jobs} chunks failed")
    print(f"done. merge with:  python3 mc_skew.py {stem}_c*.mt0")


if __name__ == "__main__":
    main()
