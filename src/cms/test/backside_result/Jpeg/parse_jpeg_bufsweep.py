#!/usr/bin/env python3
# Parse the JPEG sink-buffer x F_max sweep into 2D skew + power grids.
# x4 column is pulled from the existing Jpeg/fmax<F>/ runs (baseline).
import os, re, statistics
JDIR = "/home/wali2/backside/OpenROAD/src/cms/test/backside_result/Jpeg"
BUFS  = ["x4", "x6", "x8", "x10", "x12"]
FMAXS = [8, 16, 24, 80]

def deck_path(buf, f):
    # every buffer (incl. x4) now runs into Jpeg/<buf>/fmax<F>/ with the new code
    return os.path.join(JDIR, buf, f"fmax{f}", "jpeg_backside.mt0")

def parse(path):
    if not os.path.exists(path):
        return None
    toks = []
    for l in open(path):
        if l.startswith('$DATA') or l.strip().startswith('.TITLE'):
            continue
        toks += l.split()
    if 'alter#' not in toks:
        return None
    i = toks.index('alter#')
    d = dict(zip(toks[:i+1], toks[i+1:]))
    sink = []
    for k, v in d.items():
        if re.match(r't_sink_\d+$', k) and v != 'failed':
            try: sink.append(float(v))
            except: pass
    if not sink or 'avg_power' not in d:
        return None
    ss = sorted(sink)
    p1 = ss[int(0.01*len(ss))]; p99 = ss[int(0.99*len(ss))]
    return dict(n=len(sink),
                skew=round((p99-p1)*1e12, 1),
                pw=round(float(d['avg_power'])*1e3, 3))

def grid(metric, label, unit):
    print(f"\n=== {label} ({unit}) — rows=sink buffer, cols=F_max ===")
    print("buf   " + "".join(f"{'fmax'+str(f):>10}" for f in FMAXS))
    for b in BUFS:
        cells = []
        for f in FMAXS:
            r = parse(deck_path(b, f))
            cells.append("   --" if r is None else f"{r[metric]:>10}")
        print(f"{b:<6}" + "".join(cells))

grid('skew', 'SKEW P1-P99', 'ps')
grid('pw',   'POWER',       'mW')

# note incomplete/dropped-sink decks (n < full FF count if known)
print("\n(cells '--' = not run or failed; check that sink counts are full 4384)")
for b in BUFS:
    for f in FMAXS:
        r = parse(deck_path(b, f))
        if r and r['n'] < 4384:
            print(f"  WARNING {b}/fmax{f}: only {r['n']}/4384 sinks connected (dropped)")
