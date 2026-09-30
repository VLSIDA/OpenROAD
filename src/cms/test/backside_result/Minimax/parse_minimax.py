#!/usr/bin/env python3
# Parse the IBEX sweep .mt0 files and print a comparison table.
# Sink count is auto-detected (scans all t_sink_* measures), so this works
# for any design regardless of FF count.
import os, re, statistics
IDIR = "/home/wali2/backside/OpenROAD/src/cms/test/backside_result/Minimax"
CONFIGS = [
    ("Frontside", "Frontside/minimax_frontside.mt0"),
    ("fmax8",     "fmax8/minimax_backside.mt0"),
    ("fmax10",    "fmax10/minimax_backside.mt0"),
    ("fmax16",    "fmax16/minimax_backside.mt0"),
    ("fmax24",    "fmax24/minimax_backside.mt0"),
    ("fmax30",    "fmax30/minimax_backside.mt0"),
    ("fmax80",    "fmax80/minimax_backside.mt0"),
]

def sink_tsv_count(tag):
    # sink TSVs = "placed N sink-taps" from CMS-0703 in this config's openroad.log
    # (each sink tap = 1 sink buffer + 1 sink TSV). Frontside has no TSVs -> 0.
    log = os.path.join(IDIR, tag, "openroad.log")
    if not os.path.exists(log):
        return None
    n = None
    for l in open(log, errors='ignore'):
        m = re.search(r'placed\s+(\d+)\s+sink-taps', l)
        if m:
            n = int(m.group(1))
    return n if n is not None else 0

def parse(path):
    toks = []
    for l in open(path):
        if l.startswith('$DATA') or l.strip().startswith('.TITLE'):
            continue
        toks += l.split()
    i = toks.index('alter#')
    d = dict(zip(toks[:i+1], toks[i+1:]))
    sink = []; fail = 0
    for k, v in d.items():
        if not re.match(r't_sink_\d+$', k):
            continue
        if v == 'failed':
            fail += 1; continue
        try: sink.append(float(v))
        except: fail += 1
    ss = sorted(sink)
    p1 = ss[int(0.01*len(ss))]; p99 = ss[int(0.99*len(ss))]
    return dict(
        ok=len(sink), fail=fail,
        skew=round((p99-p1)*1e12, 1),
        sig=round(statistics.pstdev(sink)*1e12, 2),
        ins=round(statistics.mean(sink)*1e12, 1),
        pw=round(float(d['avg_power'])*1e3, 3),
    )

print(f"{'config':<10} {'sinkTSV':>7} {'sinks':>6} {'skew(ps)':>9} {'sigma':>7} {'insert(ps)':>11} {'power(mW)':>10}")
print("-"*66)
fs = None; rows = {}
for tag, rel in CONFIGS:
    p = os.path.join(IDIR, rel)
    if not os.path.exists(p):
        print(f"{tag:<10} {'-- not run --':>50}"); continue
    r = parse(p); rows[tag] = r
    tsv = sink_tsv_count(tag)
    tsv_s = "-" if tsv is None else str(tsv)
    if tag == "Frontside": fs = r
    print(f"{tag:<10} {tsv_s:>7} {r['ok']:>6} {r['skew']:>9} {r['sig']:>7} {r['ins']:>11} {r['pw']:>10}")

if fs:
    print("\nvs Frontside (skew / power):")
    for tag, _ in CONFIGS:
        if tag == "Frontside" or tag not in rows: continue
        r = rows[tag]
        ds = (r['skew']-fs['skew'])/fs['skew']*100
        dp = (r['pw']-fs['pw'])/fs['pw']*100
        print(f"  {tag:<8} skew {ds:+6.1f}%   power {dp:+6.1f}%")
