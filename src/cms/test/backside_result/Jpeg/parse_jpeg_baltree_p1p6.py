#!/usr/bin/env python3
# Parse the JPEG backside BALANCED-FEEDER sweep (32 configs).
#   Jpeg/baltree_p1p6/<buf>/fmax<F>/{openroad.log, jpeg_backside.mt0}
# Reports worst-case skew (max-min), insertion, power, feeder skew, ok count.
import os, re, statistics, sys
VARIANT = sys.argv[1] if len(sys.argv) > 1 else "full"
JDIR = "/home/wali2/backside/OpenROAD/src/cms/test/backside_result/Jpeg/baltree_p1p6/" + VARIANT
# per-buffer F_max grid (max F_max capped to each buffer's drive budget)
FMAX_FOR = {
    "x1": [8, 16, 20], "x2": [6, 16, 24, 40], "x3": [8, 16, 24, 60],
    "x4": [8, 16, 24, 80], "x6": [8, 16, 24, 80], "x8": [8, 16, 24, 80],
    "x10": [8, 16, 24, 80], "x12": [8, 16, 24, 80],
}
BUFS = list(FMAX_FOR.keys())

def parse_mt0(path):
    if not os.path.exists(path):
        return None
    toks = []
    for l in open(path):
        if l.startswith('$') or l.strip().startswith('.TITLE'):
            continue
        toks += l.split()
    names = [t for t in toks if re.match(r'[A-Za-z]', t)]
    vals = [t for t in toks if not re.match(r'[A-Za-z]', t)]
    d = dict(zip(names, vals))
    s = []
    for k, v in d.items():
        if re.match(r't_sink_\d+$', k) and re.match(r'[-0-9.eE+]+$', v):
            s.append(float(v))
    good = [v for v in s if 0 < v < 1e6]
    if not good:
        return None
    return dict(n=len(s), ok=len(good),
                skew=(max(good) - min(good)) * 1e12,
                ins=statistics.mean(good) * 1e12,
                pw=float(d.get('avg_power', 0)) * 1e3)

def feeder_skew(log):
    if not os.path.exists(log):
        return None
    a = []
    for l in open(log, errors='ignore'):
        if 'CMS-0873' in l:
            m = re.search(r"':\s*([0-9.eE+-]+)\s*ns", l)
            if m:
                a.append(float(m.group(1)) * 1e3)
    return (max(a) - min(a)) if a else None

print(f"{'buf':>4} {'fmax':>5} | {'feeder(ps)':>10} {'skew(ps)':>9} {'ins(ps)':>8} {'power(mW)':>9} {'ok':>10}")
print("-" * 68)
rows = []
for b in BUFS:
    for f in FMAX_FOR[b]:
        d = os.path.join(JDIR, b, f"fmax{f}")
        r = parse_mt0(os.path.join(d, "jpeg_backside.mt0"))
        fs = feeder_skew(os.path.join(d, "openroad.log"))
        if r:
            rows.append((b, f, fs, r))
            fsv = f"{fs:.1f}" if fs is not None else "  -"
            print(f"{b:>4} {f:>5} | {fsv:>10} {r['skew']:9.1f} {r['ins']:8.1f} {r['pw']:9.3f} {r['ok']:>5}/{r['n']}")
        else:
            print(f"{b:>4} {f:>5} | {'(no result)':>10}")
print("\nRef: frontside mesh 30.4ps/4.90mW; x4-only feeder 15.8ps/6.07mW; CTS tree ~4-14ps/2.5mW")
