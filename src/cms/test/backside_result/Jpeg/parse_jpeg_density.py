#!/usr/bin/env python3
# Parse the JPEG backside density (pitch) sweep.
import os, re, glob, statistics
JDIR="/home/wali2/backside/OpenROAD/src/cms/test/backside_result/Jpeg/density"

def parse_mt0(mt):
    if not os.path.exists(mt): return None
    toks=[]
    for l in open(mt):
        if l.startswith('$') or l.strip().startswith('.TITLE'): continue
        toks+=l.split()
    d=dict(zip([t for t in toks if re.match(r'[A-Za-z]',t)],[t for t in toks if not re.match(r'[A-Za-z]',t)]))
    s=[float(v) for k,v in d.items() if re.match(r't_sink_\d+$',k) and re.match(r'[-0-9.eE+]+$',v)]
    g=[v for v in s if 0<v<1e6]
    if not g: return None
    return (max(g)-min(g))*1e12, statistics.mean(g)*1e12, float(d.get('avg_power',0))*1e3, len(g), len(s)

def logval(log, pat, grp=1):
    if not os.path.exists(log): return None
    v=None
    for l in open(log, errors='ignore'):
        m=re.search(pat,l)
        if m: v=m.group(grp)
    return v

print(f"{'pitch':>6} | {'grid':>9} {'drivers':>7} {'taps':>5} | {'skew(ps)':>9} {'ins(ps)':>8} {'power(mW)':>9} {'ok':>10} {'status':>8}")
print("-"*82)
for d in sorted(glob.glob(f"{JDIR}/pitch*"), key=lambda x:-len(x)):
    log=f"{d}/openroad.log"
    pitch=logval(log, r"pitch=([0-9.]+)")
    grid=logval(log, r"expected (\d+ H-lines x \d+ V-lines)")
    drv=logval(log, r"Created (\d+) proxy")
    taps=logval(log, r"placed (\d+) sink-taps")
    failed = os.path.exists(log) and any(re.search(r"DRT-0206|checkConnectivity break",l) for l in open(log,errors='ignore'))
    r=parse_mt0(f"{d}/jpeg_backside.mt0")
    gg = (grid or "?").replace(" H-lines x "," x ").replace(" V-lines","")
    if r:
        sk,ins,pw,ok,n=r
        st="OK"
        print(f"{pitch or '?':>6} | {gg:>9} {drv or '?':>7} {taps or '?':>5} | {sk:9.1f} {ins:8.0f} {pw:9.3f} {ok:>5}/{n} {st:>8}")
    else:
        st="ROUTE-FAIL" if failed else "no-mt0"
        print(f"{pitch or '?':>6} | {gg:>9} {drv or '?':>7} {taps or '?':>5} | {'-':>9} {'-':>8} {'-':>9} {'-':>10} {st:>8}")
print("\nDenser = lower pitch. Watch: drivers/power rise ~1/pitch^2; skew should fall then flatten (~intrinsic 4.9ps); ROUTE-FAIL/dropped-wires = the density limit.")
