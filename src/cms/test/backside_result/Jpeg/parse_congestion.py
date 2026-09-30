#!/usr/bin/env python3
# Compare routing congestion across clock styles (BS mesh / FS mesh / CTS).
# Reads each run's openroad.log: total WL (GRT-0018), per-layer usage% +
# total congestion (GRT-0096 table), and detailed-route DRC (DRT-0199).
import os, re, sys
BASE="/home/wali2/backside/OpenROAD/src/cms/test/backside_result/Jpeg/routing_comparison"
# (label, log path) — the 3 clock-style scenario runs
RUNS=[
  ("BS mesh",  f"{BASE}/BS_mesh/openroad.log"),
  ("FS mesh",  f"{BASE}/FS_mesh/openroad.log"),
  ("CTS tree", f"{BASE}/CTS_tree/openroad.log"),
]

def parse(log):
    if not os.path.exists(log): return None
    wl=None; drc=None; layers={}
    lines=open(log,errors='ignore').read().splitlines()
    for i,l in enumerate(lines):
        m=re.search(r'GRT-0018.*wirelength:\s*([0-9]+)',l)
        if m: wl=int(m.group(1))
        m=re.search(r'DRT-0199.*violations\s*=\s*([0-9]+)',l)
        if m: drc=int(m.group(1))
        # GRT-0096 per-layer table follows the header line
        if 'GRT-0096' in l or ('Layer' in l and 'Resource' in l and 'Usage' in l):
            for l2 in lines[i+1:i+40]:
                mm=re.match(r'\s*(M\d+|BM\d+|BPR)\s+([0-9]+)\s+([0-9]+)\s+([0-9.]+)%?\s+([0-9.]+)\s*/\s*([0-9.]+)\s*/\s*([0-9.]+)', l2)
                if mm:
                    layers[mm.group(1)]={'usage':float(mm.group(4)),'cong':float(mm.group(7))}
                elif re.match(r'\s*(GRT-|Took|\[INFO)', l2) and layers:
                    break
    return dict(wl=wl, drc=drc, layers=layers)

print(f"{'clock':>9} | {'totalWL(um)':>11} {'M5 use%':>8} {'M6 use%':>8} {'maxCong':>8} {'DRC':>7}")
print("-"*60)
for name,log in RUNS:
    r=parse(log)
    if not r: print(f"{name:>9} | (log missing: {log})"); continue
    m5=r['layers'].get('M5',{}).get('usage'); m6=r['layers'].get('M6',{}).get('usage')
    maxc=max((v['cong'] for v in r['layers'].values()), default=None)
    print(f"{name:>9} | {str(r['wl']):>11} {('%.1f'%m5) if m5 is not None else '-':>8} "
          f"{('%.1f'%m6) if m6 is not None else '-':>8} {('%.1f'%maxc) if maxc is not None else '-':>8} {str(r['drc']):>7}")
print("\nBS mesh (clock on backside) should show LOWEST M5/M6 usage + total WL")
print("=> quantifies the frontside routing relief. FS mesh = highest M5/M6 (clock fills them).")
