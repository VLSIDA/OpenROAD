#!/usr/bin/env python3
# Backside-only Pareto: skew vs power across mesh pitch.
# Two series: fixed x4 mesh driver vs pitch-scaled mesh driver. Sink x4/fanout8.
import sys, os, re
sys.path.insert(0,"/home/wali2/backside/OpenROAD/src/cms/test/backside_result/Jpeg")
from lis_metrics import metrics
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

I="/home/wali2/backside/OpenROAD/src/cms/test/backside_result/Ibex"
def gridsz(d):
    p=os.path.join(d,"openroad.log")
    if os.path.exists(p):
        for l in open(p,errors='ignore'):
            m=re.search(r'expected (\d+) H-lines x (\d+) V-lines',l)
            if m: return f"{m.group(1)}x{m.group(2)}"
    return "?"

# curated well-spaced pitches (one per grid size)
PITCHES=[("1.6","pitch1p6"),("2.5","pitch2p5"),("4","pitch4"),("6","pitch6"),
         ("8","pitch8"),("10","pitch10"),("12","pitch12"),("16","pitch16")]
DRV={"1.6":"x2","2.5":"x3","4":"x4","6":"x4","8":"x6","10":"x10","12":"x10","16":"x12"}
def fixdir(tag): return f"{I}/baltree_p1p6/full/x4/fmax8" if tag=="pitch1p6" else f"{I}/density/{tag}"
def scaldir(tag): return f"{I}/density/scaled_{tag}"

fig,ax=plt.subplots(figsize=(9,6.5))
def series(dirfn,color,marker,label,dy,drvlbl=False):
    pts=[]
    for p,tag in PITCHES:
        d=dirfn(tag); r=metrics(d)
        if not r or not r["n"]: continue
        pts.append((float(p),p,r['pw'],r['skew']))
    pts.sort()
    xs=[q[2] for q in pts]; ys=[q[3] for q in pts]
    ax.scatter(xs,ys,color=color,s=72,marker=marker,edgecolors="k",linewidths=0.4,label=label,zorder=4)
    for _,p,x,y in pts:
        lab=f"P{p}"+(f"/{DRV[p]}" if drvlbl else "")
        ax.annotate(lab,(x,y),textcoords="offset points",xytext=(5,dy),fontsize=8,color=color)
    return pts

fx=series(fixdir,"#1f77b4","o","Fixed x4 mesh driver",6)
sc=series(scaldir,"#2ca02c","D","Pitch-scaled mesh driver",-13,drvlbl=True)

ax.set_xlabel("Clock power (mW)"); ax.set_ylabel("Worst-case skew, max-min (ps)")
ax.set_title("Ibex backside mesh — skew vs power across pitch\n"
             "fixed x4 driver vs pitch-scaled driver (sink x4/fanout8, full feeder)",
             fontsize=12,fontweight="bold")
ax.grid(alpha=0.3); ax.legend(fontsize=9,loc="upper right")
out=f"{I}/ibex_bs_pareto.pdf"
plt.tight_layout(); plt.savefig(out,bbox_inches="tight",dpi=130); plt.savefig(out.replace('.pdf','.png'),bbox_inches="tight",dpi=130)
print("wrote",out)
print(f"{'pitch':6s}{'driver':7s}{'FIXED skew/pw':22s}{'SCALED skew/pw'}")
for p,tag in PITCHES:
    rf=metrics(fixdir(tag)); rs=metrics(scaldir(tag))
    a=f"{rf['skew']:.1f}ps/{rf['pw']:.2f}mW" if rf else "--"
    b=f"{rs['skew']:.1f}ps/{rs['pw']:.2f}mW" if rs else "--"
    print(f"P{p:<5}{DRV[p]:7s}{a:22s}{b}")
