#!/usr/bin/env python3
# Clean fanout-8 Pareto: skew vs power, one point per sink buffer per pitch + FS.
import sys
sys.path.insert(0,"/home/wali2/backside/OpenROAD/src/cms/test/backside_result/Jpeg")
from lis_metrics import metrics
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

BASE="/home/wali2/backside/OpenROAD/src/cms/test/backside_result/Jpeg"
BUFS=["x1","x2","x3","x4","x6","x8","x10","x12"]
LOWF={"x2":6}   # x2 has no fanout-8; use its lowest (6). all others -> 8
PITCHES=[
  ("Dense  (pitch 1.6)",  f"{BASE}/baltree_p1p6/full", f"{BASE}/density/fs_pitch1p6", "#d62728","o"),
  ("Sparse (pitch 3.2)",  f"{BASE}/baltree/full",       f"{BASE}/baltree/fs_full",     "#1f77b4","s"),
  ("Floor  (pitch 12)",   f"{BASE}/baltree_p12/full",   f"{BASE}/density/fs_pitch12",   "#2ca02c","^"),
]

fig,ax=plt.subplots(figsize=(9,6.5))
for lbl,bsd,fsd,col,mk in PITCHES:
    xs=[]; ys=[]; labs=[]
    for b in BUFS:
        f=LOWF.get(b,8)
        r=metrics(f"{bsd}/{b}/fmax{f}")
        if not r: continue
        xs.append(r['pw']); ys.append(r['skew']); labs.append(b)
    # sort by power for a clean connecting line
    order=sorted(range(len(xs)),key=lambda i:xs[i])
    xs=[xs[i] for i in order]; ys=[ys[i] for i in order]; labs=[labs[i] for i in order]
    ax.plot(xs,ys,'-',color=col,alpha=0.35,lw=1.2)
    ax.scatter(xs,ys,c=col,marker=mk,s=70,label=lbl+" (BS, sink x1..x12)",zorder=4,edgecolors='k',linewidths=0.5)
    # label buffer sizes on the sparse line only (least cluttered)
    if "Sparse" in lbl:
        for x,y,l in zip(xs,ys,labs): ax.annotate(l,(x,y),textcoords="offset points",xytext=(4,4),fontsize=8)
    fr=metrics(fsd)
    if fr: ax.scatter([fr['pw']],[fr['skew']],marker='*',c=col,s=340,edgecolors='k',linewidths=0.9,
                      label=lbl+" (FS mesh)",zorder=6)
ax.set_xlabel("Clock power (mW)"); ax.set_ylabel("Worst-case skew (ps)")
ax.set_title("JPEG backside mesh — fanout-8 Pareto (skew vs power)\n"
             "circles/squares/triangles = BS sink buffers x1..x12 · stars = frontside mesh",
             fontsize=12,fontweight="bold")
ax.grid(alpha=0.3); ax.legend(fontsize=8,loc="upper right")
out=f"{BASE}/jpeg_pareto_f8.pdf"
plt.tight_layout(); plt.savefig(out,bbox_inches="tight",dpi=130); plt.savefig(out.replace('.pdf','.png'),bbox_inches="tight",dpi=130)
print("wrote",out)
