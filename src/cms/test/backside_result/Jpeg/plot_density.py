#!/usr/bin/env python3
# Skew-vs-power trade-space across the 3 mesh pitches (+ FS), from .lis.
import sys
sys.path.insert(0,"/home/wali2/backside/OpenROAD/src/cms/test/backside_result/Jpeg")
from lis_metrics import metrics
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

BASE="/home/wali2/backside/OpenROAD/src/cms/test/backside_result/Jpeg"
GRID={"x1":[8,16,20],"x2":[6,16,24,40],"x3":[8,16,24,60],"x4":[8,16,24,80],
      "x6":[8,16,24,80],"x8":[8,16,24,80],"x10":[8,16,24,80],"x12":[8,16,24,80]}
# (label, BS sweep dir, FS dir, color)
PITCHES=[
  ("Dense (1.6, 52x52)",  f"{BASE}/baltree_p1p6/full", f"{BASE}/density/fs_pitch1p6", "#d62728"),
  ("Sparse (3.2, 25x25)", f"{BASE}/baltree/full",       f"{BASE}/baltree/fs_full",     "#1f77b4"),
  ("Floor (12, 7x7)",     f"{BASE}/baltree_p12/full",   f"{BASE}/density/fs_pitch12",   "#2ca02c"),
]

fig,(ax1,ax2)=plt.subplots(1,2,figsize=(15,6.5))
for lbl,bsd,fsd,col in PITCHES:
    xs=[]; ys=[]; f8x=[]; f8y=[]
    for b,fl in GRID.items():
        for f in fl:
            r=metrics(f"{bsd}/{b}/fmax{f}")
            if not r: continue
            xs.append(r['pw']); ys.append(r['skew'])
            if f<=8: f8x.append(r['pw']); f8y.append(r['skew'])
    ax1.scatter(xs,ys,c=col,s=28,alpha=0.55,label=lbl+" BS")
    ax1.scatter(f8x,f8y,facecolors='none',edgecolors=col,s=90,linewidths=1.6)  # ring the fmax<=8
    fr=metrics(fsd)
    if fr: ax1.scatter([fr['pw']],[fr['skew']],marker='*',c=col,s=320,edgecolors='k',linewidths=0.8,
                       label=lbl+" FS", zorder=5)
ax1.set_xlabel("Clock power (mW)"); ax1.set_ylabel("Worst-case skew (ps)")
ax1.set_title("Skew vs Power — all configs (rings = low fanout ≤8, stars = FS mesh)")
ax1.set_ylim(0,120); ax1.grid(alpha=0.3); ax1.legend(fontsize=7,ncol=2)

# right panel: density U-curve at the x4/fmax8 sweet-spot config, full pitch range
PSWEEP=[("1.6",f"{BASE}/density/pitch1p6_full"),("2.4",f"{BASE}/density/pitch2p4_full"),
        ("3.2",f"{BASE}/baltree/full/x4/fmax8"),("4.8",f"{BASE}/density/pitch4p8"),
        ("8.0",f"{BASE}/density/pitch8p0"),("12",f"{BASE}/baltree_p12/full/x4/fmax8")]
p=[]; sk=[]; pw=[]
for lab,d in PSWEEP:
    r=metrics(d)
    if r: p.append(float(lab)); sk.append(r['skew']); pw.append(r['pw'])
ax2.plot(p,sk,'o-',color="#1f77b4",label="Worst skew (ps)")
ax2.set_xlabel("Mesh pitch (um)"); ax2.set_ylabel("Worst skew (ps)",color="#1f77b4")
ax2b=ax2.twinx(); ax2b.plot(p,pw,'s--',color="#d62728",label="Power (mW)")
ax2b.set_ylabel("Power (mW)",color="#d62728")
ax2.set_title("Density sweep @ x4/fanout8: skew & power vs pitch"); ax2.grid(alpha=0.3)
fig.suptitle("JPEG backside mesh — density trade-space (BS sink-buffer x fanout grid + FS reference)",
             fontsize=13,fontweight="bold")
out=f"{BASE}/jpeg_density_graph.pdf"
plt.tight_layout(rect=[0,0,1,0.96]); plt.savefig(out,bbox_inches="tight",dpi=130); plt.savefig(out.replace('.pdf','.png'),bbox_inches="tight",dpi=130)
print("wrote",out)
