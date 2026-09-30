#!/usr/bin/env python3
# Pareto for x4 / fanout-8 only: skew vs power across mesh pitches, + FS.
import sys, os, re
sys.path.insert(0,"/home/wali2/backside/OpenROAD/src/cms/test/backside_result/Jpeg")
from lis_metrics import metrics
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

KNOWN={"pitch1p6_full":"52x52","fs_full":"26x26"}  # logs not kept for these
def gridsz(d):  # read "expected N H-lines x M V-lines" from the run log
    p=os.path.join(d,"openroad.log")
    if os.path.exists(p):
        for l in open(p,errors='ignore'):
            m=re.search(r'expected (\d+) H-lines x (\d+) V-lines',l)
            if m: return f"{m.group(1)}x{m.group(2)}"
    for k,v in KNOWN.items():
        if k in d: return v
    return "?"

B="/home/wali2/backside/OpenROAD/src/cms/test/backside_result/Jpeg"
# BS x4/fanout8 across pitch
BS=[("1.6",f"{B}/density/pitch1p6_full"),("2.4",f"{B}/density/pitch2p4_full"),
    ("3.2",f"{B}/baltree/full/x4/fmax8"),("4.8",f"{B}/density/pitch4p8"),
    ("8.0",f"{B}/density/pitch8p0"),("12",f"{B}/baltree_p12/full/x4/fmax8")]
FS=[("1.6",f"{B}/density/fs_pitch1p6"),("3.2",f"{B}/baltree/fs_full"),
    ("8.0",f"{B}/density/fs_pitch8"),("12",f"{B}/density/fs_pitch12")]

fig,ax=plt.subplots(figsize=(9,6.5))
xs=[]; ys=[]; labs=[]
for p,d in BS:
    r=metrics(d)
    if r: xs.append(r['pw']); ys.append(r['skew']); labs.append(p)
szs=[gridsz(d) for _,d in BS]
ax.plot(xs,ys,'o-',color="#1f77b4",ms=9,lw=1.8,label="Backside mesh (x4 sink, fanout 8)",zorder=4)
for x,y,l,s in zip(xs,ys,labs,szs):
    ax.annotate(f"Pitch {l}",(x,y),textcoords="offset points",xytext=(6,6),fontsize=8.5)
# FS points
fx=[]; fy=[]; fl=[]
for p,d in FS:
    r=metrics(d)
    if r: fx.append(r['pw']); fy.append(r['skew']); fl.append(p)
fsz=[gridsz(d) for _,d in FS]
ax.plot(fx,fy,'o--',color="#d62728",ms=9,lw=1.4,label="Frontside mesh (direct)",zorder=5)
for x,y,l in zip(fx,fy,fl):
    ax.annotate(f"Pitch {l}",(x,y),textcoords="offset points",xytext=(6,-13),fontsize=8.5,color="#d62728")
ax.set_xlabel("Clock power (mW)"); ax.set_ylabel("Worst-case skew (ps)")
ax.set_title("JPEG — x4 sink buffer, fanout 8: skew vs power across mesh pitch\n(backside vs frontside)",
             fontsize=12,fontweight="bold")
ax.grid(alpha=0.3); ax.legend(fontsize=9,loc="upper right")

# small pitch -> mesh-size tables (BS and FS)
bs_rows=[["Pitch","Size"]]+[[l,s] for l,s in zip(labs,szs)]
fs_rows=[["Pitch","Size"]]+[[l,gridsz(d)] for (l,d) in zip(fl,[dd for _,dd in FS if metrics(dd)])]
def minitable(rows, x, y, title, tcol):
    txt=title+"\n"+"\n".join(f"{r[0]:>5}  {r[1]:>6}" for r in rows)
    ax.text(x,y,txt,transform=ax.transAxes,fontsize=8.5,va="top",ha="left",family="monospace",
            color=tcol, bbox=dict(boxstyle="round",fc="white",ec=tcol,alpha=0.95))
minitable(bs_rows,0.60,0.84,"Backside mesh","#1f77b4")
minitable(fs_rows,0.80,0.84,"Frontside mesh","#d62728")
out=f"{B}/jpeg_pareto_x4f8.pdf"
plt.tight_layout(); plt.savefig(out,bbox_inches="tight",dpi=130); plt.savefig(out.replace('.pdf','.png'),bbox_inches="tight",dpi=130)
print("wrote",out)
