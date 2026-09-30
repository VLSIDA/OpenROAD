#!/usr/bin/env python3
# Single graph: scaled-driver backside vs frontside, skew (X) vs power (Y).
# Pitch-scaled mesh driver. P4/P6/P10 dropped, line by pitch order.
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

PTS=[("1.6","x2","pitch1p6"),("2.5","x3","pitch2p5"),("8","x6","pitch8"),
     ("12","x10","pitch12"),("16","x12","pitch16")]
def collect(prefix):
    d=[]
    for p,drv,tag in PTS:
        dd=f"{I}/density/{prefix}{tag}"; r=metrics(dd)
        if r and r["n"]: d.append((float(p),p,drv,r['pw'],r['skew'],gridsz(dd)))
    return sorted(d,key=lambda q:q[0])
bs=collect("scaled_"); fs=collect("fs_scaled_")

fig,ax=plt.subplots(figsize=(10,6.6))
def draw(data,color,dark,label,dy):
    xs=[q[4] for q in data]; ys=[q[3] for q in data]
    ax.plot(xs,ys,'-',color=color,lw=1.6,zorder=3)
    ax.scatter(xs,ys,color=color,s=95,marker="D",edgecolors="k",linewidths=0.5,label=label,zorder=4)
    for _,p,drv,pw,sk,sz in data:
        ax.annotate(f"P{p}/{drv}",(sk,pw),textcoords="offset points",xytext=(7,dy),fontsize=8.5,color=dark)
draw(bs,"#2ca02c","#1b5e20","Backside mesh (sink x4/fanout8)",5)
draw(fs,"#d62728","#8b0000","Frontside mesh (direct-to-mesh)",-13)

ax.set_xlabel("Worst-case skew, max-min (ps)"); ax.set_ylabel("Clock power (mW)")
ax.set_title("Ibex - Skew vs Power",fontsize=13,fontweight="bold")
ax.grid(alpha=0.3); ax.legend(fontsize=9.5,loc="upper right")

# pitch / driver / mesh-size tables on the right, under the legend
def minitable(rows,x,y,title,tcol):
    txt=title+"\n"+"\n".join(f"{a:>4}{b:>5}{c:>7}" for a,b,c in rows)
    ax.text(x,y,txt,transform=ax.transAxes,fontsize=8,va="top",ha="left",family="monospace",
            color=tcol,bbox=dict(boxstyle="round",fc="white",ec=tcol,alpha=0.95))
bs_rows=[["Ptch","Drv","Size"]]+[[q[1],q[2],q[5]] for q in bs]
fs_rows=[["Ptch","Drv","Size"]]+[[q[1],q[2],q[5]] for q in fs]
minitable(bs_rows,0.66,0.82,"BS mesh","#2ca02c")
minitable(fs_rows,0.83,0.82,"FS mesh","#d62728")
out=f"{I}/ibex_scaled_combined.pdf"
plt.tight_layout(); plt.savefig(out,bbox_inches="tight",dpi=130); plt.savefig(out.replace('.pdf','.png'),bbox_inches="tight",dpi=130)
print("wrote",out)
