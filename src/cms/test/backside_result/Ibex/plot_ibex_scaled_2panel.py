#!/usr/bin/env python3
# Two-panel scaled-driver comparison: backside vs frontside, SHARED skew (X) axis.
# X=skew, Y=power. Pitch-scaled mesh driver. P4/P6/P10 dropped, line by pitch.
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

# (pitch, driver, tag) -- P4/P6/P10 dropped
PTS=[("1.6","x2","pitch1p6"),("2.5","x3","pitch2p5"),("8","x6","pitch8"),
     ("12","x10","pitch12"),("16","x12","pitch16")]

def collect(prefix):
    d=[]
    for p,drv,tag in PTS:
        dd=f"{I}/density/{prefix}{tag}"; r=metrics(dd)
        if r and r["n"]: d.append((float(p),p,drv,r['pw'],r['skew'],gridsz(dd)))
    return sorted(d,key=lambda q:q[0])

bs=collect("scaled_"); fs=collect("fs_scaled_")
xmax=max([q[4] for q in bs]+[q[4] for q in fs])*1.08

fig,axes=plt.subplots(1,2,figsize=(15,6.6),sharex=True,sharey=True)
def panel(ax,data,color,dark,title):
    xs=[q[4] for q in data]; ys=[q[3] for q in data]
    ax.plot(xs,ys,'-',color=color,lw=1.6,zorder=3)
    ax.scatter(xs,ys,color=color,s=90,marker="D",edgecolors="k",linewidths=0.5,zorder=4)
    for _,p,drv,pw,sk,sz in data:
        ax.annotate(f"P{p}/{drv}",(sk,pw),textcoords="offset points",xytext=(7,5),fontsize=8.5,color=dark)
    ax.set_xlabel("Worst-case skew, max-min (ps)"); ax.set_title(title,fontweight="bold",fontsize=12)
    ax.grid(alpha=0.3)
    rows=[["Pitch","Drv","Size"]]+[[p,drv,sz] for _,p,drv,pw,sk,sz in data]
    ax.text(0.985,0.97,"\n".join(f"{r[0]:>5}{r[1]:>5}{r[2]:>7}" for r in rows),
            transform=ax.transAxes,fontsize=8.5,va="top",ha="right",family="monospace",
            color=dark,bbox=dict(boxstyle="round",fc="white",ec=color,alpha=0.95))
panel(axes[0],bs,"#2ca02c","#1b5e20","(a) Backside mesh (sink x4/fanout8)")
panel(axes[1],fs,"#d62728","#8b0000","(b) Frontside mesh (direct-to-mesh)")
axes[0].set_ylabel("Clock power (mW)")
axes[0].set_xlim(0,xmax)
fig.suptitle("Ibex pitch-scaled mesh driver: skew vs power, backside vs frontside (shared skew axis)\n"
             "bigger driver rescues neither sparse side, but the backside sink tier caps the runaway",
             fontweight="bold",fontsize=12)
out=f"{I}/ibex_scaled_2panel.pdf"
plt.tight_layout(); plt.savefig(out,bbox_inches="tight",dpi=130); plt.savefig(out.replace('.pdf','.png'),bbox_inches="tight",dpi=130)
print("wrote",out)
print("BS:",[(q[1],round(q[4],1)) for q in bs]); print("FS:",[(q[1],round(q[4],1)) for q in fs])
