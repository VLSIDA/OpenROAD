#!/usr/bin/env python3
# Ibex scaled-driver Pareto on the NEW flow basis (router taps, M4-M8 clock):
# backside (green) vs frontside reference (red). X=power, Y=skew, line by pitch.
import sys, os, re
sys.path.insert(0,"/home/wali2/backside/OpenROAD/src/cms/test/backside_result/Jpeg")
from lis_metrics import metrics
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

I="/home/wali2/backside/OpenROAD/src/cms/test/backside_result/Ibex"
def gridsz(d, fallback="?"):
    p=os.path.join(d,"openroad.log")
    if os.path.exists(p):
        for l in open(p,errors='ignore'):
            m=re.search(r'expected (\d+) H-lines x (\d+) V-lines',l)
            if m: return f"{m.group(1)}x{m.group(2)}"
    return fallback

PTS=[("1.6","x2","pitch1p6","28x28"),("2.5","x3","pitch2p5","18x18"),
     ("8","x6","pitch8","6x6"),("12","x10","pitch12","4x4"),("16","x12","pitch16","3x3")]
def collect(base, fb_idx=3):
    d=[]
    for p,drv,tag,fb in PTS:
        dd=f"{base}/{tag}"; r=metrics(dd)
        if r and r["n"]: d.append((float(p),p,drv,r['pw'],r['skew'],gridsz(dd,fb)))
    return sorted(d,key=lambda q:q[0])
bs=collect(f"{I}/scaled_m4m8")
# frontside scaled reference (flow unchanged on FS -- no taps there)
fs=[]
for p,drv,tag,fb in PTS:
    dd=f"{I}/density/fs_scaled_{tag}"; r=metrics(dd)
    if r and r["n"]: fs.append((float(p),p,drv,r['pw'],r['skew'],gridsz(dd,fb)))
fs.sort(key=lambda q:q[0])

fig,ax=plt.subplots(figsize=(10,6.6))
def draw(data,color,dark,label,dy):
    xs=[q[3] for q in data]; ys=[q[4] for q in data]
    ax.plot(xs,ys,'-',color=color,lw=1.6,zorder=3)
    ax.scatter(xs,ys,color=color,s=95,marker="D",edgecolors="k",linewidths=0.5,label=label,zorder=4)
    for _,p,drv,pw,sk,sz in data:
        ax.annotate(f"P{p}/{drv}",(pw,sk),textcoords="offset points",xytext=(7,dy),fontsize=8.5,color=dark)
draw(bs,"#2ca02c","#1b5e20","Backside mesh (router taps, clock M4-M8)",5)
draw(fs,"#d62728","#8b0000","Frontside mesh (direct-to-mesh)",-13)

ax.set_xlabel("Clock power (mW)"); ax.set_ylabel("Worst-case skew, max-min (ps)")
ax.set_title("Ibex - Skew vs Power",fontsize=13,fontweight="bold")
ax.grid(alpha=0.3); ax.legend(fontsize=9.5,loc="upper right")

def minitable(rows,x,y,title,tcol):
    txt=title+"\n"+"\n".join(f"{a:>4}{b:>5}{c:>7}" for a,b,c in rows)
    ax.text(x,y,txt,transform=ax.transAxes,fontsize=8,va="top",ha="left",family="monospace",
            color=tcol,bbox=dict(boxstyle="round",fc="white",ec=tcol,alpha=0.95))
minitable([["Ptch","Drv","Size"]]+[[q[1],q[2],q[5]] for q in bs],0.66,0.80,"BS mesh","#2ca02c")
minitable([["Ptch","Drv","Size"]]+[[q[1],q[2],q[5]] for q in fs],0.83,0.80,"FS mesh","#d62728")

out=f"{I}/ibex_scaled_m4m8_xy.pdf"
plt.tight_layout(); plt.savefig(out,bbox_inches="tight",dpi=130); plt.savefig(out.replace('.pdf','.png'),bbox_inches="tight",dpi=130)
print("wrote",out)
for q in bs: print(f"BS P{q[1]:>4}/{q[2]:<4} skew={q[4]:5.1f}ps pw={q[3]:.2f}mW")
