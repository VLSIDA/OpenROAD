#!/usr/bin/env python3
# Single graph, FIXED x4 mesh driver across pitch: backside vs frontside.
# X=power, Y=skew. Curated pitches (one per grid size), line by pitch order.
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

# keep only the pitches used in the scaled graphs (P4/P6/P10 dropped)
PITCHES=[("1.6","pitch1p6"),("2.5","pitch2p5"),
         ("8","pitch8"),("12","pitch12"),("16","pitch16")]
def bsdir(tag): return f"{I}/baltree_p1p6/full/x4/fmax8" if tag=="pitch1p6" else f"{I}/density/{tag}"
def fsdir(tag): return f"{I}/frontside_p1p6" if tag=="pitch1p6" else f"{I}/density/fs_{tag}"

def collect(dirfn):
    d=[]
    for p,tag in PITCHES:
        dd=dirfn(tag); r=metrics(dd)
        if r and r["n"]: d.append((float(p),p,r['pw'],r['skew'],gridsz(dd)))
    return sorted(d,key=lambda q:q[0])
bs=collect(bsdir); fs=collect(fsdir)

fig,ax=plt.subplots(figsize=(10,6.6))
def draw(data,color,dark,label,dy):
    xs=[q[2] for q in data]; ys=[q[3] for q in data]   # X=power, Y=skew
    ax.plot(xs,ys,'-',color=color,lw=1.6,zorder=3)
    ax.scatter(xs,ys,color=color,s=90,marker="o",edgecolors="k",linewidths=0.4,label=label,zorder=4)
    for _,p,pw,sk,sz in data:
        ax.annotate(f"P{p}",(pw,sk),textcoords="offset points",xytext=(6,dy),fontsize=8.5,color=dark)
draw(bs,"#1f77b4","#0d3b66","Backside mesh (sink x4/fanout8)",5)
draw(fs,"#d62728","#8b0000","Frontside mesh (direct-to-mesh)",-13)

ax.set_xlabel("Clock power (mW)"); ax.set_ylabel("Worst-case skew, max-min (ps)")
ax.set_title("Ibex fixed x4 mesh driver — skew vs power across pitch, backside vs frontside\n"
             "(sink x4/fanout8, full LVT feeder, balanced PDN)",fontsize=12,fontweight="bold")
ax.grid(alpha=0.3); ax.legend(fontsize=9.5,loc="upper right")

# mesh-size legend tables (pitch -> grid), BS and FS, right side under the legend
def minitable(rows,x,y,title,tcol):
    txt=title+"\n"+"\n".join(f"{a:>5} {b:>6}" for a,b in rows)
    ax.text(x,y,txt,transform=ax.transAxes,fontsize=8.5,va="top",ha="left",family="monospace",
            color=tcol,bbox=dict(boxstyle="round",fc="white",ec=tcol,alpha=0.95))
bs_rows=[["Pitch","Size"]]+[[q[1],q[4]] for q in bs]
fs_rows=[["Pitch","Size"]]+[[q[1],q[4]] for q in fs]
minitable(bs_rows,0.68,0.86,"BS mesh","#1f77b4")
minitable(fs_rows,0.84,0.86,"FS mesh","#d62728")
out=f"{I}/ibex_fixed_combined.pdf"
plt.tight_layout(); plt.savefig(out,bbox_inches="tight",dpi=130); plt.savefig(out.replace('.pdf','.png'),bbox_inches="tight",dpi=130)
print("wrote",out)
print("BS:",[(q[1],round(q[3],1),round(q[2],2)) for q in bs])
print("FS:",[(q[1],round(q[3],1),round(q[2],2)) for q in fs])
