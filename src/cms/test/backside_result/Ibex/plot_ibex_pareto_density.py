#!/usr/bin/env python3
# Ibex Pareto: x4 sink / fanout-8 skew vs power across mesh pitch, backside vs
# frontside (direct). Same format as jpeg_pareto_x4f8.pdf.
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

# (pitch label, dir) ascending pitch. 1.6 lives in its own dirs; rest in density/.
# Curated, well-spaced subset (one per grid size, evenly spread) for a clean curve.
PITCHES=[("1.6","pitch1p6"),("2.5","pitch2p5"),("4","pitch4"),("6","pitch6"),
         ("8","pitch8"),("10","pitch10"),("12","pitch12"),("16","pitch16")]
def bsdir(tag): return f"{I}/baltree_p1p6/full/x4/fmax8" if tag=="pitch1p6" else f"{I}/density/{tag}"
def fsdir(tag): return f"{I}/frontside_p1p6" if tag=="pitch1p6" else f"{I}/density/fs_{tag}"

fig,ax=plt.subplots(figsize=(9.5,6.8))
def series(dirfn,color,ls,marker,label,dy):
    xs=[];ys=[];labs=[];szs=[]
    seen=set()
    for p,tag in PITCHES:            # ascending pitch: keep finest per unique grid
        d=dirfn(tag); r=metrics(d)
        if not r or not r["n"]: continue
        sz=gridsz(d)
        if sz in seen: continue      # drop pitches that quantize to a size already plotted
        seen.add(sz)
        xs.append(r['pw']); ys.append(r['skew']); labs.append(p); szs.append(sz)
    order=sorted(range(len(xs)),key=lambda i:float(labs[i]))
    xs=[xs[i] for i in order]; ys=[ys[i] for i in order]; labs=[labs[i] for i in order]; szs=[szs[i] for i in order]
    if ls:   # frontside: connect (its skew is monotonic in pitch)
        ax.plot(xs,ys,marker+ls,color=color,ms=8,lw=1.6,label=label,zorder=4)
    else:    # backside: scatter cloud (power non-monotonic in pitch -> a line self-crosses)
        ax.scatter(xs,ys,color=color,s=70,edgecolors="k",linewidths=0.4,label=label,zorder=4)
    for x,y,l in zip(xs,ys,labs):
        ax.annotate(f"P{l}",(x,y),textcoords="offset points",xytext=(5,dy),fontsize=8,color=color)
    return list(zip(labs,szs))

bs=series(bsdir,"#1f77b4","","o","Backside mesh (x4 sink, fanout 8)",6)
fs=series(fsdir,"#d62728","--","o","Frontside mesh (direct-to-mesh)",-13)

ax.set_xlabel("Clock power (mW)"); ax.set_ylabel("Worst-case skew, max-min (ps)")
ax.set_title("Ibex — x4 sink buffer, fanout 8: skew vs power across mesh pitch\n(backside vs frontside, balanced PDN, full LVT feeder)",
             fontsize=12,fontweight="bold")
ax.grid(alpha=0.3); ax.legend(fontsize=9,loc="upper left")

def minitable(rows,x,y,title,tcol):
    txt=title+"\n"+"\n".join(f"{a:>5} {b:>6}" for a,b in rows)
    ax.text(x,y,txt,transform=ax.transAxes,fontsize=7.5,va="top",ha="left",family="monospace",
            color=tcol,bbox=dict(boxstyle="round",fc="white",ec=tcol,alpha=0.95))
minitable(bs,0.70,0.98,"BS pitch/size","#1f77b4")
minitable(fs,0.86,0.98,"FS pitch/size","#d62728")

out=f"{I}/ibex_pareto_x4f8.pdf"
plt.tight_layout(); plt.savefig(out,bbox_inches="tight",dpi=130); plt.savefig(out.replace('.pdf','.png'),bbox_inches="tight",dpi=130)
print("wrote",out)
for p,tag in PITCHES:
    rb=metrics(bsdir(tag)); rf=metrics(fsdir(tag))
    print(f"P{p:>4}  BS {rb['skew']:6.1f}ps/{rb['pw']:5.2f}mW" if rb else f"P{p:>4}  BS   --",
          f"| FS {rf['skew']:6.1f}ps/{rf['pw']:5.2f}mW" if rf else "| FS  --")
