#!/usr/bin/env python3
# Backside scaled-driver Pareto: skew vs power, pitch-scaled mesh driver only.
# Drop P6 and P10.
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
# (pitch, driver, dir tag) -- P4, P6 and P10 dropped
PTS=[("1.6","x2","fs_scaled_pitch1p6"),("2.5","x3","fs_scaled_pitch2p5"),
     ("8","x6","fs_scaled_pitch8"),
     ("12","x10","fs_scaled_pitch12"),("16","x12","fs_scaled_pitch16")]

data=[]
for p,drv,tag in PTS:
    d=f"{I}/density/{tag}"; r=metrics(d)
    if r and r["n"]: data.append((float(p),p,drv,r['pw'],r['skew'],gridsz(d)))
data.sort()

fig,ax=plt.subplots(figsize=(9,6.5))
# order by PITCH ascending (lowest -> highest), connect in that order
data_byp=sorted(data,key=lambda q:q[0])
xs=[q[4] for q in data_byp]; ys=[q[3] for q in data_byp]   # X=skew, Y=power
ax.plot(xs,ys,'-',color="#d62728",lw=1.6,zorder=3)
ax.scatter(xs,ys,color="#d62728",s=90,marker="D",edgecolors="k",linewidths=0.5,zorder=4)
for _,p,drv,pw,sk,sz in data:
    ax.annotate(f"P{p} / {drv}",(sk,pw),textcoords="offset points",xytext=(7,5),fontsize=9,color="#8b0000")
ax.set_xlabel("Worst-case skew, max-min (ps)"); ax.set_ylabel("Clock power (mW)")
ax.set_title("Ibex frontside mesh — pitch-scaled mesh driver\nskew vs power (direct-to-mesh, full feeder)",
             fontsize=12,fontweight="bold")
ax.grid(alpha=0.3)

# legend table: pitch / mesh driver / grid size
rows=[["Pitch","Driver","Mesh size"]]+[[p,drv,sz] for _,p,drv,x,y,sz in data]
txt="\n".join(f"{r[0]:>5} {r[1]:>6} {r[2]:>7}" for r in rows)
ax.text(0.985,0.97,txt,transform=ax.transAxes,fontsize=9,va="top",ha="right",family="monospace",
        color="#8b0000",bbox=dict(boxstyle="round",fc="white",ec="#d62728",alpha=0.95))
out=f"{I}/ibex_fs_scaled.pdf"
plt.tight_layout(); plt.savefig(out,bbox_inches="tight",dpi=130); plt.savefig(out.replace('.pdf','.png'),bbox_inches="tight",dpi=130)
print("wrote",out)
for _,p,drv,x,y in data: print(f"P{p:<5}{drv:5s} skew={y:6.1f}ps  pw={x:.2f}mW")
