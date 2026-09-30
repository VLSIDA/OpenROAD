#!/usr/bin/env python3
# Stacked power bars, per-tier with buffer-internal folded into each buffer's section.
# Mesh section = mesh-driver dynamic + mesh-driver internal/short-circuit
# LCB section  = LCB dynamic (sink wire+FF pins) + LCB internal/short-circuit
# feeder = tiny Vclk-injected gate charging.  (from split-supply-rail HSpice)
import json
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
rows=json.load(open("/tmp/pwrsplit.json"))
labs=[r[0] for r in rows]
Mdyn=[r[1] for r in rows]; Mint=[r[2] for r in rows]
Ldyn=[r[3] for r in rows]; Lint=[r[4] for r in rows]; feed=[r[5] for r in rows]; tot=[r[6] for r in rows]
x=range(len(labs))
fig,ax=plt.subplots(figsize=(9.2,6.4))
b=[0]*len(x)
def seg(v,c,lab):
    ax.bar(x,v,bottom=b,color=c,edgecolor="white",linewidth=0.6,label=lab)
    for i,val in enumerate(v):
        if val>0.04: ax.text(i,b[i]+val/2,f"{val:.2f}",ha="center",va="center",fontsize=7.5,color="white",fontweight="bold")
    for i in range(len(b)): b[i]+=v[i]
seg(Mdyn,"#4c9be8","Mesh driver — dynamic (wire+gates)")
seg(Mint,"#1f4e8c","Mesh driver — internal + short-circuit")
seg(Ldyn,"#7ed07e","LCB — dynamic (sink wire + FF pins)")
seg(Lint,"#2e7d32","LCB — internal + short-circuit")
seg(feed,"#999999","Feeder-injected (driver input gates)")
for i,t in enumerate(tot): ax.text(i,t+0.02,f"{t:.2f} mW",ha="center",va="bottom",fontsize=9,fontweight="bold")
ax.set_xticks(list(x)); ax.set_xticklabels([l.replace("/","\n") for l in labs])
ax.set_ylabel("Clock power (mW)"); ax.set_xlabel("Mesh pitch / mesh-driver size")
ax.set_title("Ibex backside — clock power by tier (buffer-internal attributed to each tier)\n"
             "split-supply HSpice: why power is non-monotonic (P8 = minimum)",fontweight="bold",fontsize=11.5)
ax.legend(fontsize=8,loc="upper center",ncol=2)
ax.set_ylim(0,max(tot)*1.25)
out="/home/wali2/backside/OpenROAD/src/cms/test/backside_result/Ibex/ibex_power_tier.pdf"
plt.tight_layout(); plt.savefig(out,bbox_inches="tight",dpi=130); plt.savefig(out.replace(".pdf",".png"),bbox_inches="tight",dpi=130)
print("wrote",out)
