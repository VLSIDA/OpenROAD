#!/usr/bin/env python3
# Figure 2: skew decomposition per pitch (backside scaled-driver).
#   bottom = mesh-delivered skew (measured at LCB INPUT pins, t_lcb)
#   top    = LCB-tier addition   (FF-pin skew - LCB-input skew)
#   total  = FF-pin skew (t_sink)
# Shows why FF skew flattens at dense pitch: mesh delivers ~0, LCB tier sets floor.
import re, glob
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
SS="/home/wali2/backside/OpenROAD/src/cms/test/backside_result/Ibex/density/skew_split"
PTS=[("P1.6\nx2","scaled_pitch1p6"),("P2.5\nx3","scaled_pitch2p5"),("P8\nx6","scaled_pitch8"),
     ("P12\nx10","scaled_pitch12"),("P16\nx12","scaled_pitch16")]
def arr(lis,pre):
    v=[]
    for l in open(lis,errors='ignore'):
        m=re.match(rf'\s*({pre}_\d+)=\s*([-0-9.eE+]+)([a-z]?)',l)
        if m:
            u={'p':1e-12,'n':1e-9,'':1}.get(m.group(3),1); x=float(m.group(2))*u
            if 0<x<1e-6: v.append(x*1e12)
    return v
labs=[];mesh=[];tier=[];tot=[]
for lab,tag in PTS:
    lis=glob.glob(f"{SS}/{tag}/*.lis")[0]
    lcb=arr(lis,"t_lcb"); ff=arr(lis,"t_sink")
    skl=max(lcb)-min(lcb); skf=max(ff)-min(ff)
    labs.append(lab);mesh.append(skl);tier.append(max(0,skf-skl));tot.append(skf)
x=range(len(labs))
fig,ax=plt.subplots(figsize=(9,6.3))
ax.bar(x,mesh,color="#1f77b4",edgecolor="white",label="Mesh-delivered skew (at LCB input)")
ax.bar(x,tier,bottom=mesh,color="#5cb85c",edgecolor="white",label="LCB tier + distribution")
for i in x:
    if mesh[i]>2: ax.text(i,mesh[i]/2,f"{mesh[i]:.1f}",ha="center",va="center",color="white",fontsize=8,fontweight="bold")
    ax.text(i,mesh[i]+tier[i]/2,f"+{tier[i]:.1f}",ha="center",va="center",color="white",fontsize=8,fontweight="bold")
    ax.text(i,tot[i]+1.2,f"{tot[i]:.1f} ps",ha="center",va="bottom",fontsize=9,fontweight="bold")
ax.set_xticks(list(x)); ax.set_xticklabels([p[0] for p in PTS])
ax.set_ylabel("Worst-case skew, max-min (ps)"); ax.set_xlabel("Mesh pitch / driver size")
ax.set_title("Ibex - Skew decomposition",fontweight="bold",fontsize=13)
ax.legend(loc="upper left",fontsize=9); ax.set_ylim(0,max(tot)*1.18)
out="/home/wali2/backside/OpenROAD/src/cms/test/backside_result/Ibex/ibex_skew_split.pdf"
plt.tight_layout(); plt.savefig(out,bbox_inches="tight",dpi=130); plt.savefig(out.replace(".pdf",".png"),bbox_inches="tight",dpi=130)
print("wrote",out)
for i,l in enumerate(labs): print(f"{l.replace(chr(10),' '):9s} mesh={mesh[i]:.1f}  tier=+{tier[i]:.1f}  FFpin={tot[i]:.1f}")
