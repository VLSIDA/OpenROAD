#!/usr/bin/env python3
# Stacked power bars per pitch (backside scaled-driver). Decompose HSpice avg_power:
#   wire power   = CV^2f of all routing wire (mesh+stub+LCB taps/dist)
#   driver power = CV^2f of mesh-driver input-gate cap
#   LCB power    = CV^2f of (LCB gate cap + FF pin loads)   [sink tier]
#   leftover     = SPICE avg_power - the three above = short-circuit + buffer-internal
import glob, re, sys
sys.path.insert(0,"/home/wali2/backside/OpenROAD/src/cms/test/backside_result/Jpeg")
from lis_metrics import metrics
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

LIB="/home/wali2/backside/OpenROAD-flow-scripts/flow/platforms/gt2n/lib/gt2_6t_w31_lvt_tt_0p7v25c.lib"
lib=open(LIB).read()
def cin(c):
    m=re.search(r'cell\('+re.escape(c)+r'\)\s*\{.*?pin\(A\)\s*\{\s*capacitance\s*:\s*([0-9.eE+-]+)',lib,re.S)
    return float(m.group(1)) if m else 0.0
I="/home/wali2/backside/OpenROAD/src/cms/test/backside_result/Ibex/density"
VDD=0.7; F=1e9
PTS=[("P1.6\nx2","28x28","scaled_pitch1p6"),("P2.5\nx3","18x18","scaled_pitch2p5"),
     ("P8\nx6","6x6","scaled_pitch8"),("P12\nx10","4x4","scaled_pitch12"),("P16\nx12","3x3","scaled_pitch16")]
def cls(n):
    if "DFF" in n or n.endswith("CLK"): return "ffpin"
    if n.startswith("b_proxy"): return "mesh"
    if "clk_i_buf" in n or n.startswith("b_clk"): return "stub"
    if n.startswith(("sink_drv","sink_tap","b_sink")): return "lcb"
    return "other"

labs=[];wire=[];drv=[];lcb=[];left=[];tot=[]
for lab,sz,tag in PTS:
    d=f"{I}/{tag}"; sp=glob.glob(d+"/*.sp")[0]
    s={k:0 for k in["mesh","stub","lcb","ffpin","other"]};nd=0;dc=None;nl=0;lc=None
    for l in open(sp,errors='ignore'):
        if l and l[0] in "Cc":
            p=l.split()
            if len(p)>=4:
                try:s[cls(p[1])]+=float(p[-1])
                except:pass
        elif l.startswith("Xmesh_buf"):nd+=1;dc=l.split()[-1]
        elif l.startswith("Xsink_buf"):nl+=1;lc=l.split()[-1]
    k=VDD*VDD*F*1e3   # F -> mW
    Cwire=(s['mesh']+s['stub']+s['lcb']+s['other']); Cdrv=nd*cin(dc)*1e-12; Clcb=nl*cin(lc)*1e-12+s['ffpin']
    pw=metrics(d)['pw']
    pwire=Cwire*k; pdrv=Cdrv*k; plcb=Clcb*k; pleft=max(0.0,pw-pwire-pdrv-plcb)
    labs.append(lab.replace("\n"," ")); wire.append(pwire);drv.append(pdrv);lcb.append(plcb);left.append(pleft);tot.append(pw)

x=range(len(labs)); xl=[p[0] for p in PTS]
fig,ax=plt.subplots(figsize=(9,6.3))
b=[0]*len(x)
def add(vals,color,label):
    ax.bar(x,vals,bottom=b,color=color,edgecolor="white",linewidth=0.6,label=label)
    for i,v in enumerate(vals):
        if v>0.03: ax.text(i,b[i]+v/2,f"{v:.2f}",ha="center",va="center",fontsize=7.5,color="white",fontweight="bold")
    for i in range(len(b)): b[i]+=vals[i]
add(wire,"#4c9be8","Wire (mesh+stub+LCB routing)")
add(drv,"#f0a03c","Mesh-driver gate")
add(lcb,"#5cb85c","LCB tier (LCB gate + FF pin)")
add(left,"#d9534f","Short-circuit + buffer-internal")
for i,t in enumerate(tot): ax.text(i,t+0.02,f"{t:.2f} mW",ha="center",va="bottom",fontsize=9,fontweight="bold")
ax.set_xticks(list(x)); ax.set_xticklabels(xl)
ax.set_ylabel("Clock power (mW)"); ax.set_xlabel("Mesh pitch / driver size")
ax.set_title("Ibex backside — clock power decomposition per pitch\nwhy power is non-monotonic (P8 = minimum)",fontweight="bold",fontsize=12)
ax.legend(fontsize=8.5,loc="upper center",ncol=2)
ax.set_ylim(0,max(tot)*1.22)
out="/home/wali2/backside/OpenROAD/src/cms/test/backside_result/Ibex/ibex_power_stack.pdf"
plt.tight_layout(); plt.savefig(out,bbox_inches="tight",dpi=130); plt.savefig(out.replace(".pdf",".png"),bbox_inches="tight",dpi=130)
print("wrote",out)
for i,l in enumerate(labs):
    print(f"{l:9s} wire={wire[i]:.2f} drv={drv[i]:.2f} lcb={lcb[i]:.2f} left={left[i]:.2f}  tot={tot[i]:.2f}")
