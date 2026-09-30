#!/usr/bin/env python3
# Ibex pitch-1.6 skew-vs-power: backside (BM1/BM2) 31-config sink sweep vs the
# single frontside (M5/M6) reference point. On the frontside FFs connect directly
# to the mesh (connect_sinks_to_mesh) -- no sink-buffer / F_max sweep -- so it is
# ONE config, drawn as a red marker with reference lines. Writes PDF+PNG.
import os, glob, sys
sys.path.insert(0, "/home/wali2/backside/OpenROAD/src/cms/test/backside_result/Jpeg")
import lis_metrics as L
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

BASE="/home/wali2/backside/OpenROAD/src/cms/test/backside_result/Ibex"

def collect(subdir, side):
    pts=[]
    for lis in glob.glob(f"{BASE}/{subdir}/full/*/fmax*/ibex_{side}.lis"):
        d=os.path.dirname(lis)
        buf=d.split("/full/")[1].split("/")[0]
        fmax=int(os.path.basename(d).replace("fmax",""))
        m=L.metrics(d)
        if not m or not m["n"]: continue
        pts.append(dict(buf=buf,fmax=fmax,skew=m["skew"],pw=m["pw"],n=m["n"],ins=m["ins"]))
    return pts

bs=collect("baltree_p1p6","backside")
# frontside is ONE config (FFs direct-to-mesh, no sink tier) -> flat dir
m=L.metrics(f"{BASE}/frontside_p1p6")
fs_pt=dict(skew=m["skew"],pw=m["pw"],n=m["n"],ins=m["ins"]) if m and m["n"] else None

fig,ax=plt.subplots(figsize=(9,6.6))
# backside cloud
xs=[p["pw"] for p in bs]; ys=[p["skew"] for p in bs]
ax.scatter(xs,ys,c="tab:blue",marker="o",s=44,alpha=0.82,edgecolors="k",linewidths=0.4,
           label=f"Backside BM1/BM2  ({len(bs)} sink configs)")
for p in bs:
    if p["buf"]=="x4" and p["fmax"]==8:
        ax.annotate("x4/F8",(p["pw"],p["skew"]),textcoords="offset points",xytext=(6,4),
                    fontsize=8,color="tab:blue",fontweight="bold")
# frontside single reference point + crosshair lines
if fs_pt:
    ax.axhline(fs_pt["skew"],color="tab:red",ls="--",lw=1,alpha=0.5)
    ax.axvline(fs_pt["pw"],color="tab:red",ls="--",lw=1,alpha=0.5)
    ax.scatter([fs_pt["pw"]],[fs_pt["skew"]],c="tab:red",marker="*",s=340,edgecolors="k",
               linewidths=0.6,zorder=5,
               label=f"Frontside M5/M6 (direct-to-mesh, 1 config)")
    ax.annotate(f"  FS: {fs_pt['skew']:.1f} ps, {fs_pt['pw']:.2f} mW",
                (fs_pt["pw"],fs_pt["skew"]),textcoords="offset points",xytext=(8,-4),
                fontsize=9,color="tab:red",fontweight="bold")

ax.set_xlabel("Clock power (mW)"); ax.set_ylabel("Worst-case skew, max-min (ps)")
ax.set_title("Ibex clock mesh @ pitch 1.6 — skew vs power\n"
             "Backside sink-buffer sweep vs Frontside direct-to-mesh reference "
             "(balanced PDN, full LVT feeder)",fontweight="bold",fontsize=11)
ax.grid(True,alpha=0.3); ax.legend(loc="upper right")

print("== BACKSIDE (31) ==")
for p in sorted(bs,key=lambda z:(z['buf'],z['fmax'])):
    print(f"  {p['buf']:3s} F{p['fmax']:<3d}  skew={p['skew']:6.1f}ps  pw={p['pw']:6.2f}mW  n={p['n']}")
if fs_pt: print(f"== FRONTSIDE (1) ==  skew={fs_pt['skew']:.1f}ps  pw={fs_pt['pw']:.2f}mW  n={fs_pt['n']}")

out=f"{BASE}/ibex_pareto_p1p6.pdf"
plt.tight_layout(); plt.savefig(out); plt.savefig(out.replace(".pdf",".png"),dpi=140)
print("wrote",out)
