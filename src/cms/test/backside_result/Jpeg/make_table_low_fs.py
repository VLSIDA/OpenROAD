#!/usr/bin/env python3
# Colored PDF: backside LOW-feeder sweep vs FS-mesh (low feeder) reference only.
import os, re, statistics
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

BASE="/home/wali2/backside/OpenROAD/src/cms/test/backside_result/Jpeg/baltree"
GRID={"x1":[8,16,20],"x2":[6,16,24,40],"x3":[8,16,24,60],
      "x4":[8,16,24,80],"x6":[8,16,24,80],"x8":[8,16,24,80],
      "x10":[8,16,24,80],"x12":[8,16,24,80]}

def parse(mt):
    if not os.path.exists(mt): return None
    toks=[]
    for l in open(mt):
        if l.startswith('$') or l.strip().startswith('.TITLE'): continue
        toks+=l.split()
    d=dict(zip([t for t in toks if re.match(r'[A-Za-z]',t)],[t for t in toks if not re.match(r'[A-Za-z]',t)]))
    s=[float(v) for k,v in d.items() if re.match(r't_sink_\d+$',k) and re.match(r'[-0-9.eE+]+$',v)]
    g=[v for v in s if 0<v<1e6]
    if not g: return None
    return (max(g)-min(g))*1e12, statistics.mean(g)*1e12, float(d.get('avg_power',0))*1e3

fs=parse(f"{BASE}/fs_low/jpeg_frontside.mt0")
FS_SKEW,FS_INS,FS_PW=(fs[0],fs[1],fs[2]) if fs else (0,0,0)
def pct(ref,val):
    d=(ref-val)/ref*100
    return f"{'+' if d>=0 else ''}{d:.0f}%", d

GREEN="#c8e6c9"; RED="#ffcdd2"; HEAD="#455a64"; FSROW="#fff3c4"
headers=["Hanging Buffer","F_max","Skew (ps)","Insertion (ps)","Power (mW)",
         "Skew vs FS","Ins vs FS","Power vs FS"]
cells=[]; colors=[]; groups=[]
for b,fl in GRID.items():
    cnt=0
    for f in fl:
        r=parse(f"{BASE}/low/{b}/fmax{f}/jpeg_backside.mt0")
        if not r: continue
        sk,ins,pw=r
        s1,d1=pct(FS_SKEW,sk); s2,d2=pct(FS_INS,ins); s3,d3=pct(FS_PW,pw)
        cells.append([b,f"fmax{f}",f"{sk:.1f}",f"{ins:.0f}",f"{pw:.3f}",s1,s2,s3])
        colors.append(["white"]*5+[GREEN if d1>=0 else RED,GREEN if d2>=0 else RED,GREEN if d3>=0 else RED])
        cnt+=1
    if cnt: groups.append((b,cnt))
cells.append(["FS mesh","(ref)",f"{FS_SKEW:.1f}",f"{FS_INS:.0f}",f"{FS_PW:.3f}","0%","0%","0%"])
colors.append([FSROW]*8)

NC=len(headers)
all_cells=[headers]+cells; all_colors=[[HEAD]*NC]+colors
fig,ax=plt.subplots(figsize=(12,0.4*len(all_cells)+2.0)); ax.axis("off")
t=ax.table(cellText=all_cells,cellColours=all_colors,cellLoc="center",loc="center")
t.auto_set_font_size(False); t.set_fontsize(9); t.scale(1,1.45)
for j in range(NC): t[0,j].set_text_props(color="white",fontweight="bold")
start=0
for b,g in groups:
    mid=start+g//2
    for k in range(start,start+g):
        c=t[k+1,0]; c.get_text().set_text(b if k==mid else "")
        c.visible_edges="LTR" if k==start else ("LBR" if k==start+g-1 else "LR")
        c.set_text_props(fontweight="bold")
    start+=g
fig.canvas.draw()
cols=(5,6,7)
x0=min(t[0,c].get_x() for c in cols); x1=max(t[0,c].get_x()+t[0,c].get_width() for c in cols)
yt=t[0,cols[0]].get_y()+t[0,cols[0]].get_height(); h=t[0,cols[0]].get_height()
ax.add_patch(Rectangle((x0+0.004,yt+0.002),(x1-x0)-0.008,h,facecolor=HEAD,edgecolor="white",lw=1.2,transform=ax.transAxes,clip_on=False,zorder=3))
ax.text((x0+x1)/2,yt+0.002+h/2,"Comparison with FS Mesh",ha="center",va="center",color="white",fontsize=10,fontweight="bold",transform=ax.transAxes,clip_on=False,zorder=4)
ax.set_title("JPEG backside Mesh — LVT (Mesh Buffer x4), Size 25x25 — LOW feeder",
             fontsize=12,fontweight="bold",pad=20)
out=f"{os.path.dirname(BASE)}/jpeg_baltree_low_vs_fs.pdf"
plt.savefig(out,bbox_inches="tight"); print("wrote",out)
print(f"FS-low ref: skew={FS_SKEW:.1f}ps ins={FS_INS:.0f}ps power={FS_PW:.3f}mW")
