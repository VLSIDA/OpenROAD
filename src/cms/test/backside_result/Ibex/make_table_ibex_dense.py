#!/usr/bin/env python3
# Ibex pitch-1.6 backside sink-buffer sweep table PDF (jpeg_dense_sweep.pdf format).
import sys
sys.path.insert(0,"/home/wali2/backside/OpenROAD/src/cms/test/backside_result/Jpeg")
from lis_metrics import metrics
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

B="/home/wali2/backside/OpenROAD/src/cms/test/backside_result/Ibex/baltree_p1p6/full"
FS_DIR="/home/wali2/backside/OpenROAD/src/cms/test/backside_result/Ibex/frontside_p1p6"
GRID={"x1":[8,16,20],"x2":[6,16,24,40],"x3":[8,16,24,60],"x4":[8,16,24,80],
      "x6":[8,16,24,80],"x8":[8,16,24,80],"x10":[8,16,24,80],"x12":[8,16,24,80]}
HEAD="#455a64"; FSROW="#fff3c4"
headers=["Sink Local Buffer","Fanout Sink Buffer","WORST Skew (ps)\nmax-min","Insertion (ps)","Power (mW)"]
cells=[]; colors=[]; groups=[]
for b,fl in GRID.items():
    cnt=0
    for f in fl:
        r=metrics(f"{B}/{b}/fmax{f}")
        if not r: continue
        cells.append([b,f"{f}",f"{r['skew']:.1f}",f"{r['ins']:.0f}",f"{r['pw']:.3f}"])
        colors.append(["white"]*len(headers))
        cnt+=1
    if cnt: groups.append((b,cnt))
# frontside-mesh reference row (direct FF drive, same pitch)
fr=metrics(FS_DIR)
if fr:
    cells.append(["FS mesh","direct",f"{fr['skew']:.1f}",f"{fr['ins']:.0f}",f"{fr['pw']:.3f}"])
    colors.append([FSROW]*len(headers))

NC=len(headers); all_cells=[headers]+cells; all_colors=[[HEAD]*NC]+colors
fig,ax=plt.subplots(figsize=(8.5,0.4*len(all_cells)+1.4)); ax.axis("off")
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
ax.set_title("Ibex backside Mesh — Pitch: 1.6, Size: 28x28\n"
             "worst-case skew (max-min), from HSpice .lis",
             fontsize=12,fontweight="bold",pad=14)
out="/home/wali2/backside/OpenROAD/src/cms/test/backside_result/Ibex/ibex_dense_sweep.pdf"
plt.savefig(out,bbox_inches="tight"); print("wrote",out)
