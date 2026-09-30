#!/usr/bin/env python3
# Colored PDF of the backside full-feeder sweep, vs CTS + FS references.
# Extra columns: Skew vs CTS, Power vs CTS (green=better than CTS, red=worse).
import os, re, statistics
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

BASE = "/home/wali2/backside/OpenROAD/src/cms/test/backside_result/Jpeg/baltree"
GRID = {"x1":[8,16,20],"x2":[6,16,24,40],"x3":[8,16,24,60],
        "x4":[8,16,24,80],"x6":[8,16,24,80],"x8":[8,16,24,80],
        "x10":[8,16,24,80],"x12":[8,16,24,80]}
# CTS reference (balanced PDN, full LVT feeder) -- STA global skew, insertion, clock-group power
CTS_SKEW, CTS_INS, CTS_PW = 47.97, 330.3, 2.80

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

fs = parse(f"{BASE}/fs_full/jpeg_frontside.mt0")     # (skew, ins, pw)

FS_SKEW, FS_INS, FS_PW = (fs[0], fs[1], fs[2]) if fs else (34.5, 326.0, 4.367)
def pct(ref, val):
    d=(ref-val)/ref*100
    return f"{'+' if d>=0 else ''}{d:.0f}%", d
def gr(d): return GREEN if d>=0 else RED

GREEN="#c8e6c9"; RED="#ffcdd2"; HEAD="#455a64"; CTSROW="#d0e6f5"; FSROW="#fff3c4"
headers=["Hanging Buffer","F_max","Skew (ps)","Insertion (ps)","Power (mW)",
         "Skew vs CTS","Ins vs CTS","Power vs CTS","Skew vs FS","Ins vs FS","Power vs FS"]
cells=[]; colors=[]; groups=[]     # groups = (buffer, nrows) for merging the Buffer col
for b,fl in GRID.items():
    cnt=0
    for f in fl:
        r=parse(f"{BASE}/full/{b}/fmax{f}/jpeg_backside.mt0")
        if not r: continue
        sk,ins,pw=r
        skc,d1=pct(CTS_SKEW,sk); inc,d2=pct(CTS_INS,ins); pwc,d3=pct(CTS_PW,pw)
        skf,d4=pct(FS_SKEW,sk);  inf,d5=pct(FS_INS,ins);  pwf,d6=pct(FS_PW,pw)
        cells.append([b,f"fmax{f}",f"{sk:.1f}",f"{ins:.0f}",f"{pw:.3f}",skc,inc,pwc,skf,inf,pwf])
        colors.append(["white"]*5+[gr(d1),gr(d2),gr(d3),gr(d4),gr(d5),gr(d6)])
        cnt+=1
    if cnt: groups.append((b,cnt))
n_data=len(cells)
# reference rows
cells.append(["CTS tree","(ref)",f"{CTS_SKEW:.1f}",f"{CTS_INS:.0f}","2.80 (STA)","0%","0%","0%",
              pct(FS_SKEW,CTS_SKEW)[0],pct(FS_INS,CTS_INS)[0],pct(FS_PW,CTS_PW)[0]]); colors.append([CTSROW]*11)
cells.append(["FS mesh","(ref)",f"{FS_SKEW:.1f}",f"{FS_INS:.0f}",f"{FS_PW:.3f}",
              pct(CTS_SKEW,FS_SKEW)[0],pct(CTS_INS,FS_INS)[0],pct(CTS_PW,FS_PW)[0],"0%","0%","0%"]); colors.append([FSROW]*11)

NC=len(headers)
all_cells=[headers]+cells                 # single header row (row 0)
all_colors=[[HEAD]*NC]+colors

fig,ax=plt.subplots(figsize=(16,0.4*len(all_cells)+2.0)); ax.axis("off")
t=ax.table(cellText=all_cells,cellColours=all_colors,cellLoc="center",loc="center")
t.auto_set_font_size(False); t.set_fontsize(9); t.scale(1,1.45)
for j in range(NC): t[0,j].set_text_props(color="white",fontweight="bold")
# merge the Buffer column per buffer group (row = k+1)
start=0
for b,g in groups:
    mid=start+g//2
    for k in range(start,start+g):
        cell=t[k+1,0]
        cell.get_text().set_text(b if k==mid else "")
        cell.visible_edges = "LTR" if k==start else ("LBR" if k==start+g-1 else "LR")
        cell.set_text_props(fontweight="bold")
    start+=g
# grouped super-labels drawn as their OWN boxed cells ABOVE the header row,
# each spanning its 3-col group, with a gap between them for separation.
from matplotlib.patches import Rectangle
fig.canvas.draw()
for cols,label in [((5,6,7),"Comparison with CTS"),((8,9,10),"Comparison with FS Mesh")]:
    x0=min(t[0,c].get_x() for c in cols)
    x1=max(t[0,c].get_x()+t[0,c].get_width() for c in cols)
    yt=t[0,cols[0]].get_y()+t[0,cols[0]].get_height()
    h =t[0,cols[0]].get_height()
    gap=0.004
    ax.add_patch(Rectangle((x0+gap, yt+0.002), (x1-x0)-2*gap, h,
        facecolor=HEAD, edgecolor="white", lw=1.2, transform=ax.transAxes,
        clip_on=False, zorder=3))
    ax.text((x0+x1)/2, yt+0.002+h/2, label, ha="center", va="center",
            color="white", fontsize=10, fontweight="bold",
            transform=ax.transAxes, clip_on=False, zorder=4)
ax.set_title("JPEG backside Mesh — LVT (Mesh Buffer x4), Size 25x25",
             fontsize=12, fontweight="bold", pad=20)
out=f"{os.path.dirname(BASE)}/jpeg_baltree_full_vs_cts.pdf"
plt.savefig(out,bbox_inches="tight"); print("wrote",out)
print(f"FS ref: skew={fs[0]:.1f}ps ins={fs[1]:.0f}ps power={fs[2]:.3f}mW" if fs else "FS: no result")
