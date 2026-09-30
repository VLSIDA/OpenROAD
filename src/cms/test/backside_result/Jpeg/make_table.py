#!/usr/bin/env python3
# Build a single colored PDF table of the JPEG sink-buffer x F_max sweep.
# Columns: buffer | F_max | sink TSVs | skew(ps) | insertion(ps) | power(mW)
#          | skew vs FS (%) | power vs FS (%)   (green=better, red=worse)
# Last row = frontside (FS) reference.
import os, re, statistics
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

JDIR = "/home/wali2/backside/OpenROAD/src/cms/test/backside_result/Jpeg"
BUFS = ["x4", "x6", "x8", "x10", "x12"]
FMAXS = [8, 16, 24, 80]

def parse_mt0(path):
    if not os.path.exists(path):
        return None
    toks = []
    for l in open(path):
        if l.startswith('$DATA') or l.strip().startswith('.TITLE'):
            continue
        toks += l.split()
    if 'alter#' not in toks:
        return None
    i = toks.index('alter#')
    d = dict(zip(toks[:i+1], toks[i+1:]))
    s = []
    for k, v in d.items():
        if re.match(r't_sink_\d+$', k) and v != 'failed':
            try: s.append(float(v))
            except: pass
    if not s or 'avg_power' not in d:
        return None
    return dict(n=len(s),
                skew=(max(s)-min(s))*1e12,  # worst-case skew (max - min)
                ins=statistics.mean(s)*1e12,
                pw=float(d['avg_power'])*1e3)

def sink_tsvs(buf, f):
    log = os.path.join(JDIR, buf, f"fmax{f}", "openroad.log")
    if not os.path.exists(log):
        return None
    n = None
    for l in open(log, errors='ignore'):
        m = re.search(r'placed\s+(\d+)\s+sink-taps', l)
        if m: n = int(m.group(1))
    return n

# frontside reference
fs = parse_mt0(os.path.join(JDIR, "Frontside", "jpeg_frontside.mt0"))

rows = []       # (buffer, fmax, tsvs, skew, ins, pw, dskew, dpow)
for b in BUFS:
    for f in FMAXS:
        r = parse_mt0(os.path.join(JDIR, b, f"fmax{f}", "jpeg_backside.mt0"))
        if r is None:
            continue
        tsv = sink_tsvs(b, f)
        # improvement %: positive = better than FS (lower skew / lower power)
        dskew = (fs['skew'] - r['skew']) / fs['skew'] * 100
        dpow  = (fs['pw']   - r['pw'])  / fs['pw']   * 100
        rows.append([b, f"fmax{f}", tsv, r['skew'], r['ins'], r['pw'], dskew, dpow])

# ---- render ----
headers = ["Buffer", "F_max", "Sink TSVs", "Skew (ps)\nworst-case", "Insertion (ps)",
           "Power (mW)", "Skew vs FS", "Power vs FS"]
GREEN = "#c8e6c9"; RED = "#ffcdd2"; HEAD = "#455a64"; FSROW = "#fff3c4"

cells, cellcolors = [], []
for r in rows:
    b, fm, tsv, sk, ins, pw, dsk, dpw = r
    sk_txt = f"{'+' if dsk>=0 else ''}{dsk:.1f}%"
    pw_txt = f"{'+' if dpw>=0 else ''}{dpw:.1f}%"
    cells.append([b, fm, str(tsv), f"{sk:.1f}", f"{ins:.1f}", f"{pw:.3f}",
                  sk_txt, pw_txt])
    row_c = ["white"]*6 + [GREEN if dsk >= 0 else RED,
                           GREEN if dpw >= 0 else RED]
    cellcolors.append(row_c)
# FS reference row
cells.append(["FS (ref)", "frontside", "0", f"{fs['skew']:.1f}",
              f"{fs['ins']:.1f}", f"{fs['pw']:.3f}", "--", "--"])
cellcolors.append([FSROW]*8)

fig, ax = plt.subplots(figsize=(11, 0.42*len(cells)+1.2))
ax.axis("off")
tbl = ax.table(cellText=cells, colLabels=headers, cellColours=cellcolors,
               cellLoc="center", loc="center")
tbl.auto_set_font_size(False); tbl.set_fontsize(9); tbl.scale(1, 1.5)
for j in range(len(headers)):
    c = tbl[0, j]; c.set_facecolor(HEAD); c.set_text_props(color="white", fontweight="bold")

# Merge the Buffer column vertically per group of len(FMAXS) rows: keep the
# label only on the group's middle row, blank the rest, and drop the internal
# horizontal borders so each buffer reads as one merged cell. (table row 0 is
# the header, so data rows start at 1; the last row is the FS reference.)
g = len(FMAXS)
n_backside = len(rows)
for start in range(0, n_backside, g):
    label = cells[start][0]
    mid = start + g // 2                       # row that keeps the label
    for k in range(start, start + g):
        r = k + 1                              # +1 for header row
        cell = tbl[r, 0]
        cell.get_text().set_text(label if k == mid else "")
        if k == start:
            cell.visible_edges = "LTR"         # top of the merged box
        elif k == start + g - 1:
            cell.visible_edges = "LBR"         # bottom of the merged box
        else:
            cell.visible_edges = "LR"          # interior: no horizontal line
        cell.set_text_props(fontweight="bold")
ax.set_title("JPEG backside clock mesh — sink-buffer x F_max sweep\n"
             "worst-case skew (max-min); green = better than frontside, red = worse; "
             "FS = frontside M5/M6 mesh",
             fontsize=11, pad=14)
out = os.path.join(JDIR, "jpeg_bufsweep_table.pdf")
plt.savefig(out, bbox_inches="tight"); print("wrote", out)
print(f"FS ref: skew={fs['skew']:.1f}ps insertion={fs['ins']:.1f}ps power={fs['pw']:.3f}mW sinks={fs['n']}")
