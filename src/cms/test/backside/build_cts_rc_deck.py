#!/usr/bin/env python3
"""Build the distributed-RC Ibex CTS tree deck from DEF wiring (ground truth).

Full clock-network fidelity:
  - per-segment wire R (per-layer ohm/um from setRC.tcl)
  - pi-model wire C (half to each segment end)
  - VIA RESISTANCE (per-via ohm from setRC.tcl), layer-aware nodes
  - per-FF sink caps + measures (all 1938 FFs), inverter loads as caps
  - branches connected through pin metal (layer-aware pin-box ties)
  - transistor-level buffers (CDL subckts)

Inputs (export_cts_data.tcl + export_pinboxes.py in the CTS results dir):
  ibex_cts.def, cts_pins.csv, cts_bufs.csv, cts_pinboxes.csv
Output: ibex_cts_rc.sp
"""
import re
import sys
from collections import defaultdict
from pathlib import Path

IDIR = Path("/home/wali2/backside/OpenROAD/src/cms/test/backside_result/Ibex/CTS")
HERE = Path("/home/wali2/backside/OpenROAD/src/cms/test/backside")
GT2N = Path("/home/wali2/backside/GT2N")
SETRC = Path("/home/wali2/backside/OpenROAD-flow-scripts/flow/platforms/gt2n/setRC.tcl")
DBU = 2000.0

# ---- per-layer R (ohm/um), C (F/um); per-via R (ohm) ----
rum, cum, rvia = {}, {}, {}
for line in SETRC.read_text().splitlines():
    m = re.match(r"set_layer_rc\s+-layer\s+(\S+)\s+-resistance\s+(\S+)\s+-capacitance\s+(\S+)", line)
    if m:
        rum[m.group(1)] = float(m.group(2))
        cum[m.group(1)] = float(m.group(3)) * 1e-12
    m = re.match(r"set_layer_rc\s+-via\s+(\S+)\s+-resistance\s+(\S+)", line)
    if m:
        rvia[m.group(1)] = float(m.group(2))

def via_layers(vname):
    m = re.match(r"(B?)V(\d+)", vname)
    if not m:
        return None
    k = int(m.group(2))
    pre = "BM" if m.group(1) else "M"
    return f"{pre}{k}", f"{pre}{k+1}", f"{m.group(1)}V{k}"

# ---- inputs ----
bufs = [l.split("|") for l in (IDIR / "cts_bufs.csv").read_text().splitlines()]
pins = [l.split("|") for l in (IDIR / "cts_pins.csv").read_text().splitlines()]
nets = [(IDIR / "cts_root.txt").read_text().split("\n")[0].split("|")[0]]
root_name = nets[0]
nets += [b[2] for b in bufs]
netset = set(nets)
net_idx = {n: i for i, n in enumerate(nets)}

# ---- parse DEF wiring: segments (layer-aware) + vias ----
def_text = (IDIR / "ibex_cts.def").read_text()
nets_sec = def_text.split("\nNETS ")[1].split("\nEND NETS")[0]
segs = defaultdict(list)   # net -> [(layer, x1,y1,x2,y2)]
vias = defaultdict(list)   # net -> [(vianame, x, y)]
for st in nets_sec.split("\n    - ")[1:]:
    nname = st.split("\n")[0].split()[0].replace("\\", "")
    if nname not in netset:
        continue
    body = st.replace("\n", " ")
    mroute = re.search(r"\+ ROUTED (.*?)(?:\+ (?:USE|PROPERTY)|;)", body)
    if not mroute:
        continue
    rawsegs, pts = [], defaultdict(set)   # pts[layer] = {(x,y)}
    for chunk in re.split(r"\bNEW\b", mroute.group(1)):
        toks = chunk.split()
        if not toks:
            continue
        layer = toks[0]
        i, px, py = 1, None, None
        while i < len(toks):
            if toks[i] == "(":
                x = px if toks[i+1] == "*" else int(toks[i+1])
                y = py if toks[i+2] == "*" else int(toks[i+2])
                j = i + 3
                if j < len(toks) and toks[j] != ")":
                    j += 1
                assert toks[j] == ")", f"bad point in {nname}"
                pts[layer].add((x, y))
                if px is not None and (x != px or y != py):
                    rawsegs.append((layer, px, py, x, y))
                px, py = x, y
                i = j + 1
            elif toks[i] == "RECT":
                i += 6
            elif toks[i] == "TAPER":
                i += 1
            else:  # via name at current point
                vl = via_layers(toks[i])
                if vl and px is not None:
                    vias[nname].append((toks[i], px, py))
                    pts[vl[0]].add((px, py))
                    pts[vl[1]].add((px, py))
                i += 1
    # split segments at every same-layer net point on them (T-junctions, vias)
    for (layer, x1, y1, x2, y2) in rawsegs:
        P = pts[layer]
        if y1 == y2:
            lo, hi = sorted((x1, x2))
            cuts = sorted({x for (x, y) in P if y == y1 and lo <= x <= hi} | {lo, hi})
            segs[nname] += [(layer, a, y1, b, y1) for a, b in zip(cuts, cuts[1:])]
        elif x1 == x2:
            lo, hi = sorted((y1, y2))
            cuts = sorted({y for (x, y) in P if x == x1 and lo <= y <= hi} | {lo, hi})
            segs[nname] += [(layer, x1, a, x1, b) for a, b in zip(cuts, cuts[1:])]
        else:
            segs[nname].append((layer, x1, y1, x2, y2))

# ---- nodes (net, x, y, layer), wire R, via R, wire C ----
netnodes = defaultdict(list)     # net -> [(node, x, y, layer)]
node_seen = set()
nodecap = defaultdict(float)
rlines = []
k = 0

def node(nname, x, y, layer):
    nd = f"n{net_idx[nname]}_{x}_{y}_{layer}"
    if nd not in node_seen:
        node_seen.add(nd)
        netnodes[nname].append((nd, x, y, layer))
    return nd

for nname in nets:
    for (layer, x1, y1, x2, y2) in segs[nname]:
        if layer not in rum:
            sys.exit(f"no RC for layer {layer}")
        n1, n2 = node(nname, x1, y1, layer), node(nname, x2, y2, layer)
        lum = (abs(x2 - x1) + abs(y2 - y1)) / DBU
        rlines.append(f"Rw{k} {n1} {n2} {lum * rum[layer]:.5g}")
        half = lum * cum[layer] / 2.0
        nodecap[n1] += half
        nodecap[n2] += half
        k += 1
    for (vname, x, y) in vias[nname]:
        lb, lt, rkey = via_layers(vname)
        if rkey not in rvia:
            sys.exit(f"no via R for {vname}")
        n1, n2 = node(nname, x, y, lb), node(nname, x, y, lt)
        rlines.append(f"Rv{k} {n1} {n2} {rvia[rkey]:.5g}")
        k += 1

# ---- pin boxes (layer-aware): tie wire nodes inside a pin's shapes ----
TOL = 2
pinboxes = defaultdict(list)  # (pinname, net) -> [(layer, box)]
for l in (IDIR / "cts_pinboxes.csv").read_text().splitlines():
    pn, nname, layer, x1, y1, x2, y2 = l.split("|")
    pinboxes[(pn, nname)].append((layer, int(x1)-TOL, int(y1)-TOL,
                                  int(x2)+TOL, int(y2)+TOL))
pin_tie_node = {}
tie_lines = []
tk = 0
for (pn, nname), boxes in pinboxes.items():
    hits = []
    for nd, nx, ny, nl in netnodes[nname]:
        for (bl, x1, y1, x2, y2) in boxes:
            if nl == bl and x1 <= nx <= x2 and y1 <= ny <= y2:
                hits.append(nd)
                break
    if not hits:
        continue
    pin_tie_node[(pn, nname)] = hits[0]
    for nd in hits[1:]:
        tie_lines.append(f"Rp{tk} {hits[0]} {nd} 0.01")
        tk += 1
print(f"pin ties: {tk}; pins hit by wire: {len(pin_tie_node)} / {len(pinboxes)}")

def nearest(nname, x, y):
    best, bd = None, 1e30
    for nd, nx, ny, nl in netnodes[nname]:
        d = abs(nx - int(x)) + abs(ny - int(y))
        if d < bd:
            bd, best = d, nd
    return best

# ---- bind pins ----
pin_node = {}
sinks, loads = [], []
fallback = 0
for inst, nname, ax, ay, cap, kind in pins:
    pname = None
    for (pn, nn2) in pin_tie_node:
        if nn2 == nname and pn.startswith(inst + "/"):
            pname = (pn, nn2)
            break
    if pname:
        nd = pin_tie_node[pname]
    else:
        nd = nearest(nname, ax, ay)
        fallback += 1
    if kind == "ff":
        sinks.append((nd, inst, float(cap)))
    elif kind == "load":
        loads.append((nd, float(cap)))
    else:
        pin_node[(inst, kind)] = nd
root_node = pin_tie_node.get(("BTERM/clk_i", root_name))
if root_node is None:
    rx, ry = (IDIR / "cts_root.txt").read_text().split("\n")[0].split("|")[1:3]
    root_node = nearest(root_name, rx, ry)
    fallback += 1
print(f"pins bound: {len(pins)} ({fallback} by nearest-node fallback)")

# ---- connectivity check (fail loud) ----
adj = defaultdict(set)
for l in rlines + tie_lines:
    t = l.split()
    adj[t[1]].add(t[2]); adj[t[2]].add(t[1])
bad = 0
for nname in nets:
    nds = [e[0] for e in netnodes[nname]]
    seen = set()
    stack = [nds[0]]
    while stack:
        u = stack.pop()
        if u in seen:
            continue
        seen.add(u)
        stack += list(adj[u] - seen)
    if len(seen) != len(nds):
        bad += 1
if bad:
    sys.exit(f"FATAL: {bad} nets fragmented")
print("connectivity: all nets single-component")

# ---- emit ----
out = IDIR / "ibex_cts_rc.sp"
nvia = sum(len(v) for v in vias.values())
with open(out, "w") as f:
    w = f.write
    w("* CTS clock-tree SPICE deck v3 (ibex / gt2n) -- DISTRIBUTED wire RC + VIA R\n")
    w(f"* {k} R elements ({nvia} vias), pi-model C, per-FF sinks, pin-metal ties\n")
    w(".option rshunt=1e12\n.option method=gear\n")
    w(".option abstol=1e-10 reltol=0.003 vntol=1e-4\n")
    w(".option delmax=10p\n.option autostop\n.option measdgt=7\n")
    w(".param mc_mm_switch=0\n.param mc_pr_switch=0\n")
    w(f".include {HERE}/gt2_w31_lvt_tt_renamed.sp\n")
    w(f".include {GT2N}/cdl/gt2_6t_w31_lvt.cdl\n\n")
    w("Vvdd VDD 0 0.7\n")
    w(f"Vclk {root_node} 0 PULSE(0 0.7 0.1n 0.01n 0.01n 0.5n 1.0n)\n\n")
    w("* --- clock tree buffers ---\n")
    for iname, bin_, bout, master in bufs:
        a, y = pin_node.get((iname, "bufA")), pin_node.get((iname, "bufY"))
        if a is None or y is None:
            sys.exit(f"unbound buffer {iname}")
        w(f"X{iname} {a} {y} VDD 0 {master}\n")
    w("\n* --- wire + via resistance ---\n")
    for l in rlines:
        w(l + "\n")
    w("\n* --- pin-metal ties ---\n")
    for l in tie_lines:
        w(l + "\n")
    w("\n* --- wire capacitance (pi halves per node) ---\n")
    for i, (nd, c) in enumerate(sorted(nodecap.items())):
        w(f"Cw{i} {nd} 0 {c:.6g}\n")
    w("\n* --- non-FF clock loads ---\n")
    for i, (nd, c) in enumerate(loads):
        w(f"Cload{i} {nd} 0 {c:.6g}\n")
    w("\n* --- FF clock-pin caps (per sink) ---\n")
    for i, (nd, ff, c) in enumerate(sinks):
        w(f"Csink{i} {nd} 0 {c:.6g} $ {ff}\n")
    w("\n.tran 0.005n 2.1n\n")
    for i, (nd, ff, c) in enumerate(sinks):
        w(f".measure tran t_sink_{i} WHEN v({nd})=0.35 RISE=2\n")
        w(f".measure tran slew_sink_{i} TRIG v({nd})=0.07 RISE=2 TARG v({nd})=0.63 RISE=2\n")
    w(".measure tran avg_power AVG power FROM=1.1n TO=2.1n\n.end\n")

print(f"R={k} (vias {nvia})  Cnodes={len(nodecap)}  sinks={len(sinks)} loads={len(loads)}")
print(f"wrote {out}")
