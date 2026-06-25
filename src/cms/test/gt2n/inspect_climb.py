#!/usr/bin/env openroad -python
# Why do only b_clk_buf_11200_1120 / _15232_1120 climb while the other 62 stay
# on the backside? Dump the routed layers of every crossing net, then detail the
# 2 climbers vs a good one, and show what mesh metal exists near each node.
import odb
import re

DB = "results/gcd_tsv_routed.odb"

db = odb.dbDatabase.create()
odb.read_db(db, DB)
block = db.getChip().getBlock()
tech = db.getTech()
dbu = block.getDbUnitsPerMicron()

die = block.getDieArea()
core = block.getCoreArea()
print(f"die  = ({die.xMin()},{die.yMin()})-({die.xMax()},{die.yMax()})")
print(f"core = ({core.xMin()},{core.yMin()})-({core.xMax()},{core.yMax()})")
rows = list(block.getRows())
if rows:
    ys = sorted(set(r.getOrigin()[1] for r in rows))
    print(f"rows: {len(rows)}  y range = {ys[0]} .. {ys[-1]}  "
          f"first-rows={ys[:4]}")
print()


def is_backside(name):
    L = tech.findLayer(name)
    return L.isBackside() if L else None


def net_layers(net):
    layers = set()
    w = net.getWire()
    if w:
        dec = odb.dbWireDecoder()
        dec.begin(w)
        op = dec.next()
        while op != odb.dbWireDecoder.END_DECODE:
            try:
                L = dec.getLayer()
            except Exception:
                L = None
            if L is not None:
                layers.add(L.getName())
            op = dec.next()
    return layers


print("=== every b_clk_buf net: routed layers ===")
bad = []
for net in block.getNets():
    n = net.getName()
    if not n.startswith("b_clk_buf_"):
        continue
    Ls = net_layers(net)
    front = [x for x in Ls if is_backside(x) is False]
    if front:
        bad.append(n)
    tag = "   <<< CLIMBS (frontside layers!)" if front else ""
    print(f"  {n:30s} {sorted(Ls)}{tag}")

print(f"\nclimbers ({len(bad)}): {bad}")

# pick one clean net for comparison
good = None
for net in block.getNets():
    n = net.getName()
    if n.startswith("b_clk_buf_") and n not in bad:
        good = n
        break

targets = ["b_clk_buf_11200_1120", "b_clk_buf_15232_1120"]
if good:
    targets.append(good)

mesh = block.findNet("clk_mesh")
win = int(2 * dbu)  # 2um window around the node


def dump_net(tn):
    net = block.findNet(tn)
    print(f"\n=== {tn} ===")
    if not net:
        print("  (net not found)")
        return
    for it in net.getITerms():
        bb = it.getBBox()
        print(f"  ITerm {it.getInst().getName()}/{it.getMTerm().getName()} "
              f"({bb.xMin()},{bb.yMin()})-({bb.xMax()},{bb.yMax()})")
    for bt in net.getBTerms():
        for bp in bt.getBPins():
            for b in bp.getBoxes():
                print(f"  BTerm {bt.getName()} {b.getTechLayer().getName()} "
                      f"({b.xMin()},{b.yMin()})-({b.xMax()},{b.yMax()})")
    print(f"  routed layers: {sorted(net_layers(net))}")
    w = net.getWire()
    if w:
        try:
            bb = w.getBBox()
            outside = (bb.xMin() < core.xMin() or bb.yMin() < core.yMin()
                       or bb.xMax() > core.xMax() or bb.yMax() > core.yMax())
            tag = "   *** route extends OUTSIDE core ***" if outside else ""
            print(f"  route bbox: ({bb.xMin()},{bb.yMin()})-"
                  f"({bb.xMax()},{bb.yMax()}){tag}")
        except Exception as e:
            print(f"  route bbox: (unavailable: {e})")
    # mesh metal near the node coords parsed from the name
    m = re.findall(r"_(\d+)_(\d+)$", tn)
    if m and mesh:
        x, y = int(m[0][0]), int(m[0][1])
        print(f"  node coords x={x} y={y}; clk_mesh stripes within 2um:")
        found = 0
        for sw in mesh.getSWires():
            for b in sw.getWires():
                cx = (b.xMin() + b.xMax()) // 2
                cy = (b.yMin() + b.yMax()) // 2
                if abs(cx - x) < win and abs(cy - y) < win:
                    print(f"      {b.getTechLayer().getName():5s} "
                          f"({b.xMin()},{b.yMin()})-({b.xMax()},{b.yMax()})")
                    found += 1
        if not found:
            print("      *** NO clk_mesh metal within 2um of this node ***")


for tn in targets:
    dump_net(tn)
