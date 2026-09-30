# openroad -python: dump pin shape boxes for all tree-net ITerms + root BTerm
import odb
IDIR = "/home/wali2/backside/OpenROAD/src/cms/test/backside_result/Ibex/CTS"
db = odb.dbDatabase.create()
odb.read_db(db, f"{IDIR}/ibex_cts_routed.odb")
block = db.getChip().getBlock()
root = block.findBTerm("clk_i").getNet()
nets = [root.getName()]
for inst in block.getInsts():
    if not inst.getName().startswith("clkbuf_"):
        continue
    for it in inst.getITerms():
        n = it.getNet()
        if n and it.getMTerm().getIoType() == "OUTPUT":
            nets.append(n.getName())
with open(f"{IDIR}/cts_pinboxes.csv", "w") as f:
    for nname in nets:
        net = block.findNet(nname)
        for it in net.getITerms():
            for layer, rect in it.getGeometries():
                f.write(f"{it.getName()}|{nname}|{layer.getName()}|{rect.xMin()}|{rect.yMin()}|{rect.xMax()}|{rect.yMax()}\n")
        for bt in net.getBTerms():
            for bp in bt.getBPins():
                for bx in bp.getBoxes():
                    f.write(f"BTERM/{bt.getName()}|{nname}|{bx.getTechLayer().getName()}|{bx.xMin()}|{bx.yMin()}|{bx.xMax()}|{bx.yMax()}\n")
print("wrote cts_pinboxes.csv")
