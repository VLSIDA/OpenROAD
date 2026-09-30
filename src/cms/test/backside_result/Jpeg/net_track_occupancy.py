#!/usr/bin/env python3
# Per-layer track occupancy for ALL nets (routing utilization from real geometry,
# regular dbWire + special dbSWire). Splits clock vs signal so you see the clock's
# share and the total. Works on any routed odb.
#   len(um)     = total wire length on the layer (all nets)
#   clk(um)     = clock-net portion
#   tracks/cap  = routable tracks and total track-length capacity
#   occ%        = total_len / capacity * 100  (layer routing utilization)
# Usage: openroad -exit -python net_track_occupancy.py <odb> [<odb> ...]
import sys, glob
from openroad import Tech, Design
import odb
LIBDIR="/home/wali2/backside/OpenROAD-flow-scripts/flow/platforms/gt2n/lib"

def is_clock(n):
    try:
        if n.getSigType()=="CLOCK": return True
    except: pass
    nm=n.getName().lower(); return ("clk" in nm) or ("mesh" in nm)

def wire_len_by_layer(n, tot, clk):
    dst = clk if is_clock(n) else None
    def add(layer,L):
        tot[layer]=tot.get(layer,0)+L
        if dst is not None: dst[layer]=dst.get(layer,0)+L
    w=n.getWire()
    if w:
        dec=odb.dbWireDecoder(); dec.begin(w)
        op=dec.next(); lay=None; px=py=None
        while op!=odb.dbWireDecoder.END_DECODE:
            if op in (odb.dbWireDecoder.PATH,odb.dbWireDecoder.JUNCTION,odb.dbWireDecoder.SHORT):
                try: lay=dec.getLayer().getName()
                except: lay=None
                px=py=None
            elif op in (odb.dbWireDecoder.POINT,odb.dbWireDecoder.POINT_EXT):
                pt=dec.getPoint(); x,y=pt[0],pt[1]
                if px is not None and lay: add(lay,abs(x-px)+abs(y-py))
                px,py=x,y
            op=dec.next()
    for sw in n.getSWires():
        for sb in sw.getWires():
            if sb.isVia(): continue
            try: lay=sb.getTechLayer().getName()
            except: continue
            add(lay, max(sb.xMax()-sb.xMin(), sb.yMax()-sb.yMin()))

def analyze(odbpath):
    tech=Tech()
    for f in sorted(glob.glob(LIBDIR+"/*.lib")): tech.readLiberty(f)
    d=Design(tech); d.readDb(odbpath); b=d.getBlock()
    dbu=b.getDbUnitsPerMicron(); core=b.getCoreArea()
    cw=(core.xMax()-core.xMin())/dbu; ch=(core.yMax()-core.yMin())/dbu
    t=d.getTech().getDB().getTech()
    tot={}; clk={}
    for n in b.getNets(): wire_len_by_layer(n,tot,clk)
    print(f"\n=== {odbpath.split('/')[-2]} ({odbpath.split('/')[-1]}) core {cw:.1f}x{ch:.1f}um ===")
    print(f"{'layer':>6} {'dir':>4} {'total(um)':>10} {'clk(um)':>9} {'tracks':>7} {'cap(um)':>9} {'occ%':>7}")
    for L in t.getLayers():
        if L.getType()!="ROUTING": continue
        nm=L.getName(); length=tot.get(nm,0)/dbu; cl=clk.get(nm,0)/dbu
        dirn=L.getDirection(); p=L.getPitch()/dbu if L.getPitch()>0 else L.getWidth()*2/dbu
        if dirn=="HORIZONTAL": tracks=ch/p if p>0 else 0; cap=tracks*cw
        elif dirn=="VERTICAL": tracks=cw/p if p>0 else 0; cap=tracks*ch
        else: tracks=0; cap=0
        occ=(length/cap*100) if cap>0 else 0
        if length>0.01 or nm in ("M5","M6","BM1","BM2"):
            print(f"{nm:>6} {str(dirn)[:4]:>4} {length:10.1f} {cl:9.1f} {tracks:7.0f} {cap:9.0f} {occ:7.2f}")

for p in sys.argv[1:]: analyze(p)
