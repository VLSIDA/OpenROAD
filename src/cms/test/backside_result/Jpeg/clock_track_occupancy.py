#!/usr/bin/env python3
# Per-layer CLOCK track occupancy: how much of each routing layer's capacity the
# clock network consumes. Works on any routed odb (FS mesh / BS mesh / CTS).
# Metric per layer:
#   clock_len(um)  = total clock wire length on that layer (regular + special)
#   tracks         = routable tracks on the layer (core_perp / track_pitch)
#   capacity(um)   = tracks * core_parallel_span  (total track-length available)
#   occupancy%     = clock_len / capacity * 100   (fraction of tracks the clock eats)
# Usage: openroad -exit -python clock_track_occupancy.py <odb> [<odb> ...]
import sys, glob
from openroad import Tech, Design
import odb

LIBDIR="/home/wali2/backside/OpenROAD-flow-scripts/flow/platforms/gt2n/lib"

def is_clock_net(n):
    try:
        if n.getSigType()=="CLOCK": return True
    except: pass
    nm=n.getName().lower()
    return ("clk" in nm) or ("mesh" in nm)

def analyze(odbpath):
    tech=Tech()
    for f in sorted(glob.glob(LIBDIR+"/*.lib")): tech.readLiberty(f)
    d=Design(tech); d.readDb(odbpath); b=d.getBlock()
    dbu=b.getDbUnitsPerMicron()
    core=b.getCoreArea()
    cw=(core.xMax()-core.xMin())/dbu; ch=(core.yMax()-core.yMin())/dbu
    t=d.getTech().getDB().getTech()

    # per-layer clock wire length (dbu) — regular (dbWire) + special (dbSWire)
    clk_len={}   # layer name -> length in dbu
    def add(layer, L):
        clk_len[layer]=clk_len.get(layer,0)+L
    for n in b.getNets():
        if not is_clock_net(n): continue
        # regular routed wire
        w=n.getWire()
        if w:
            dec=odb.dbWireDecoder(); dec.begin(w)
            op=dec.next(); lay=None; px=py=None
            while op!=odb.dbWireDecoder.END_DECODE:
                if op==odb.dbWireDecoder.PATH or op==odb.dbWireDecoder.JUNCTION or op==odb.dbWireDecoder.SHORT:
                    try: lay=dec.getLayer().getName()
                    except: lay=None
                    px=py=None
                elif op in (odb.dbWireDecoder.POINT, odb.dbWireDecoder.POINT_EXT):
                    pt=dec.getPoint(); x,y=pt[0],pt[1]
                    if px is not None and lay:
                        add(lay, abs(x-px)+abs(y-py))
                    px,py=x,y
                op=dec.next()
        # special wire (mesh, PDN-like)
        for sw in n.getSWires():
            for sb in sw.getWires():
                if sb.isVia(): continue
                try: lay=sb.getTechLayer().getName()
                except: continue
                dx=sb.xMax()-sb.xMin(); dy=sb.yMax()-sb.yMin()
                add(lay, max(dx,dy))   # length along the wire

    print(f"\n=== {odbpath.split('/')[-2]}  ({odbpath.split('/')[-1]})  core {cw:.1f}x{ch:.1f}um ===")
    print(f"{'layer':>6} {'dir':>4} {'pitch(um)':>9} {'clk_len(um)':>11} {'tracks':>7} {'cap(um)':>9} {'occ%':>7}")
    for L in t.getLayers():
        if L.getType()!="ROUTING": continue
        nm=L.getName()
        length=clk_len.get(nm,0)/dbu
        d_=L.getDirection()
        # track pitch: use layer pitch; capacity = tracks * span-in-routing-dir
        p=L.getPitch()/dbu if L.getPitch()>0 else (L.getWidth()*2/dbu)
        if d_=="HORIZONTAL":
            tracks=ch/p if p>0 else 0; cap=tracks*cw
        elif d_=="VERTICAL":
            tracks=cw/p if p>0 else 0; cap=tracks*ch
        else:
            tracks=0; cap=0
        occ=(length/cap*100) if cap>0 else 0
        if length>0.01 or nm in ("M5","M6","BM1","BM2"):
            print(f"{nm:>6} {str(d_)[:4]:>4} {p:9.3f} {length:11.1f} {tracks:7.0f} {cap:9.0f} {occ:7.2f}")

for p in sys.argv[1:]:
    analyze(p)
