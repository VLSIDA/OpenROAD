#!/bin/bash
# IBEX FRONTSIDE (M5/M6) mesh at PITCH=1.6 -- ONE config.
# The frontside has NO sink-buffer / F_max tier: FFs connect directly to the mesh
# (connect_sinks_to_mesh), so SBUF/FMAX do not apply. This is the single FS
# reference point to compare against the backside sink-buffer sweep.
# Result -> Ibex/frontside_p1p6/ibex_frontside.{sp,lis,odb}
set -u
ORD=/home/wali2/backside/OpenROAD/build/bin/openroad
JDIR=/home/wali2/backside/OpenROAD/src/cms/test/backside_result/Ibex
FS=/home/wali2/backside/OpenROAD/src/cms/test/backside_result/multi_frontside_baltree.tcl

export FEEDER=full DESIGN=ibex PITCH=1.6
out=$JDIR/frontside_p1p6; mkdir -p "$out"

echo "======== FS ibex pitch 1.6 (direct-to-mesh, full feeder) ========"
OUTDIR="$out" $ORD -threads 128 -exit "$FS" 2>&1 | tee "$out/openroad.log"
if grep -qE "DRT-0206|checkConnectivity break|^\[ERROR|ERROR:" "$out/openroad.log"; then
  echo ">>> FS ibex p1.6 OpenROAD FAILED"; exit 1
fi
( cd "$out" && hspice ibex_frontside.sp -mt 48 -o ibex_frontside 2>&1 | tee hspice.log )
echo "FRONTSIDE IBEX p1.6 DONE -> $out"
