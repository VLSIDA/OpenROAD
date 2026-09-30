#!/bin/bash
# JPEG backside sweep WITH -remove_colliding_wires (drops mesh drivers sitting on
# power straps -> fewer drivers -> lower power). Uses jpeg_backside_rmcoll.tcl and
# writes into Jpeg/rmcoll/<buf>/fmax<F>/ so it never touches the baseline sweep.
# Usage:  ./run_jpeg_rmcoll.sh                 # all buffers x fmax {8,16,24,80}
#         ./run_jpeg_rmcoll.sh x4 16           # single config
set -u
ORD=/home/wali2/backside/OpenROAD/build/bin/openroad
JDIR=/home/wali2/backside/OpenROAD/src/cms/test/backside_result/Jpeg
BK=$JDIR/jpeg_backside_rmcoll.tcl

if [ $# -eq 2 ]; then BUFS="$1"; FMAXS="$2"; else BUFS="x4 x6 x8 x10 x12"; FMAXS="8 16 24 80"; fi

for b in $BUFS; do
  master="gt2_6t_buf_${b}_w31_lvt"
  for f in $FMAXS; do
    out=$JDIR/rmcoll/$b/fmax$f; mkdir -p "$out"
    echo "======== [rmcoll] $b/fmax$f ========"
    SBUF=$master FMAX=$f OUTDIR="$out" $ORD -threads 72 -exit "$BK" 2>&1 | tee "$out/openroad.log"
    if grep -q "DRT-0206\|^\[ERROR\|ERROR:" "$out/openroad.log"; then
      echo ">>> [rmcoll] $b/fmax$f OpenROAD FAILED"; continue
    fi
    ( cd "$out" && hspice jpeg_backside.sp -mt 48 -o jpeg_backside 2>&1 | tee hspice.log )
    echo "  CMS-0733(drivers dropped): $(grep 'CMS-0733' "$out/openroad.log" | tail -1)"
    echo "  bridges:$(grep -c '^Rbridgesink_tap_' "$out/jpeg_backside.sp") shorts:$(grep -c 'marker -- src: sink_tap' "$out/openroad.log")"
  done
done
echo "RMCOLL DONE"
