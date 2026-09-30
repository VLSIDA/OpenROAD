#!/bin/bash
# JPEG backside mesh DENSITY sweep: fix sink=x4, F_max=8, LOW feeder; sweep the
# mesh PITCH from sparse->dense. Answers "how dense can we go" — reports grid
# size, driver/TSV count, routing status, sink skew + power per pitch.
# Results -> Jpeg/density/pitch<P>/
# Usage:  ./run_jpeg_density_sweep.sh                 # default pitch list
#         ./run_jpeg_density_sweep.sh 1.6             # single pitch
set -u
ORD=/home/wali2/backside/OpenROAD/build/bin/openroad
JDIR=/home/wali2/backside/OpenROAD/src/cms/test/backside_result/Jpeg
BK=$JDIR/jpeg_backside_baltree.tcl
export FEEDER=low
SBUF=gt2_6t_buf_x4_w31_lvt; FMAX=8

if [ $# -ge 1 ]; then PITCHES="$*"; else PITCHES="3.2 2.4 1.6 1.2 0.8"; fi

for p in $PITCHES; do
  tag=$(echo "$p" | tr '.' 'p')
  out=$JDIR/density/pitch$tag; mkdir -p "$out"
  echo "======== [density] pitch=$p um -> density/pitch$tag ========"
  PITCH=$p SBUF=$SBUF FMAX=$FMAX OUTDIR="$out" $ORD -threads 128 -exit "$BK" 2>&1 | tee "$out/openroad.log"
  # quick per-pitch summary
  echo "  grid:   $(grep 'CMS-0121' "$out/openroad.log" | tail -1 | grep -oE '[0-9]+ H-lines x [0-9]+ V-lines')"
  echo "  drivers:$(grep 'CMS-0610' "$out/openroad.log" | grep -oE '[0-9]+ proxy' | grep -oE '[0-9]+')  dropped-wires:$(grep -oE '[0-9]+ dropped' "$out/openroad.log" | grep -oE '^[0-9]+' | paste -sd+ | bc 2>/dev/null)"
  if grep -qE "DRT-0206|checkConnectivity break|^\[ERROR|ERROR:" "$out/openroad.log"; then
    echo ">>> [density] pitch=$p FAILED (route/error) — likely the density limit"; continue
  fi
  ( cd "$out" && hspice jpeg_backside.sp -mt 48 -o jpeg_backside 2>&1 | tee hspice.log >/dev/null )
done

echo ""
echo "######## DENSITY SWEEP SUMMARY ########"
python3 "$JDIR/parse_jpeg_density.py"
echo "DENSITY SWEEP DONE"
