#!/bin/bash
# JPEG sink-buffer sweep: buffer {x6,x8,x10,x12} x F_max {8,16,24,80} = 16 runs.
# Mesh drivers stay x4; only the SINK buffer (create_sink_taps -buffer) varies.
# Backside mesh, pitch 3.2, balanced PDN, 6ohm TSV. Each combo -> its own folder:
#   Jpeg/<buf>/fmax<F>/  (e.g. Jpeg/x8/fmax16/)
# Overnight-safe: continues on failure, logs everything, parses at the end.
set -u
ORD=/home/wali2/backside/OpenROAD/build/bin/openroad
JDIR=/home/wali2/backside/OpenROAD/src/cms/test/backside_result/Jpeg
BK=$JDIR/jpeg_backside_param.tcl

BUFS="x4 x6 x8 x10 x12"
FMAXS="8 16 24 80"

for b in $BUFS; do
  master="gt2_6t_buf_${b}_w31_lvt"
  for f in $FMAXS; do
    out=$JDIR/$b/fmax$f
    mkdir -p "$out"
    echo "======== SINK BUF=$b (F_max=$f) -> $b/fmax$f ========"
    SBUF=$master FMAX=$f OUTDIR="$out" $ORD -threads 72 -exit "$BK" \
        2>&1 | tee "$out/openroad.log"
    if grep -q "DPL-0036\|^\[ERROR\|ERROR:" "$out/openroad.log"; then
      echo ">>> [$b/fmax$f] OpenROAD FAILED (see $out/openroad.log)"
      grep -E "DPL-0036|CMS-0703" "$out/openroad.log" | tail -2
      continue
    fi
    grep -E "CMS-0703" "$out/openroad.log" | tail -1
    echo ">>> [$b/fmax$f] running HSpice (-mt 48) ..."
    ( cd "$out" && hspice jpeg_backside.sp -mt 48 -o jpeg_backside 2>&1 | tee hspice.log )
    if grep -q "hspice job concluded" "$out/hspice.log"; then
      echo ">>> [$b/fmax$f] hspice OK"
    else
      echo ">>> [$b/fmax$f] hspice FAILED"
    fi
  done
done

echo ""
echo "######## PARSING BUFFER SWEEP ########"
python3 "$JDIR/parse_jpeg_bufsweep.py"
echo "BUFSWEEP ALL DONE"
