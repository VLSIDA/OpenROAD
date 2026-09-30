#!/bin/bash
# JPEG backside BALANCED-FEEDER sweep. Feeder variant selectable:
#   full -> LVT x1..x12    low -> LVT x2/x3/x4   (single VT, one corner)
# Per-buffer F_max grid (max F_max capped to each sink buffer's drive budget):
#   x1: 8 16 20   x2: 6 16 24 40   x3: 8 16 24 60   x4/x6/x8/x10/x12: 8 16 24 80
# = 31 configs. Only the SINK buffer varies within a run; feeder is fixed.
# Results -> Jpeg/baltree/<variant>/<buf>/fmax<F>/
# Usage:  ./run_jpeg_baltree_sweep.sh full            # all 31, full feeder
#         ./run_jpeg_baltree_sweep.sh low             # all 31, low feeder
#         ./run_jpeg_baltree_sweep.sh full x4 16      # single config
set -u
ORD=/home/wali2/backside/OpenROAD/build/bin/openroad
JDIR=/home/wali2/backside/OpenROAD/src/cms/test/backside_result/Jpeg
BK=$JDIR/jpeg_backside_baltree.tcl

VARIANT="${1:-full}"                     # full | low
if [ "$VARIANT" != "full" ] && [ "$VARIANT" != "low" ]; then
  echo "first arg must be 'full' or 'low'"; exit 1
fi
export FEEDER="$VARIANT"

declare -A FMAX_FOR
FMAX_FOR[x1]="8 16 20"
FMAX_FOR[x2]="6 16 24 40"
FMAX_FOR[x3]="8 16 24 60"
FMAX_FOR[x4]="8 16 24 80"
FMAX_FOR[x6]="8 16 24 80"
FMAX_FOR[x8]="8 16 24 80"
FMAX_FOR[x10]="8 16 24 80"
FMAX_FOR[x12]="8 16 24 80"

if [ $# -eq 3 ]; then
  RUN_BUFS="$2"; FMAX_FOR[$2]="$3"
else
  RUN_BUFS="x1 x2 x3 x4 x6 x8 x10 x12"
fi

for b in $RUN_BUFS; do
  master="gt2_6t_buf_${b}_w31_lvt"
  for f in ${FMAX_FOR[$b]}; do
    out=$JDIR/baltree/$VARIANT/$b/fmax$f; mkdir -p "$out"
    echo "======== [$VARIANT] SINK=$b (F_max=$f) -> baltree/$VARIANT/$b/fmax$f ========"
    SBUF=$master FMAX=$f OUTDIR="$out" $ORD -threads 128 -exit "$BK" 2>&1 | tee "$out/openroad.log"
    if grep -qE "DRT-0206|checkConnectivity break|^\[ERROR|ERROR:" "$out/openroad.log"; then
      echo ">>> [$VARIANT] $b/fmax$f OpenROAD FAILED"; continue
    fi
    ( cd "$out" && hspice jpeg_backside.sp -mt 48 -o jpeg_backside 2>&1 | tee hspice.log )
    echo "  CMS-0703: $(grep 'CMS-0703' "$out/openroad.log" | tail -1)"
  done
done

echo ""
echo "######## PARSING BALTREE SWEEP ($VARIANT) ########"
python3 "$JDIR/parse_jpeg_baltree.py" "$VARIANT"
echo "BALTREE SWEEP DONE ($VARIANT)"
