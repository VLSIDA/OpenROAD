#!/bin/bash
# Cross-benchmark density-crossover sweep. Per design, full feeder, sink x4/fmax8:
#   BS mesh sparse (pitch 3.2) + dense (1.6),  FS mesh sparse + dense,  CTS tree.
# Results -> crossbench/<design>/<tag>/
# Usage:  ./run_crossbench.sh                 # all ready designs
#         ./run_crossbench.sh ibex minimax    # specific designs
set -u
ORD=/home/wali2/backside/OpenROAD/build/bin/openroad
RD=/home/wali2/backside/OpenROAD/src/cms/test/backside_result
export FEEDER=full
SBUF=gt2_6t_buf_x4_w31_lvt; FMAX=8

DESIGNS="${*:-jpeg ibex minimax sha3 gcd}"

run() { # tag  tcl  extra-env  deckbase
  local tag=$1
  local tcl=$2
  local xenv=$3
  local deck=$4
  local out=$RD/crossbench/$D/$tag
  mkdir -p "$out"
  echo "==== [$D] $tag ===="
  env DESIGN=$D SBUF=$SBUF FMAX=$FMAX OUTDIR="$out" $xenv $ORD -threads 128 -exit "$RD/$tcl" \
      > "$out/openroad.log" 2>&1
  if grep -qE "GRT-0116|checkConnectivity break|DRT-0206|^\[ERROR|ERROR:" "$out/openroad.log"; then
    echo "   FAILED (route/error) — see $out/openroad.log"; return
  fi
  if [ -n "$deck" ] && [ -f "$out/${D}_${deck}.sp" ]; then
    ( cd "$out" && hspice ${D}_${deck}.sp -mt 48 -o ${D}_${deck} > hspice.log 2>&1 )
  fi
}

for D in $DESIGNS; do
  [ -f "$RD/../backside/results/${D}_pdn_balanced.odb" ] || { echo ">> $D: no pdn_balanced.odb, skipping"; continue; }
  run BS_sparse multi_backside_baltree.tcl  "PITCH=3.2" backside
  run BS_dense  multi_backside_baltree.tcl  "PITCH=1.6" backside
  run FS_sparse multi_frontside_baltree.tcl "PITCH=3.2" frontside
  run FS_dense  multi_frontside_baltree.tcl "PITCH=1.6" frontside
  run CTS       multi_cts_balanced.tcl      ""          ""     # STA skew, no deck
done
echo "CROSSBENCH DONE -> crossbench/<design>/"
