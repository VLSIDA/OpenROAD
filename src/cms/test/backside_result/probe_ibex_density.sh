#!/bin/bash
# IBEX densest-mesh probe: run the DESIGN-parameterized backside flow at
# increasingly DENSE pitches, route only, report grid/drivers/route status.
# Finds the densest pitch that still routes (dense limit) for ibex.
# Usage:  ./probe_ibex_density.sh                 # default dense pitch list
#         ./probe_ibex_density.sh 1.2 1.0         # specific pitches
set -u
ORD=/home/wali2/backside/OpenROAD/build/bin/openroad
RD=/home/wali2/backside/OpenROAD/src/cms/test/backside_result
BK=$RD/multi_backside_baltree.tcl
export DESIGN=ibex FEEDER=full
SBUF=gt2_6t_buf_x4_w31_lvt; FMAX=8

if [ $# -ge 1 ]; then PITCHES="$*"; else PITCHES="3.2 1.6 1.2 1.0 0.8"; fi

echo "=== IBEX densest-mesh probe (x4/fanout8, full feeder, ~44um core) ==="
printf "%-7s %-10s %-8s %-11s %-12s\n" pitch grid drivers HPWL-delta status
for p in $PITCHES; do
  tag=$(echo "$p" | tr '.' 'p')
  out=$RD/Ibex/density/probe_$tag; mkdir -p "$out"
  PITCH=$p SBUF=$SBUF FMAX=$FMAX OUTDIR="$out" $ORD -threads 128 -exit "$BK" > "$out/openroad.log" 2>&1
  L="$out/openroad.log"
  grid=$(grep -oE '[0-9]+ H-lines x [0-9]+ V-lines' "$L" | tail -1 | sed 's/ H-lines x / x /;s/ V-lines//')
  drv=$(grep -oE 'Created [0-9]+ proxy' "$L" | grep -oE '[0-9]+')
  hpwl=$(grep -oE 'delta HPWL +[0-9]+ %' "$L" | tail -1 | grep -oE '[0-9]+ %')
  if grep -qE "GRT-0116" "$L"; then st="FAIL: GRT congestion"
  elif grep -qE "DRT-0206|checkConnectivity break" "$L"; then st="FAIL: DRT connectivity"
  elif grep -qE "CMS-0862" "$L"; then st="OK (routed, deck written)"
  else st="FAIL/incomplete: $(grep -oE 'Error:.*' "$L" | tail -1 | cut -c1-40)"; fi
  printf "%-7s %-10s %-8s %-11s %-12s\n" "$p" "${grid:-?}" "${drv:-?}" "${hpwl:-?}" "$st"
done
echo "densest OK pitch = ibex dense limit. (jpeg was pitch 1.6 densest / 1.2 failed)"
