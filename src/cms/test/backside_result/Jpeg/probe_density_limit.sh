#!/bin/bash
# DENSITY-LIMIT PROBE: find the densest mesh pitch that still ROUTES for the
# worst-case sweep config (fmax8 = most sink taps, full feeder = biggest buffers).
# Route only (no HSpice) — we just need pass/fail per pitch.
# Reports: pitch -> grid, drivers, HPWL-delta, route status.
# Usage:  ./probe_density_limit.sh 2.0 1.6      (default: 2.0 1.6)
set -u
ORD=/home/wali2/backside/OpenROAD/build/bin/openroad
JDIR=/home/wali2/backside/OpenROAD/src/cms/test/backside_result/Jpeg
BK=$JDIR/jpeg_backside_baltree.tcl
export FEEDER=full
SBUF=gt2_6t_buf_x4_w31_lvt; FMAX=8

if [ $# -ge 1 ]; then PITCHES="$*"; else PITCHES="2.0 1.6"; fi

echo "=== density-limit probe (fmax8 + full feeder = worst case for routing) ==="
printf "%-7s %-10s %-8s %-11s %-12s\n" pitch grid drivers HPWL-delta status
for p in $PITCHES; do
  tag=$(echo "$p" | tr '.' 'p')
  out=$JDIR/density/probe_$tag; mkdir -p "$out"
  PITCH=$p SBUF=$SBUF FMAX=$FMAX OUTDIR="$out" $ORD -threads 128 -exit "$BK" > "$out/openroad.log" 2>&1
  L="$out/openroad.log"
  grid=$(grep -oE '[0-9]+ H-lines x [0-9]+ V-lines' "$L" | tail -1 | sed 's/ H-lines x / x /;s/ V-lines//')
  drv=$(grep -oE 'Created [0-9]+ proxy' "$L" | grep -oE '[0-9]+')
  hpwl=$(grep -oE 'delta HPWL +[0-9]+ %' "$L" | tail -1 | grep -oE '[0-9]+ %')
  if grep -qE "GRT-0116" "$L"; then st="FAIL: GRT congestion"
  elif grep -qE "DRT-0206|checkConnectivity break" "$L"; then st="FAIL: DRT connectivity"
  elif grep -qE "CMS-0862" "$L"; then st="OK (routed, deck written)"
  else st="FAIL: stopped ($(grep -oE 'Error:.*' "$L" | tail -1))"; fi
  printf "%-7s %-10s %-8s %-11s %-12s\n" "$p" "${grid:-?}" "${drv:-?}" "${hpwl:-?}" "$st"
done
echo "known: 3.2 OK, 2.4 OK, 1.2 FAIL(GRT). densest OK pitch here = use for the full sweep."
