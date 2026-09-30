#!/bin/bash
# IBEX FRONTSIDE (M5/M6) density sweep -- direct-to-mesh (no sink tier), full
# feeder, vary PITCH. One config per pitch. Full flow + HSpice.
# Results -> Ibex/density/fs_pitch<tag>/
# Usage:  ./run_ibex_fs_density_pitch.sh 2 2.5 3 3.5 4 5 6 7 8 10 12 14 16
set -u
ORD=/home/wali2/backside/OpenROAD/build/bin/openroad
JDIR=/home/wali2/backside/OpenROAD/src/cms/test/backside_result/Ibex
FS=/home/wali2/backside/OpenROAD/src/cms/test/backside_result/multi_frontside_baltree.tcl
export DESIGN=ibex FEEDER=full

if [ $# -lt 1 ]; then echo "usage: $0 <pitch> [pitch ...]"; exit 1; fi

printf "%-7s %-11s %-8s %-8s %-9s %-8s %-s\n" pitch grid drivers sinks toggled skew status
for p in "$@"; do
  tag=$(echo "$p" | tr '.' 'p')
  out=$JDIR/density/fs_pitch$tag; mkdir -p "$out"
  PITCH=$p OUTDIR="$out" $ORD -threads 128 -exit "$FS" > "$out/openroad.log" 2>&1
  L="$out/openroad.log"
  if grep -qE "DRT-0206|checkConnectivity break|^\[ERROR|ERROR:" "$L"; then
    printf "%-7s %-11s %-8s %-8s %-9s %-8s %-s\n" "$p" "?" "?" "?" "?" "?" "FAIL: OpenROAD/route"; continue
  fi
  ( cd "$out" && hspice ibex_frontside.sp -mt 48 -o ibex_frontside > hspice.log 2>&1 )
  grid=$(grep -oE '[0-9]+ H-lines x [0-9]+ V-lines' "$L" | tail -1 | sed 's/ H-lines x / x /;s/ V-lines//')
  drv=$(grep -oE 'Created [0-9]+ proxy' "$L" | grep -oE '[0-9]+' | head -1)
  read sinks toggled skew < <(python3 - "$out" <<'PY'
import sys,re,glob
d=sys.argv[1]
ts={}
for l in open(glob.glob(d+"/*.lis")[0],errors='ignore'):
    m=re.match(r'\s*(t_sink_\d+)=\s*([-0-9.eE+]+)([a-z]?)',l)
    if m:
        u={'p':1e-12,'n':1e-9,'':1}.get(m.group(3),1); ts[m.group(1)]=float(m.group(2))*u
tot=len(ts); tog=[v*1e12 for v in ts.values() if v and 0<v<1e-6]
print(tot, len(tog), f"{(max(tog)-min(tog)):.1f}" if tog else "NA")
PY
)
  st="OK"; [ "$toggled" != "$sinks" ] && st="SPARSE: $((sinks-toggled)) not toggling"
  printf "%-7s %-11s %-8s %-8s %-9s %-8s %-s\n" "$p" "${grid:-?}" "${drv:-?}" "$sinks" "$toggled" "$skew" "$st"
done
echo "frontside density sweep done -> Ibex/density/fs_pitch<tag>/"
