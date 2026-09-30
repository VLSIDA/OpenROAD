#!/bin/bash
# IBEX backside: mesh DRIVER buffer scaled with pitch (sparser mesh -> bigger
# driver). Sink tier fixed at x4/fanout8, full feeder. Full flow + HSpice.
# Results -> Ibex/density/scaled_pitch<tag>/
# Pairing (pitch:mesh-driver): 1.6:x2 2.5:x3 4:x4 6:x4 8:x6 10:x10 12:x10 16:x12
set -u
ORD=/home/wali2/backside/OpenROAD/build/bin/openroad
JDIR=/home/wali2/backside/OpenROAD/src/cms/test/backside_result/Ibex
BK=/home/wali2/backside/OpenROAD/src/cms/test/backside_result/multi_backside_baltree.tcl
export DESIGN=ibex FEEDER=full
SBUF=gt2_6t_buf_x4_w31_lvt; FMAX=8

# pitch -> mesh-driver buffer
PAIRS="1.6:x2 2.5:x3 4:x4 6:x4 8:x6 10:x10 12:x10 16:x12"

printf "%-7s %-7s %-11s %-8s %-8s %-9s %-8s %-s\n" pitch driver grid drivers sinks toggled skew status
for pair in $PAIRS; do
  p="${pair%%:*}"; drv="${pair##*:}"
  mbuf="gt2_6t_buf_${drv}_w31_lvt"
  tag=$(echo "$p" | tr '.' 'p')
  out=$JDIR/density/scaled_pitch$tag; mkdir -p "$out"
  PITCH=$p MBUF=$mbuf SBUF=$SBUF FMAX=$FMAX OUTDIR="$out" $ORD -threads 128 -exit "$BK" > "$out/openroad.log" 2>&1
  L="$out/openroad.log"
  if grep -qE "DRT-0206|checkConnectivity break|^\[ERROR|ERROR:" "$L"; then
    printf "%-7s %-7s %-11s %-8s %-8s %-9s %-8s %-s\n" "$p" "$drv" "?" "?" "?" "?" "?" "FAIL: OpenROAD/route"; continue
  fi
  ( cd "$out" && hspice ibex_backside.sp -mt 48 -o ibex_backside > hspice.log 2>&1 )
  grid=$(grep -oE '[0-9]+ H-lines x [0-9]+ V-lines' "$L" | tail -1 | sed 's/ H-lines x / x /;s/ V-lines//')
  ndrv=$(grep -oE 'Created [0-9]+ proxy' "$L" | grep -oE '[0-9]+' | head -1)
  read sinks toggled skew < <(python3 - "$out" <<'PY'
import sys,re,glob
d=sys.argv[1]; ts={}
for l in open(glob.glob(d+"/*.lis")[0],errors='ignore'):
    m=re.match(r'\s*(t_sink_\d+)=\s*([-0-9.eE+]+)([a-z]?)',l)
    if m:
        u={'p':1e-12,'n':1e-9,'':1}.get(m.group(3),1); ts[m.group(1)]=float(m.group(2))*u
tot=len(ts); tog=[v*1e12 for v in ts.values() if v and 0<v<1e-6]
print(tot, len(tog), f"{(max(tog)-min(tog)):.1f}" if tog else "NA")
PY
)
  st="OK"; [ "$toggled" != "$sinks" ] && st="SPARSE: $((sinks-toggled)) not toggling"
  printf "%-7s %-7s %-11s %-8s %-8s %-9s %-8s %-s\n" "$p" "$drv" "${grid:-?}" "${ndrv:-?}" "$sinks" "$toggled" "$skew" "$st"
done
echo "scaled-driver sweep done -> Ibex/density/scaled_pitch<tag>/"
