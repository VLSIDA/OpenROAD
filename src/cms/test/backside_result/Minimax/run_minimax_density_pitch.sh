#!/bin/bash
# MINIMAX backside density probe -- fixed x4 sink / fanout 8, full feeder,
# NEW basis (TAPROUTE=router, CLKLAYERS=M4-M8). Vary PITCH to find the density
# window: densest pitch that routes+legalizes (min) and sparsest pitch where
# all 2251 FFs still connect AND toggle (max). Full flow + HSpice per pitch.
# Results -> Minimax/density/pitch<tag>/
# Usage:  ./run_minimax_density_pitch.sh 1.0 1.2 1.6 2.5 4 6 8 10 12 14 16
set -u
ORD=/home/wali2/backside/OpenROAD/build/bin/openroad
MDIR=/home/wali2/backside/OpenROAD/src/cms/test/backside_result/Minimax
BK=/home/wali2/backside/OpenROAD/src/cms/test/backside_result/multi_backside_baltree.tcl
export DESIGN=minimax FEEDER=full TAPROUTE=router CLKLAYERS=M4-M8
SBUF=gt2_6t_buf_x4_w31_lvt; FMAX=8

if [ $# -lt 1 ]; then echo "usage: $0 <pitch> [pitch ...]"; exit 1; fi

printf "%-7s %-11s %-8s %-8s %-9s %-8s %-s\n" pitch grid drivers sinks toggled skew status
for p in "$@"; do
  tag=$(echo "$p" | tr '.' 'p')
  out=$MDIR/density/pitch$tag; mkdir -p "$out"
  PITCH=$p SBUF=$SBUF FMAX=$FMAX OUTDIR="$out" $ORD -threads 64 -exit "$BK" > "$out/openroad.log" 2>&1
  L="$out/openroad.log"
  if grep -qE "DRT-0206|checkConnectivity break|GRT-0116|\[ERROR" "$L"; then
    st=$(grep -oE "DRT-0206|checkConnectivity break|GRT-0116" "$L" | head -1)
    printf "%-7s %-11s %-8s %-8s %-9s %-8s %-s\n" "$p" "?" "?" "?" "?" "?" "FAIL: ${st:-error}"; continue
  fi
  ( cd "$out" && hspice minimax_backside.sp -mt 24 -o minimax_backside > hspice.log 2>&1 )
  grid=$(grep -oE '[0-9]+ H-lines x [0-9]+ V-lines' "$L" | tail -1 | sed 's/ H-lines x / x /;s/ V-lines//')
  drv=$(grep -oE 'Created [0-9]+ proxy' "$L" | grep -oE '[0-9]+' | head -1)
  read sinks toggled skew < <(python3 - "$out" <<'PY'
import sys,re,glob
d=sys.argv[1]; ts={}
fs=glob.glob(d+"/*.lis")
if not fs: print(0,0,"NA"); raise SystemExit
for l in open(fs[0],errors='ignore'):
    m=re.match(r'\s*(t_sink_\d+)=\s*([-0-9.eE+]+)([a-z]?)',l)
    if m:
        u={'p':1e-12,'n':1e-9,'':1}.get(m.group(3),1); ts[m.group(1)]=float(m.group(2))*u
tot=len(ts); tog=[v*1e12 for v in ts.values() if v and 0<v<1e-6]
print(tot, len(tog), f"{(max(tog)-min(tog)):.1f}" if tog else "NA")
PY
)
  st="OK"; [ "$sinks" != "2251" ] && st="DROPPED: $((2251-sinks)) FFs not in deck"
  [ "$toggled" != "$sinks" ] && st="SPARSE: $((sinks-toggled)) not toggling"
  printf "%-7s %-11s %-8s %-8s %-9s %-8s %-s\n" "$p" "${grid:-?}" "${drv:-?}" "$sinks" "$toggled" "$skew" "$st"
done
echo "min pitch = densest with OK; max pitch = sparsest with all 2251 in deck + toggling."
