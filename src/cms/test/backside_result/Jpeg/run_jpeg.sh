#!/bin/bash
# Fresh end-to-end JPEG run: frontside + 4 backside F_max configs.
# Each config -> OpenROAD mesh flow -> HSpice -> results land in its own folder:
#   Jpeg/Frontside/  Jpeg/fmax8/  Jpeg/fmax16/  Jpeg/fmax24/  Jpeg/fmax80/
# Usage:  ./run_jpeg.sh              # run all 5
#         ./run_jpeg.sh fmax8 fmax16 # run a subset
set -u
ORD=/home/wali2/backside/OpenROAD/build/bin/openroad
JDIR=/home/wali2/backside/OpenROAD/src/cms/test/backside_result/Jpeg
BK=$JDIR/jpeg_backside_param.tcl
FS=$JDIR/jpeg_frontside_param.tcl

run_backside() {  # $1 = fmax value, $2 = folder tag
  local fmax=$1 tag=$2 out=$JDIR/$2
  echo "======== BACKSIDE F_max=$fmax  -> $tag ========"
  mkdir -p "$out"
  FMAX=$fmax OUTDIR="$out" $ORD -threads 72 -exit "$BK" 2>&1 | tee "$out/openroad.log"
  echo ">>> [$tag] key milestones:"; grep -E "CMS-0703|DRT-0199|DPL-0036" "$out/openroad.log" | tail -3
  echo ">>> [$tag] running HSpice (-mt 48) ..."
  ( cd "$out" && hspice jpeg_backside.sp -mt 48 -o jpeg_backside 2>&1 | tee hspice.log )
  echo "  hspice done  (mt0: $out/jpeg_backside.mt0)"
}

run_frontside() {
  local out=$JDIR/Frontside
  echo "======== FRONTSIDE -> Frontside ========"
  mkdir -p "$out"
  OUTDIR="$out" $ORD -threads 72 -exit "$FS" 2>&1 | tee "$out/openroad.log"
  echo ">>> [Frontside] key milestones:"; grep -E "CMS-0|DRT-0199" "$out/openroad.log" | tail -3
  echo ">>> [Frontside] running HSpice (-mt 48) ..."
  ( cd "$out" && hspice jpeg_frontside.sp -mt 48 -o jpeg_frontside 2>&1 | tee hspice.log )
  echo "  hspice done  (mt0: $out/jpeg_frontside.mt0)"
}

# which configs to run (default: all)
CONFIGS=("$@")
[ ${#CONFIGS[@]} -eq 0 ] && CONFIGS=(Frontside fmax8 fmax16 fmax24 fmax80)

for c in "${CONFIGS[@]}"; do
  case "$c" in
    Frontside) run_frontside ;;
    fmax8)  run_backside 8  fmax8  ;;
    fmax16) run_backside 16 fmax16 ;;
    fmax24) run_backside 24 fmax24 ;;
    fmax80) run_backside 80 fmax80 ;;
    *) echo "unknown config: $c" ;;
  esac
done

echo ""
echo "######## PARSING RESULTS ########"
python3 "$JDIR/parse_jpeg.py"
echo "ALL DONE"
