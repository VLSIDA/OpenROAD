#!/bin/bash
# Re-run HSpice on already-generated JPEG decks (skips the OpenROAD flow).
# Use after fixing the hspice arg order, or to re-measure without rebuilding.
JDIR=/home/wali2/backside/OpenROAD/src/cms/test/backside_result/Jpeg
declare -A DECK=( [Frontside]=jpeg_frontside [fmax8]=jpeg_backside \
                  [fmax16]=jpeg_backside [fmax24]=jpeg_backside [fmax80]=jpeg_backside )
for tag in Frontside fmax8 fmax16 fmax24 fmax80; do
  out=$JDIR/$tag; base=${DECK[$tag]}
  [ -f "$out/$base.sp" ] || { echo "$tag: no deck, skip"; continue; }
  echo "======== HSpice $tag ($base.sp, -mt 48) ========"
  ( cd "$out" && hspice "$base.sp" -mt 48 -o "$base" 2>&1 | tee hspice.log | tail -3 )
done
echo ""
echo "######## PARSING RESULTS ########"
python3 "$JDIR/parse_jpeg.py"
echo "ALL DONE"
