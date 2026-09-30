#!/bin/bash
# Re-run HSpice on already-generated IBEX decks (skips the OpenROAD flow).
IDIR=/home/wali2/backside/OpenROAD/src/cms/test/backside_result/Minimax
declare -A DECK=( [Frontside]=minimax_frontside [fmax8]=minimax_backside \
                  [fmax16]=minimax_backside [fmax24]=minimax_backside [fmax80]=minimax_backside )
for tag in Frontside fmax8 fmax16 fmax24 fmax80; do
  out=$IDIR/$tag; base=${DECK[$tag]}
  [ -f "$out/$base.sp" ] || { echo "$tag: no deck, skip"; continue; }
  echo "======== HSpice $tag ($base.sp, -mt 48) ========"
  ( cd "$out" && hspice "$base.sp" -mt 48 -o "$base" 2>&1 | tee hspice.log | tail -3 )
done
echo ""
echo "######## PARSING RESULTS ########"
python3 "$IDIR/parse_minimax.py"
echo "ALL DONE"
