#!/usr/bin/env bash
# Generate REAL OpenRCX rules for GT2N from the cloned StarRC ITF, using
# FasterCap as the field solver. Full chain, all steps validated 2026-07-05:
#   1. itf2process.py  : GT2.itf  -> RCX `process` stack file
#   2. gen_solver_patterns : process -> Over/Under/OverUnder/DiagUnder patterns
#                            + resistance.<corner> (R verified vs ITF RPSQ)
#   3. run_fasterCap.bash  : each pattern geometry -> FasterCap capacitance
#   4. fasterCapParse.py   : FasterCap logs -> RCX .caps tables (per category)
#   5. init/read/write_rcx_model : .caps + resistance -> gt2n_<side>.rcx.model
#
# The FasterCap solve (step 3) is the long pole: ~1-2 min per pattern, and a
# real (coupling-bearing) model needs WIRE_CNT=3, so budget hours. Run it
# yourself; this script is idempotent (run_fasterCap skips solved patterns).
#
# Usage:  gen_gt2n_rules.sh <frontside|backside> [WIRE_CNT] [WORKDIR]
set -euo pipefail

SIDE=${1:-frontside}
WIRE_CNT=${2:-3}            # 1 = ground/fringe only (fast smoke); 3 = real coupling
HERE=$(cd "$(dirname "$0")" && pwd)
FCM=/home/wali2/backside/OpenROAD/src/rcx/test/rcx_v2/FasterCapModel
OR=$FCM/bin/openroad
FC=$FCM/bin/FasterCap
PY=$FCM/scripts/UniversalFormat2FasterCap_923.py
WORK=${3:-$HERE/rcx_$SIDE}

# metal count per side (frontside M0..M7 = 8 ; backside BRDL..BPR = 6)
if [ "$SIDE" = "frontside" ]; then MET=8; else MET=6; fi

echo ">>> GT2N RCX rule-gen : side=$SIDE wire_cnt=$WIRE_CNT met=$MET workdir=$WORK"
rm -rf "$WORK"; mkdir -p "$WORK/gen"
python3 "$HERE/itf2process.py" "$SIDE" > "$WORK/gen/gt2n_$SIDE.process"

# 1+2) patterns (run from gen/ so process.out + TYP/ land there)
( cd "$WORK/gen"
  echo "gen_solver_patterns -process_file gt2n_$SIDE.process -process_name TYP \
        -version 2 -wire_cnt $WIRE_CNT -len 10 -over_dist 4 -under_dist 4" \
    | "$OR" -exit )
NPAT=$(find "$WORK/gen/TYP" -name wires | wc -l)
echo ">>> generated $NPAT patterns"

# 3) FasterCap solve over ALL patterns (idempotent; the slow step)
( cd "$WORK"
  "$FCM/scripts/run_fasterCap.bash" gen fc standard 20 ALL "$PY" "$FC" )
FCDIR="$WORK/fc.standard.20.20.20.ALL"

# 4) parse each pattern category into its own .caps table.
#    victim wire index = middle wire = (WIRE_CNT+1)/2 (1 for cnt=1, 2 for cnt=3)
WIRE=$(( (WIRE_CNT + 1) / 2 ))
mkdir -p "$WORK/caps"
parse_cat () { # <category-substring> <caps-name>
  local cat=$1 name=$2 d="$WORK/caps/$name"
  rm -rf "$d"; mkdir -p "$d"; ( cd "$d"
    find "$FCDIR" -path "*/$cat/*" -name wires.log | sort > sorted.input.list
    [ -s sorted.input.list ] || { echo "   (no $cat patterns)"; return; }
    python3 "$FCM/scripts/fasterCapParse.py" \
        -in_list_file sorted.input.list -wire "$WIRE" -out_file "$name.caps" > OUT 2>&1
    echo "   parsed $cat -> $(wc -l < "$name.caps") rows" )
}
parse_cat Over1      over
parse_cat Under1     under
parse_cat OverUnder1 overunder
parse_cat UnderDiag1 diag

# 5) assemble the model
cat > "$WORK/model.tcl" <<EOF
init_rcx_model -corner_names "TYP" -met_cnt $MET
read_rcx_tables -corner_name TYP -file $WORK/caps/over/over.caps            -wire_index $WIRE -over
read_rcx_tables -corner_name TYP -file $WORK/caps/under/under.caps          -wire_index $WIRE -under
read_rcx_tables -corner_name TYP -file $WORK/caps/overunder/overunder.caps  -wire_index $WIRE -over_under
read_rcx_tables -corner_name TYP -file $WORK/caps/diag/diag.caps            -wire_index $WIRE -diag
read_rcx_tables -corner_name TYP -file $WORK/gen/resistance.TYP             -wire_index $WIRE -over
write_rcx_model -file $HERE/gt2n_$SIDE.rcx.model
EOF
"$OR" -exit < "$WORK/model.tcl"
echo ">>> DONE: $HERE/gt2n_$SIDE.rcx.model"
