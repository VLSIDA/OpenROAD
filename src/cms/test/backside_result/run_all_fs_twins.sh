#!/usr/bin/env bash
# Run the frontside twin campaign (FS+LCB and FS-direct, CHECKER=1, mesh on
# M5/M6, FULL SPICE decks) for every Pareto point of all four checkerboard
# backside studies, one design at a time. Fully resumable: re-running skips
# twins whose .lis already exists and only executes new frontier points.
set -u
BASE=/home/wali2/backside/OpenROAD/src/cms/test/backside_result
PY=/home/wali2/backside/dse_venv/bin/python
declare -A LOGDIR=( [ibex]=Ibex [jpeg]=Jpeg [minimax]=Minimax [floonoc]=Floonoc )

for d in ibex jpeg minimax floonoc; do
  echo "==================== $d twins: $(date) ===================="
  $PY $BASE/fs_twins.py $d 2>&1 | tee $BASE/${LOGDIR[$d]}/fs_checker_run.log
  echo "==================== $d done: $(date) ======================"
done
echo "ALL DONE: $(date)"
for d in ibex jpeg minimax floonoc; do
  echo "--- ${LOGDIR[$d]}/fs_checker/twins.csv"
done
