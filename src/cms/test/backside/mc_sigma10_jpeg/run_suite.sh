#!/bin/bash
# Uniform sigma=10% MC suite: the three full-network JPEG checker configs.
# One folder per experiment (chunks + mt0s + summary.txt).
# - Skips an experiment if <dir>/summary.txt exists.
# - Adopts chunks already sitting in this directory instead of re-running.
# Usage:  ./run_suite.sh          (sequential, -j 16)
#         J=8 ./run_suite.sh      (fewer licenses)
set -e
cd "$(dirname "$0")"
J=${J:-16}

if pgrep -x hspice >/dev/null; then
  echo "ERROR: hspice is already running - wait for it to finish first." >&2
  exit 1
fi

run_one () {  # <dir> <deck.sp> <samples>
  local dir=$1 deck=$2 n=$3
  local stem=${deck%.sp}
  if [ -f "$dir/summary.txt" ]; then
    echo "== $dir: already done, skipping"
    return
  fi
  mkdir -p "$dir"
  if ls ${stem}_c0.mt0 >/dev/null 2>&1; then
    echo "== $dir: adopting existing chunks"
    mv ${stem}_c*.sp ${stem}_c*.mt0 "$dir"/ 2>/dev/null || true
    mv ${stem}_c*.lis ${stem}_c*.st0 ${stem}_c*.ic0 "$dir"/ 2>/dev/null || true
  else
    echo "== $dir: running $deck  n=$n  j=$J"
    cp "$deck" "$dir/"
    python3 run_mc.py "$dir/$deck" -n "$n" -j "$J"
  fi
  python3 mc_skew.py "$dir"/${stem}_c*.mt0 | tee "$dir/summary.txt"
}

#        folder          deck             samples
run_one  full_bslcb      bslcb_s10.sp     5000
run_one  full_fslcb      fslcb_s10.sp     5000
run_one  full_fsdirect   fsdirect_s10.sp  5000
run_one  tree            tree_s10.sp      5000

echo ""
echo "==== SIGMA=10% JPEG SUITE COMPLETE ===="
for d in full_bslcb full_fslcb full_fsdirect tree; do
  echo "--- $d"; head -6 "$d/summary.txt" 2>/dev/null
done
