#!/bin/bash
# Literature-profile MC suite: full-network + mesh-only BS+LCB, FS legs, tree.
# One folder per experiment; each gets its chunks, mt0s, and summary.txt.
# - Skips an experiment if <dir>/summary.txt already exists.
# - Adopts chunks already present in this directory (from earlier manual runs)
#   instead of re-simulating them.
# Usage:  ./run_suite.sh          (all experiments, sequential, -j 16)
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
    echo "== $dir: adopting existing chunks from $(pwd)"
    mv ${stem}_c*.sp ${stem}_c*.mt0 "$dir"/ 2>/dev/null || true
    mv ${stem}_c*.lis ${stem}_c*.st0 ${stem}_c*.ic0 "$dir"/ 2>/dev/null || true
  else
    echo "== $dir: running $deck  n=$n  j=$J"
    cp "$deck" "$dir/"
    python3 run_mc.py "$dir/$deck" -n "$n" -j "$J"
  fi
  python3 mc_skew.py "$dir"/${stem}_c*.mt0 | tee "$dir/summary.txt"
}

# 1-3: full-network, literature variations, tree SIMULATED (no injection)
# 4-6: mesh-only, literature variations + tree injection (+/-25ps arr, +/-10ps slew)
#        folder             deck                   samples
run_one  full_bslcb         bslcb_lit.sp           5000
run_one  full_fslcb         fslcb_lit.sp           5000
run_one  full_fsdirect      fsdirect_lit.sp        5000
run_one  mesh_bslcb         bslcb_mesh_inj.sp      1000
run_one  mesh_fslcb         fslcb_mesh_inj.sp      1000
run_one  mesh_fsdirect      fsdirect_mesh_inj.sp   1000

echo ""
echo "==== SUITE COMPLETE ===="
for d in full_bslcb full_fslcb full_fsdirect mesh_bslcb mesh_fslcb mesh_fsdirect; do
  echo "--- $d"; head -5 "$d/summary.txt" 2>/dev/null
done
