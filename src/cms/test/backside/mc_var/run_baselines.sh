#!/bin/bash
# Full 10k-sample MC baselines for the three Ibex clock networks.
# Sequential (throughput is hspice-license-bound, not CPU-bound).
# Safe to re-run after a crash/interrupt: each run resumes from samples.csv.
set -e
cd "$(dirname "$0")"

SEED=20260901   # same seed for all three -> identical global-Vdd sequence (CRN)
N=10000
JOBS=32

python3 check_knobs.py

for net in bslcb fslcb fsdirect; do
    echo "=== $net : $N samples ==="
    python3 run_mcvar.py --network $net -n $N --seed $SEED -j $JOBS -o ${net}_10k
done

python3 mc_stats.py summary bslcb_10k fslcb_10k fsdirect_10k | tee baseline_summary.txt
python3 mc_stats.py hist    bslcb_10k fslcb_10k fsdirect_10k
echo "done: baseline_summary.txt + per-network skew_hist.png"
