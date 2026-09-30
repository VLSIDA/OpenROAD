#!/bin/bash
# IID-VDD Monte Carlo suite: BS+LCB, FS+LCB, FS Direct (tree already done in
# tree_10k_iidvdd).  10k samples each, seed 20260901, table-of-record sigmas
# (wire R 12.5% FS / 6.25% BS, Vdd 18 mV per-cell IID, Karner device sigmas).
#
# Safe to re-run: each campaign is checkpointed (samples.csv) and failed
# samples are retried on the next invocation (see failures.log per outdir).
# Usage:  ./run_iidvdd_suite.sh          (-j 16)
#         J=8 ./run_iidvdd_suite.sh
set -e
cd "$(dirname "$0")"
J=${J:-16}

for NET in bslcb fslcb fsdirect; do
  echo "==== $NET (iid vdd, 10k) ===="
  python3 run_mcvar.py --network $NET -n 10000 --seed 20260901 -j "$J" \
      --vdd-scope iid -o ${NET}_10k_iidvdd
done

echo ""
echo "==== IID-VDD SUITE COMPLETE ===="
python3 mc_stats.py summary bslcb_10k_iidvdd fslcb_10k_iidvdd fsdirect_10k_iidvdd tree_10k_iidvdd
