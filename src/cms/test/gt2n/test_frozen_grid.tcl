# Verify computeFrozenGrid on the placed gcd: reads the core box + BM1/BM2 track
# grids, builds the deterministic deformed/pruned mesh, and logs CMS-144 (V/H/
# intersections/fragments) + CMS-146 (fragment sizes). No design modification.
set orfs /home/wali2/backside/OpenROAD-flow-scripts/flow
set res  $orfs/results/gt2n/gcd/base

read_db $res/3_place.odb

# gt2n BM3 vertical power straps (from pdn.tcl): pitch 2.16, offset 1.08, width 0.36 um
compute_frozen_grid -h_layer BM2 -v_layer BM1 -pitch 1.0 \
    -strap_pitch 2.16 -strap_offset 1.08 -strap_width 0.36
exit 0
