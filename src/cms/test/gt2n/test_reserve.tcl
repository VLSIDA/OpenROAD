# Test Phase 1 (reserve) on the floorplan-stage gcd (rows present; no placed
# std cells, no taps, no PDN yet -- the real stage this runs at). Places TSV
# cells + layer-selective keepouts on the frozen grid, writes the result.
set orfs /home/wali2/backside/OpenROAD-flow-scripts/flow
set res  $orfs/results/gt2n/gcd/base
set plat $orfs/platforms/gt2n

read_db  $res/2_2_floorplan_macro.odb
read_lef $plat/lef/gt2_6t_TSV.lef

reserve_clock_mesh -h_layer BM2 -v_layer BM1 -pitch 1.0 \
    -strap_pitch 2.16 -strap_offset 1.08 -strap_width 0.36 \
    -tsv_master gt2_6t_TSV -spacing 0

write_db results/gcd_reserve.odb
exit 0
