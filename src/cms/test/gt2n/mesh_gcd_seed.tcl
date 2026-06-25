# Seed-via-on-BTerm flow: the front<->back crossing is an explicit nTSV via
# stack seeded at each crossing-net BTerm (mesh<->M0). The router then only
# routes the frontside stub down to the column. Validates that the 4 offset
# sinks now connect (no DRT-0218) because the crossing is no longer derived
# from FastRoute guides.
set orfs    /home/wali2/backside/OpenROAD-flow-scripts/flow
set results $orfs/results/gt2n/gcd/base
set plat    $orfs/platforms/gt2n
set rdir    [file join [file dirname [file normalize [info script]]] results]
file mkdir $rdir

read_db      $results/3_place.odb
read_liberty $plat/lib/gt2_6t_w31_lvt_tt_0p7v25c.lib
read_sdc     $results/3_place.sdc
detailed_placement
source $plat/setRC.tcl

set clk ""
foreach c [sta::all_clocks] { set clk [get_name $c]; break }

create_clock_mesh -clock $clk -h_layer BM2 -v_layer BM1 -pitch 1.0 \
    -buffers {gt2_6t_buf_x4_w31_lvt} -cts_buffers {gt2_6t_buf_x4_w31_lvt}
detailed_placement -max_displacement 1000
setup_proxy_bterms    -clock $clk -proxy_layer BM2
connect_sinks_to_mesh -clock $clk -proxy_layer BM2

# NEW: seed the nTSV via stack at every crossing-net BTerm (mesh<->M0).
seed_via_stacks_at_bterms -clock $clk

set_routing_layers -signal M2-M5 -clock BM2-M5
global_route   -guide_file $rdir/seed.guide -congestion_iterations 50
detailed_route -output_drc $rdir/seed_drc.rpt -droute_end_iter 0
write_db $rdir/gcd_seed.odb
puts ">>> seed flow done: gcd_seed.odb"
exit 0
