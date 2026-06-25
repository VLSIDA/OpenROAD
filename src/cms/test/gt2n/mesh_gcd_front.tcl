# Frontside-BTerm flow (user's design):
#  1. BTerms on a FRONTSIDE layer (M2), slid into whitespace (off all cells).
#  2. Route the stubs entirely on the frontside (stock router).
#  3. Post-routing: drop the nTSV column M2->...->mesh at each BTerm.
#  4. Merge stub nets into clk_mesh so the columns tie to the grid.
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
# proxy_layer M2 = FRONTSIDE -> BTerms land on M2 (router target), slid to gaps.
setup_proxy_bterms    -clock $clk -proxy_layer M2
connect_sinks_to_mesh -clock $clk -proxy_layer M2

# All routing on the frontside.
set_routing_layers -signal M2-M5 -clock M2-M5
global_route   -guide_file $rdir/front.guide -congestion_iterations 50
detailed_route -output_drc $rdir/front_drc.rpt -droute_end_iter 0

# Post-routing: replace each frontside BTerm with the nTSV column down to mesh.
seed_via_stacks_at_bterms -clock $clk
write_db $rdir/gcd_front.odb
puts ">>> frontside-BTerm flow done: gcd_front.odb"
exit 0
