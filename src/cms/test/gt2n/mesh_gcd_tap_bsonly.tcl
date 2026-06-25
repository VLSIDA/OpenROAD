# Isolate the backside route: build mesh+taps+nets, mask EVERYTHING except the
# b_* nets (left UNROUTED so there are no frozen guides), then route the b_* nets
# (tap.Y -> grid BTerm) on the backside mesh layers. Demonstrates the router can
# do the PinY->BTerm connection.
set orfs    /home/wali2/backside/OpenROAD-flow-scripts/flow
set results $orfs/results/gt2n/gcd/base
set plat    $orfs/platforms/gt2n
set rdir    [file join [file dirname [file normalize [info script]]] results]

read_db      $results/3_place.odb
read_liberty $plat/lib/gt2_6t_w31_lvt_tt_0p7v25c.lib
read_liberty $plat/lib/gt2_ntsv_tap.lib
read_sdc     $results/3_place.sdc
detailed_placement
source $plat/setRC.tcl
set clk ""; foreach c [sta::all_clocks] { set clk [get_name $c]; break }

create_clock_mesh -clock $clk -h_layer BM2 -v_layer BM1 -pitch 1.0 \
    -buffers {gt2_6t_buf_x4_w31_lvt} -cts_buffers {gt2_6t_buf_x4_w31_lvt}
detailed_placement -max_displacement 1000
setup_proxy_bterms    -clock $clk -proxy_layer M2
connect_sinks_to_mesh -clock $clk -proxy_layer M2
detailed_placement -max_displacement 1000

# Route ONLY the b_* nets; mask everything else (unrouted -> no frozen guides).
set b [ord::get_db_block]
set active 0
foreach n [$b getNets] {
  if {[string match "b_*" [$n getName]]} { incr active } else { $n setSpecial }
}
puts ">>> backside b_* nets to route: $active"

set_routing_layers -signal M2-M5 -clock BM2-BM1
global_route   -guide_file $rdir/bsonly.guide -congestion_iterations 50
detailed_route -output_drc $rdir/bsonly_drc.rpt -droute_end_iter 0
write_db $rdir/gcd_tap_bsonly.odb
puts ">>> SAVED: gcd_tap_bsonly.odb"
exit 0
