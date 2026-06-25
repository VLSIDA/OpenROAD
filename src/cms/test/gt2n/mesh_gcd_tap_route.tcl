# Route the FRONTSIDE up to each tap's A pin: clock tree + clk_buf_*/sink_*
# stubs (buffer.Y / sink.CLK -> tap.A). The backside nets (b_clk_buf_*, b_sink_*,
# clk_mesh) are masked, so the router stays on the frontside. Saves the odb.
set orfs    /home/wali2/backside/OpenROAD-flow-scripts/flow
set results $orfs/results/gt2n/gcd/base
set plat    $orfs/platforms/gt2n
set rdir    [file join [file dirname [file normalize [info script]]] results]
file mkdir $rdir

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

# Mask the backside nets (b_*) so the router only does the frontside stubs.
set b [ord::get_db_block]
set masked 0
foreach n [$b getNets] {
  if {[string match "b_*" [$n getName]]} { $n setSpecial; incr masked }
}
puts ">>> masked $masked backside (b_*) nets"

set_routing_layers -signal M2-M5 -clock M2-M5
global_route   -guide_file $rdir/tap_route.guide -congestion_iterations 50
detailed_route -output_drc $rdir/tap_route_drc.rpt -droute_end_iter 0
write_db $rdir/gcd_tap_routed.odb
puts ">>> SAVED: gcd_tap_routed.odb"
exit 0
