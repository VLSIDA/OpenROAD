# Pass 2 (backside): route each tap's Y pin to its grid BTerm on BM1/BM2.
# Fresh process (so the DRT floor is set for the backside this run). Reads the
# frontside-routed odb, freezes everything except the b_* nets, and routes the
# b_clk_buf_*/b_sink_* nets (tap.Y -> BTerm) on the backside mesh layers.
set orfs    /home/wali2/backside/OpenROAD-flow-scripts/flow
set plat    $orfs/platforms/gt2n
set rdir    [file join [file dirname [file normalize [info script]]] results]

read_db      $rdir/gcd_tap_routed.odb
read_liberty $plat/lib/gt2_6t_w31_lvt_tt_0p7v25c.lib
read_liberty $plat/lib/gt2_ntsv_tap.lib
read_sdc     $orfs/results/gt2n/gcd/base/3_place.sdc
source $plat/setRC.tcl

# Route ONLY the backside b_* nets; freeze everything else (frontside routes,
# tree, signals, and the special clk_mesh stay put).
set b [ord::get_db_block]
set active 0
foreach n [$b getNets] {
  if {[string match "b_*" [$n getName]]} { $n clearSpecial; incr active } else { $n setSpecial }
}
puts ">>> backside nets to route: $active"

set_routing_layers -signal M2-M5 -clock BM2-BM1
global_route   -guide_file $rdir/tap_p2.guide -congestion_iterations 50
detailed_route -output_drc $rdir/tap_p2_drc.rpt -droute_end_iter 0
write_db $rdir/gcd_tap_full.odb
puts ">>> SAVED: gcd_tap_full.odb"
exit 0
