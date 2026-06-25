# Pass 2 (backside crossing): fresh process reading the frontside-routed odb.
# A fresh process is REQUIRED: the DRT fixes BOTTOM_ROUTING_LAYER on its first
# detailed_route, so the backside clock floor (BM2) must be established here
# before any routing, not carried over from pass 1 (which used clock M3-M5).
#
# We freeze everything already routed (set special) and route ONLY the crossing
# stubs (mesh-buffer outputs + sinks). Clock range spans BM2..M5 so each stub
# routes continuously from its frontside pin (M0) down M0 -nTSV- BPR -BV0- BM1
# -BV1- BM2 to the mesh BTerm. Signals + the frontside tree stay on top (maze
# restrictToFrontside); only the clock crossing nets are unrestricted.
set orfs    /home/wali2/backside/OpenROAD-flow-scripts/flow
set plat    $orfs/platforms/gt2n
set rdir    [file join [file dirname [file normalize [info script]]] results]

read_db      $rdir/gcd_4c_frontside.odb
read_liberty $plat/lib/gt2_6t_w31_lvt_tt_0p7v25c.lib
read_sdc     $orfs/results/gt2n/gcd/base/3_place.sdc
source $plat/setRC.tcl

proc is_stub {nm} { return [expr {[string match "sink_*" $nm] || [string match "*_buf_*" $nm]}] }
set b [ord::get_db_block]
foreach n [$b getNets] {
  set nm [$n getName]
  if {[is_stub $nm]} { $n clearSpecial } else { $n setSpecial }
}

set_routing_layers -signal M2-M5 -clock BM2-M5
global_route   -guide_file $rdir/4c_p2.guide -congestion_iterations 50
detailed_route -output_drc $rdir/4c_p2_drc.rpt -droute_end_iter 0
write_db $rdir/gcd_4c_final.odb
puts ">>> PASS2 done: gcd_4c_final.odb"
exit 0
