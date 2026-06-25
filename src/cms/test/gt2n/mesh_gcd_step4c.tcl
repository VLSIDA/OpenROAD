# Step 4c (Option B): two-pass routing, tree on TOP, mesh on backside BM1/BM2.
#  Pass 1 (frontside): clock tree + signals route on M2..M5 (stubs masked).
#  Pass 2 (backside):  buffer outputs + sinks cross to the BM1/BM2 mesh.
#
# Key vs the old version: we do NOT lower the global signal floor. The global
# floor stays frontside (M2). The clock reaches the backside only through the
# clock-specific range (set_routing_layers -clock), which the proven backside
# router lowers BOTTOM_ROUTING_LAYER for internally and which triggers the
# on-pin nTSV seeding (connectClockPinsToBackside). The device cut VSD is the
# nTSV cut layer (so it has a via and survives), VG drops out -> no DRT-0233.
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

# helper: is this a backside stub net (buffer-output or sink)?
proc is_stub {nm} { return [expr {[string match "sink_*" $nm] || [string match "*_buf_*" $nm]}] }

set b [ord::get_db_block]

# ---- PASS 1: frontside only (mask stubs so GRT/DRT skip them) ----
foreach n [$b getNets] { if {[is_stub [$n getName]]} { $n setSpecial } }
set_routing_layers -signal M2-M5 -clock M3-M5
global_route   -guide_file $rdir/4c_p1.guide -congestion_iterations 50
detailed_route -output_drc $rdir/4c_p1_drc.rpt -droute_end_iter 0
write_db $rdir/gcd_4c_frontside.odb
puts ">>> SAVED frontside odb: gcd_4c_frontside.odb"

# ---- PASS 2: backside only (keep frontside routes, route only the stubs) ----
foreach n [$b getNets] {
  set nm [$n getName]
  if {[is_stub $nm]} { $n clearSpecial } else { $n setSpecial }
}
# Clock range spans the FULL column BM2..M5 so each crossing net can route
# continuously from its frontside pin (M0, buffer output / sink) down through
# M0 -nTSV- BPR -BV0- BM1 -BV1- BM2 to the mesh BTerm on BM2. Signals stay on
# the frontside (M2-M5) and the maze confines them (and the frontside tree)
# there; only the crossing nets are unrestricted and descend to the mesh.
set_routing_layers -signal M2-M5 -clock BM2-M5
global_route   -guide_file $rdir/4c_p2.guide -congestion_iterations 50
detailed_route -output_drc $rdir/4c_p2_drc.rpt -droute_end_iter 0
write_db $rdir/gcd_4c_final.odb
puts ">>> Step 4c done: gcd_4c_final.odb"
exit 0
