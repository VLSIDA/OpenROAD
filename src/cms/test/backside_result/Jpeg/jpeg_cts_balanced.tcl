# NORMAL CTS clock-tree flow on the SAME balanced PDN as the FS/BS mesh flows.
# Apples-to-apples reference: no mesh, just clock_tree_synthesis -> route -> end.
# Full LVT buffer range (single VT / one corner). FEEDER unused here.
set base /home/wali2/backside/OpenROAD/src/cms/test/backside
set orfs /home/wali2/backside/OpenROAD-flow-scripts/flow
set res  $orfs/results/gt2n/jpeg/base
set plat $orfs/platforms/gt2n
set gt2n /home/wali2/backside/GT2N
set rdir [file join $base results]

set odir [expr {[info exists env(OUTDIR)] ? $env(OUTDIR) : [file join $base results]}]
file mkdir $odir

read_db      $rdir/jpeg_pdn_balanced.odb
foreach lib [lsort [glob $plat/lib/gt2_6t_w*_tt_0p7v25c.lib]] { read_liberty $lib }
read_sdc     $res/3_place.sdc

detailed_placement
source $plat/setRC.tcl
set_wire_rc -signal -layer M3
set_wire_rc -clock  -layer M4
set clk ""
foreach c [sta::all_clocks] { set clk [get_name $c]; break }
puts ">>> jpeg CTS TREE (balanced PDN): clock=$clk  outdir=$odir"

# ---- clock tree synthesis: LVT buffer set (FEEDER env: full x1..x12 | low x2/x3/x4) ----
set feeder [expr {[info exists env(FEEDER)] ? $env(FEEDER) : "full"}]
if {$feeder eq "low"} {
  set cts_bufs "gt2_6t_buf_x2_w31_lvt gt2_6t_buf_x3_w31_lvt gt2_6t_buf_x4_w31_lvt"
} else {
  set cts_bufs "gt2_6t_buf_x1_w31_lvt gt2_6t_buf_x2_w31_lvt gt2_6t_buf_x3_w31_lvt gt2_6t_buf_x4_w31_lvt gt2_6t_buf_x6_w31_lvt gt2_6t_buf_x8_w31_lvt gt2_6t_buf_x10_w31_lvt gt2_6t_buf_x12_w31_lvt"
}
puts ">>> CTS feeder=$feeder  buf_list=$cts_bufs"
clock_tree_synthesis -buf_list $cts_bufs -sink_clustering_enable
detailed_placement -max_displacement {80 6}

# ---- route: signal + clock M2-M9 (stock router) ----
set_routing_layers -signal M2-M9 -clock M2-M9
global_route -guide_file $odir/jpeg_cts.guide -congestion_iterations 50 -verbose -congestion_report_file $odir/congestion.rpt
detailed_route -output_drc $odir/jpeg_cts_drc.rpt -droute_end_iter 0

report_clock_skew > $odir/clock_skew.rpt
write_db $odir/jpeg_cts_routed.odb
puts ">>> CTS-TREE DONE: routed odb=$odir/jpeg_cts_routed.odb"
exit 0
