# NORMAL CTS clock-tree flow on the SAME balanced PDN as the FS/BS mesh flows.
# Apples-to-apples reference: no mesh, just clock_tree_synthesis -> route -> end.
# Full LVT buffer range (single VT / one corner). FEEDER unused here.
set base /home/wali2/backside/OpenROAD/src/cms/test/backside
set orfs /home/wali2/backside/OpenROAD-flow-scripts/flow
set res  $orfs/results/gt2n/ibex/base
set plat $orfs/platforms/gt2n
set gt2n /home/wali2/backside/GT2N
set rdir [file join $base results]

set odir [expr {[info exists env(OUTDIR)] ? $env(OUTDIR) : [file join $base results]}]
file mkdir $odir

read_db      $rdir/ibex_pdn_balanced.odb
foreach lib [lsort [glob $plat/lib/gt2_6t_w*_tt_0p7v25c.lib]] { read_liberty $lib }
read_sdc     $res/3_place.sdc

detailed_placement
source $plat/setRC.tcl
set_wire_rc -signal -layer M3
set_wire_rc -clock  -layer M4
set clk ""
foreach c [sta::all_clocks] { set clk [get_name $c]; break }
puts ">>> ibex CTS TREE (balanced PDN): clock=$clk  outdir=$odir"

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
global_route -guide_file $odir/ibex_cts.guide -congestion_iterations 50 -verbose -congestion_report_file $odir/congestion.rpt
detailed_route -output_drc $odir/ibex_cts_drc.rpt -droute_end_iter 0

report_clock_skew > $odir/clock_skew.rpt

# ---- SPICE deck of the FULL TREE (acyclic -> STA-valid, but SPICE for
# apples-to-apples with the mesh decks; same method as the jpeg CTS deck) ----
estimate_parasitics -global_routing
set block [ord::get_db_block]
set tech  [ord::get_db_tech]
set dbu [$block getDbUnitsPerMicron]
set fh [open $plat/setRC.tcl r]; set rc [read $fh]; close $fh
set nl 0; set nv 0
foreach line [split $rc "\n"] {
  if {[regexp {set_layer_rc\s+-layer\s+(\S+)\s+-resistance\s+(\S+)\s+-capacitance\s+(\S+)} $line -> ln r c]} {
    set L [$tech findLayer $ln]
    if {$L eq "NULL"} continue
    set w [expr {[$L getWidth]/double($dbu)}]
    $L setResistance [expr {$r * $w}]
    $L setCapacitance [expr {$c / $w}]
    incr nl
  } elseif {[regexp {set_layer_rc\s+-via\s+(\S+)\s+-resistance\s+(\S+)} $line -> vn r]} {
    foreach tv [$tech getVias] {
      if {[string match "${vn}_*" [$tv getName]] || [$tv getName] eq $vn} {
        $tv setResistance $r; incr nv
      }
    }
  }
}
puts ">>> tech RC set: $nl layers, $nv vias"
set gt2n /home/wali2/backside/GT2N
write_mesh_spice -clock $clk -output $odir/ibex_cts.sp -vdd 0.7 \
    -spice_models [list $base/gt2_w31_lvt_tt_renamed.sp $gt2n/cdl/gt2_6t_w31_lvt.cdl] \
    -full_tree
write_db $odir/ibex_cts_routed.odb
puts ">>> CTS-TREE DONE: deck=$odir/ibex_cts.sp odb=$odir/ibex_cts_routed.odb"
exit 0
