# PARAMETERIZED frontside clock-mesh flow for IBEX (comparison baseline).
#   OUTDIR (env) -> where the deck / odb land
# Mesh on frontside M5/M6 (no TSVs, connect_sinks_to_mesh). UPSTREAM ORFS PDN
# (ibex 3_place.odb). Inputs + spice models read from $base / ORFS results.
set base /home/wali2/backside/OpenROAD/src/cms/test/backside
set orfs /home/wali2/backside/OpenROAD-flow-scripts/flow
set res  $orfs/results/gt2n/ibex/base
set plat $orfs/platforms/gt2n
set gt2n /home/wali2/backside/GT2N

set odir [expr {[info exists env(OUTDIR)] ? $env(OUTDIR) : [file join $base results]}]
file mkdir $odir

read_db      $res/3_place.odb
foreach lib [lsort [glob $plat/lib/gt2_6t_w*_tt_0p7v25c.lib]] { read_liberty $lib }
read_sdc     $res/3_place.sdc

detailed_placement
source $plat/setRC.tcl
set clk ""
foreach c [sta::all_clocks] { set clk [get_name $c]; break }
puts ">>> ibex FRONTSIDE mesh: clock=$clk  outdir=$odir"

# ---- frontside mesh on M5/M6 (M6 horizontal, M5 vertical), pitch 3 ----
create_clock_mesh -clock $clk -h_layer M6 -v_layer M5 -pitch 3.0 \
    -buffers {gt2_6t_buf_x4_w31_lvt} -cts_buffers {gt2_6t_buf_x4_w31_lvt}
detailed_placement -max_displacement {80 6}
setup_proxy_bterms -clock $clk -proxy_layer M6
connect_sinks_to_mesh -clock $clk -proxy_layer M6

# ---- route: signal M2-M9, clock M2-M6 ----
set block  [ord::get_db_block]
set tech   [ord::get_db_tech]
set_routing_layers -signal M2-M9 -clock M2-M6
global_route -guide_file $odir/ibex_fs.guide -congestion_iterations 50
detailed_route -output_drc $odir/ibex_fs_drc.rpt -droute_end_iter 0

connect_proxy_bterms_to_mesh -clock $clk

# ---- SPICE prep ----
estimate_parasitics -global_routing
capture_mesh_arrivals -clock $clk
convert_mesh_swire -clock $clk

# analytic tech RC from setRC.tcl
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

# NO -tsv_res (frontside mesh, 0 TSVs)
write_mesh_spice -clock $clk -output $odir/ibex_frontside.sp -vdd 0.7 \
    -spice_models [list $base/gt2_w31_lvt_tt_renamed.sp $gt2n/cdl/gt2_6t_w31_lvt.cdl]

write_db $odir/ibex_frontside.odb
puts ">>> FRONTSIDE DONE: deck=$odir/ibex_frontside.sp"
exit 0
