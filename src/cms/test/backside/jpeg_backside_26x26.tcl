# BACKSIDE clock mesh at 26x26 grid (to match the frontside 26x26 for an
# apples-to-apples front-vs-back comparison). Single session: build mesh + route
# + SPICE deck. Same setup as jpeg_spice_skew.tcl:
#   - BALANCED PDN base (jpeg_pdn_balanced.odb, 4 wide straps/net) -- "same as
#     last time"
#   - mesh on BM2(h)/BM1(v), pitch 3.2um -> ~26x26 (BM tracks snap it; the
#     frontside M5/M6 landed 26x26 at 3.192). Verify CMS-0121 and nudge pitch if
#     it comes out 25 or 27.
#   - sink buffers gt2_6t_buf_x4_w31_lvt, capacity 16
#   - sink-TSV pin escape (M1->M3) is built into create_sink_taps so x4 (or any
#     size) routes -- no DRT-0206
#   - TSV = 6 ohm (thesis nano-TSV); mesh-layer caps overridden with FasterCap
#     field-solved values
set orfs /home/wali2/backside/OpenROAD-flow-scripts/flow
set res  $orfs/results/gt2n/jpeg/base
set plat $orfs/platforms/gt2n
set here [file dirname [file normalize [info script]]]
set rdir [file join $here results]
set gt2n /home/wali2/backside/GT2N
file mkdir $rdir

read_db      $rdir/jpeg_pdn_balanced.odb
foreach lib [lsort [glob $plat/lib/gt2_6t_w*_tt_0p7v25c.lib]] { read_liberty $lib }
read_sdc     $res/3_place.sdc
read_lef     $plat/lef/gt2_6t_TSV.lef
read_liberty $plat/lib/gt2_6t_TSV.lib
read_lef     $plat/lef/gt2_ntsv_via.lef

detailed_placement
source $plat/setRC.tcl
set clk ""
foreach c [sta::all_clocks] { set clk [get_name $c]; break }
puts ">>> jpeg BACKSIDE 26x26: clock=$clk (balanced PDN, x4 sink, 6ohm TSV)"

# ---- backside mesh on BM1/BM2, pitch 3.2 -> ~26x26 ----
create_clock_mesh -clock $clk -h_layer BM2 -v_layer BM1 -pitch 3.2 \
    -buffers {gt2_6t_buf_x4_w31_lvt} -cts_buffers {gt2_6t_buf_x4_w31_lvt}
detailed_placement -max_displacement {80 6}
setup_proxy_bterms -clock $clk -proxy_layer BM2
create_sink_taps -h_layer BM2 -v_layer BM1 \
    -buffer gt2_6t_buf_x4_w31_lvt -capacity 8 -tsv_master gt2_6t_TSV
break_bpr_at_tsvs -halo 0.224 -relocate_rows 1
detailed_placement -max_displacement {80 6}

# ---- route (frontside signal M2-M9; BPR blocked over core) ----
set block  [ord::get_db_block]
set tech   [ord::get_db_tech]
set bpr    [$tech findLayer BPR]
set core_r [$block getCoreArea]
set bpr_obs [odb::dbObstruction_create $block $bpr \
    [$core_r xMin] [$core_r yMin] [$core_r xMax] [$core_r yMax]]
set_routing_layers -signal M2-M9 -clock M2-M9
global_route -guide_file $rdir/jpeg_bk26.guide -congestion_iterations 50
detailed_route -output_drc $rdir/jpeg_bk26_drc.rpt -droute_end_iter 0
odb::dbObstruction_destroy $bpr_obs

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

# mesh-layer caps -> FasterCap field-solved (dedicated backside mesh, no phantom
# min-spacing coupling)
foreach {ly cperum} {BM1 5.62e-5 BM2 7.43e-5} {
  set L [$tech findLayer $ly]
  if {$L eq "NULL"} continue
  set w [expr {[$L getWidth]/double($dbu)}]
  $L setCapacitance [expr {$cperum / $w}]
  puts ">>> mesh-cap override: $ly -> $cperum pF/um"
}

write_mesh_spice -clock $clk -output $rdir/jpeg_backside_26_skew.sp -vdd 0.7 \
    -spice_models [list $here/gt2_w31_lvt_tt_renamed.sp $gt2n/cdl/gt2_6t_w31_lvt.cdl] \
    -tsv_res 6

write_db $rdir/jpeg_backside_26.odb
puts ">>> DONE: deck=$rdir/jpeg_backside_26_skew.sp odb=$rdir/jpeg_backside_26.odb"
exit 0
