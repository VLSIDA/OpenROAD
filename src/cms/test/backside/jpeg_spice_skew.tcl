# SPICE skew/power deck for JPEG -- SINGLE SESSION (mesh build + deck).
# The SPICE-prep commands need in-memory ClockMesh state (block_,
# grid_intersections_ arrivals, mesh_connection_points_) that is NOT stored in
# the odb, so we CANNOT read a pre-built odb and prep it -- we must rebuild the
# mesh in the same session. Detailed route IS needed (do not skip it): the
# sink_drv_* leaf nets must be routed so the analytic RC gives them cap nodes,
# otherwise write_mesh_spice emits the FF sinks as floating caps and every
# skew .measure returns 'failed'.
#
#   balanced-PDN base -> create_clock_mesh (pitch 10) -> sink taps (cap 128)
#   -> break_bpr -> global+detailed route -> estimate_parasitics
#   -> capture_mesh_arrivals -> convert_mesh_swire -> tech RC
#   -> write_mesh_spice (-tsv_res 6)
set orfs /home/wali2/backside/OpenROAD-flow-scripts/flow
set res  $orfs/results/gt2n/jpeg/base
set plat $orfs/platforms/gt2n
set here [file dirname [file normalize [info script]]]
set rdir [file join $here results]
set gt2n /home/wali2/backside/GT2N
file mkdir $rdir

# balanced-PDN base (4 wide straps/net), same input the routed odb used
set base_odb $rdir/jpeg_pdn_balanced.odb
read_db      $base_odb
foreach lib [lsort [glob $plat/lib/gt2_6t_w*_tt_0p7v25c.lib]] { read_liberty $lib }
read_sdc     $res/3_place.sdc
read_lef     $plat/lef/gt2_6t_TSV.lef
read_liberty $plat/lib/gt2_6t_TSV.lib
read_lef     $plat/lef/gt2_ntsv_via.lef

detailed_placement
source $plat/setRC.tcl
set clk ""
foreach c [sta::all_clocks] { set clk [get_name $c]; break }
puts ">>> jpeg SPICE prep (single session): clock=$clk"

# ---- rebuild the sparse mesh (matches jpeg_backside.odb density) ----
create_clock_mesh -clock $clk -h_layer BM2 -v_layer BM1 -pitch 3.0 \
    -buffers {gt2_6t_buf_x4_w31_lvt} -cts_buffers {gt2_6t_buf_x4_w31_lvt}
detailed_placement -max_displacement {80 6}
setup_proxy_bterms -clock $clk -proxy_layer BM2
create_sink_taps -h_layer BM2 -v_layer BM1 \
    -buffer gt2_6t_buf_x4_w31_lvt -capacity 16 -tsv_master gt2_6t_TSV
break_bpr_at_tsvs -halo 0.224 -relocate_rows 1
detailed_placement -max_displacement {80 6}

# ---- global route only (for parasitic estimation; NO detailed route) ----
set block  [ord::get_db_block]
set tech   [ord::get_db_tech]
set bpr    [$tech findLayer BPR]
set core_r [$block getCoreArea]
set bpr_obs [odb::dbObstruction_create $block $bpr \
    [$core_r xMin] [$core_r yMin] [$core_r xMax] [$core_r yMax]]
set_routing_layers -signal M2-M9 -clock M2-M9
global_route -guide_file $rdir/jpeg_sk.guide -congestion_iterations 50
# Detailed route IS required here: the sink_drv_* leaf nets (sink buffer -> FF
# CLK pins) must have dbWire so the analytic RC walker gives them cap nodes.
# Without it, write_mesh_spice falls back to instance-pin node names and the
# FF sinks end up as floating caps (all .measure skew statements return failed).
detailed_route -output_drc $rdir/jpeg_sk_drc.rpt -droute_end_iter 0
odb::dbObstruction_destroy $bpr_obs

# ---- SPICE prep ----
estimate_parasitics -global_routing
capture_mesh_arrivals -clock $clk
convert_mesh_swire -clock $clk

# per-layer/via analytic RC onto the db tech (from setRC.tcl)
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
puts ">>> tech RC set: $nl layers, $nv vias (from setRC.tcl)"

# Override the MESH-layer caps with FasterCap field-solved values (isolated
# min-width wire, real 3D fringe, NO phantom min-spacing coupling). setRC assumes
# min-spaced neighbors on both sides, which the dedicated 3um-pitch backside mesh
# does NOT have -> it overestimated BM1 2.7x, BM2 1.4x. Field-solved pF/um; see
# the cap-verification spot-check.
foreach {ly cperum} {BM1 5.62e-5 BM2 7.43e-5} {
  set L [$tech findLayer $ly]
  if {$L eq "NULL"} continue
  set w [expr {[$L getWidth]/double($dbu)}]
  $L setCapacitance [expr {$cperum / $w}]
  puts ">>> mesh-cap override: $ly -> $cperum pF/um (FasterCap, was setRC)"
}

# deck: real PDK cells + BSIM-CMG models.
# TSV = 6 ohm per nano-TSV, from Bethur (GT MS thesis 2023) Table 3.2, citing
# the IMEC nano-TSV (Jourdain et al., ECTC 2022). Cap treated as ~0 (R-only).
write_mesh_spice -clock $clk -output $rdir/jpeg_mesh_skew.sp -vdd 0.7 \
    -spice_models [list $here/gt2_w31_lvt_tt_renamed.sp $gt2n/cdl/gt2_6t_w31_lvt.cdl] \
    -tsv_res 6

write_db $rdir/jpeg_spice_prep.odb
puts ">>> SPICE deck: $rdir/jpeg_mesh_skew.sp"
exit 0
