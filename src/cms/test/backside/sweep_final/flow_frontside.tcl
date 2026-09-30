# FRONTSIDE clock-mesh SPICE deck for JPEG -- comparison baseline vs the
# backside mesh. Mesh on frontside M5/M6 (no TSVs: FFs are frontside too, so
# sinks connect to the mesh directly via the router + BTerms). PDN stays on the
# BACKSIDE (BM1/BM2 balanced) so the ONLY variable vs jpeg_spice_skew.tcl is the
# mesh side: frontside M6(h)/M5(v) here vs backside BM2/BM1 there.
#   balanced-PDN base -> create_clock_mesh (M6/M5, pitch 3) -> setup_proxy_bterms
#   -> connect_sinks_to_mesh (router+BTerms, NO TSVs, NO sink taps, NO break_bpr)
#   -> route -> connect_proxy_bterms_to_mesh -> capture_mesh_arrivals
#   -> convert_mesh_swire -> analytic tech RC -> write_mesh_spice (NO -tsv_res)
set orfs /home/wali2/backside/OpenROAD-flow-scripts/flow
set res  $orfs/results/gt2n/jpeg/base
set plat $orfs/platforms/gt2n
set here [file dirname [file normalize [info script]]]
set rdir [file join $here results]
set gt2n /home/wali2/backside/GT2N
file mkdir $rdir

# UPSTREAM ORFS PDN base (3_place carries the stock gt2n PDN: ~26/27 straps/net
# on BM1/BM2 @0.2um, backside). NOT the balanced rebuild -- the frontside (FS-CDN)
# case is paired with the conventional upstream PDN, per request.
read_db      $res/3_place.odb
foreach lib [lsort [glob $plat/lib/gt2_6t_w*_tt_0p7v25c.lib]] { read_liberty $lib }
read_sdc     $res/3_place.sdc

detailed_placement
source $plat/setRC.tcl
set clk ""
foreach c [sta::all_clocks] { set clk [get_name $c]; break }
puts ">>> jpeg FRONTSIDE mesh: clock=$clk  (mesh M6/M5, PDN backside)"

# ---- frontside mesh on M5/M6 (M6 horizontal, M5 vertical), same pitch 3 ----
create_clock_mesh -clock $clk -h_layer M6 -v_layer M5 -pitch 3.0 \
    -buffers {gt2_6t_buf_x4_w31_lvt} -cts_buffers {gt2_6t_buf_x4_w31_lvt}
detailed_placement -max_displacement {80 6}
# proxy BTerms on the h mesh layer (M6); sinks share/create BTerms at grid pts
setup_proxy_bterms -clock $clk -proxy_layer M6
# connect FF sinks to the mesh via the router (BTerms) -- NO TSVs, NO sink taps
connect_sinks_to_mesh -clock $clk -proxy_layer M6

# ---- route: signal M2-M9, clock M2-M6 (sinks reach the M5/M6 mesh) ----
set block  [ord::get_db_block]
set tech   [ord::get_db_tech]
set_routing_layers -signal M2-M9 -clock M2-M6
global_route -guide_file $rdir/jpeg_fs.guide -congestion_iterations 50
detailed_route -output_drc $rdir/jpeg_fs_drc.rpt -droute_end_iter 0

# via stacks tying proxy/sink BTerms down to the mesh grid
connect_proxy_bterms_to_mesh -clock $clk

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

# deck: real PDK cells + BSIM-CMG models. NO -tsv_res (frontside mesh, 0 TSVs).
write_mesh_spice -clock $clk -output $rdir/jpeg_frontside_skew.sp -vdd 0.7 \
    -spice_models [list $here/gt2_w31_lvt_tt_renamed.sp $gt2n/cdl/gt2_6t_w31_lvt.cdl]

write_db $rdir/jpeg_frontside_prep.odb
puts ">>> FRONTSIDE SPICE deck: $rdir/jpeg_frontside_skew.sp"
exit 0
