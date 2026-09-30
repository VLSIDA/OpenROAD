# SPICE deck for backside clock-mesh SKEW: full mesh flow, then
#   capture_mesh_arrivals -> convert_mesh_swire (mesh + b_* stubs)
#   -> tech RC from setRC -> OpenRCX -lef_rc -> write_mesh_spice.
# NO merge_mesh_nets: the TSVs must stay as separate A/Y nets (the writer
# emits each TSV as a 130ohm series R and aliases BTerm ties to mesh nodes).
set orfs /home/wali2/backside/OpenROAD-flow-scripts/flow
set res  $orfs/results/gt2n/gcd/base
set plat $orfs/platforms/gt2n
set here [file dirname [file normalize [info script]]]
set rdir [file join $here results]
file mkdir $rdir

read_db      $res/3_place.odb
# the flow synthesizes with ALL W/Vt families (GT2N_USE_W = w31 w13, all Vt):
# read every width/Vt liberty or cells like gt2_6t_dffasync_x1_w13_elvt have
# no liberty view (sink pin caps would silently come out 0)
foreach lib [lsort [glob $plat/lib/gt2_6t_w*_tt_0p7v25c.lib]] {
  read_liberty $lib
}
read_sdc     $res/3_place.sdc
read_lef     $plat/lef/gt2_6t_TSV.lef
read_liberty $plat/lib/gt2_6t_TSV.lib
read_lef     $plat/lef/gt2_ntsv_via.lef

detailed_placement
source $plat/setRC.tcl
set clk ""
foreach c [sta::all_clocks] { set clk [get_name $c]; break }

create_clock_mesh -clock $clk -h_layer BM2 -v_layer BM1 -pitch 1.0 \
    -buffers {gt2_6t_buf_x4_w31_lvt} -cts_buffers {gt2_6t_buf_x4_w31_lvt}
detailed_placement -max_displacement 1000
setup_proxy_bterms -clock $clk -proxy_layer BM2
create_sink_taps -h_layer BM2 -v_layer BM1 \
    -buffer gt2_6t_buf_x4_w31_lvt -capacity 16 -tsv_master gt2_6t_TSV
break_bpr_at_tsvs -halo 0.224 -relocate_rows 1
detailed_placement -max_displacement 1000

# ---- route the frontside (signal hops buffer<->TSV<->FFs) ----
set block  [ord::get_db_block]
set tech   [ord::get_db_tech]
set bpr    [$tech findLayer BPR]
set core_r [$block getCoreArea]
set bpr_obs [odb::dbObstruction_create $block $bpr \
    [$core_r xMin] [$core_r yMin] [$core_r xMax] [$core_r yMax]]
set_routing_layers -signal M2-M5 -clock M2-M5
global_route   -guide_file $rdir/sk.guide -congestion_iterations 50
detailed_route -output_drc $rdir/sk_drc.rpt -droute_end_iter 0
odb::dbObstruction_destroy $bpr_obs

# ---- SPICE prep ----
# STA arrivals at mesh-buffer inputs (baked into per-buffer PULSE sources)
estimate_parasitics -global_routing
capture_mesh_arrivals -clock $clk
# mesh special wires + b_* stubs -> regular dbWire so OpenRCX extracts them
convert_mesh_swire -clock $clk

# per-layer/via RC onto the db tech (from setRC: R ohm/um, C pF/um)
#   LEF-style units: RPERSQ = R*width ; CPERSQDIST = C/width
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

# NO OpenRCX: no gt2n rules file exists, and a foreign model (asap7) segfaults
# on the 21-routing-layer stack. write_mesh_spice emits the RC ANALYTICALLY
# from each net's dbWire + the tech-layer/via RC set above.

# the deck: REAL PDK cells + models (cloned GT2N):
#   - cdl/gt2_6t_w31_lvt.cdl : real cell netlists (pin order A Y vdd vss)
#   - gt2_w31_lvt_tt_renamed.sp : real BSIM-CMG (level 72) GAAFET models,
#     model names aligned to the CDL (nmos_lvt/pmos_lvt)
# TSV = 149 ohm series R from the real ITF chain:
#   V0 54.99 + VSD 36.86 + VBPR 32.0 + BV0 25.10  (M1->M0->SDCON->BPR->BM1)
set gt2n /home/wali2/backside/GT2N
write_mesh_spice -clock $clk -output $rdir/gcd_mesh_skew.sp -vdd 0.7 \
    -spice_models [list $here/gt2_w31_lvt_tt_renamed.sp $gt2n/cdl/gt2_6t_w31_lvt.cdl] \
    -tsv_res 149

write_db $rdir/gcd_spice_prep.odb
puts ">>> SPICE deck: $rdir/gcd_mesh_skew.sp"
exit 0
