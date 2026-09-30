# PARAMETERIZED backside clock-mesh flow for JPEG.
#   FMAX   (env) -> create_sink_taps -capacity  (cluster size cap)
#   OUTDIR (env) -> where the deck / odb / guides / drc land
# Same physics as jpeg_backside_26x26.tcl (balanced PDN, x4 sink, 6ohm TSV, 25x25
# mesh, mesh-cap field-solve override). Inputs + spice models read from $base.
set base /home/wali2/backside/OpenROAD/src/cms/test/backside
set orfs /home/wali2/backside/OpenROAD-flow-scripts/flow
set res  $orfs/results/gt2n/jpeg/base
set plat $orfs/platforms/gt2n
set gt2n /home/wali2/backside/GT2N
set rdir [file join $base results]

set fmax [expr {[info exists env(FMAX)] ? $env(FMAX) : 16}]
set sbuf [expr {[info exists env(SBUF)] ? $env(SBUF) : "gt2_6t_buf_x4_w31_lvt"}]
set odir [expr {[info exists env(OUTDIR)] ? $env(OUTDIR) : $rdir}]
file mkdir $odir

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
puts ">>> jpeg BACKSIDE: clock=$clk  F_max=$fmax  sink_buf=$sbuf  outdir=$odir"

# ---- backside mesh on BM1/BM2, pitch 3.2 -> ~25x25 ----
# Feeder buffer set selectable via FEEDER env (single VT = LVT, one corner):
#   full -> x1..x12 (whole range)      low -> x2/x3/x4 (capped at x4 width)
# Stock OpenROAD router (custom drt/grt mods reverted) routes any width.
set feeder [expr {[info exists env(FEEDER)] ? $env(FEEDER) : "full"}]
if {$feeder eq "low"} {
  set cts_bufs {gt2_6t_buf_x2_w31_lvt gt2_6t_buf_x3_w31_lvt gt2_6t_buf_x4_w31_lvt}
} else {
  set cts_bufs {gt2_6t_buf_x1_w31_lvt gt2_6t_buf_x2_w31_lvt gt2_6t_buf_x3_w31_lvt gt2_6t_buf_x4_w31_lvt gt2_6t_buf_x6_w31_lvt gt2_6t_buf_x8_w31_lvt gt2_6t_buf_x10_w31_lvt gt2_6t_buf_x12_w31_lvt}
}
set pitch [expr {[info exists env(PITCH)] ? $env(PITCH) : 3.2}]
puts ">>> feeder=$feeder  pitch=$pitch  cts_buffers=$cts_bufs"
set_placement_padding -masters $cts_bufs -left 1 -right 1
create_clock_mesh -clock $clk -h_layer BM2 -v_layer BM1 -pitch $pitch \
    -buffers {gt2_6t_buf_x4_w31_lvt} -cts_buffers $cts_bufs
detailed_placement -max_displacement {80 6}
setup_proxy_bterms -clock $clk -proxy_layer BM2
create_sink_taps -h_layer BM2 -v_layer BM1 \
    -buffer $sbuf -capacity $fmax -tsv_master gt2_6t_TSV
break_bpr_at_tsvs -halo 0.224 -relocate_rows 1
detailed_placement -max_displacement {80 6}
# author sink_tap buffer->TSV special wires NOW -- buffers are in their final
# (post-legalization) positions, so the wires won't dangle at pre-move spots
# reassign_sink_ffs disabled: post-placement re-assignment refuted (greedy
# overflow creates late outliers, bulk skew unchanged) -- keep original clustering
connect_sink_taps

# ---- route (frontside signal M2-M9; BPR blocked over core) ----
set block  [ord::get_db_block]
set tech   [ord::get_db_tech]
set bpr    [$tech findLayer BPR]
set core_r [$block getCoreArea]
set bpr_obs [odb::dbObstruction_create $block $bpr \
    [$core_r xMin] [$core_r yMin] [$core_r xMax] [$core_r yMax]]
set_routing_layers -signal M2-M9 -clock M2-M9
global_route -guide_file $odir/jpeg_bk.guide -congestion_iterations 50 -verbose -congestion_report_file $odir/congestion.rpt
detailed_route -output_drc $odir/jpeg_bk_drc.rpt -droute_end_iter 0
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

# mesh-layer caps -> FasterCap field-solved
foreach {ly cperum} {BM1 5.62e-5 BM2 7.43e-5} {
  set L [$tech findLayer $ly]
  if {$L eq "NULL"} continue
  set w [expr {[$L getWidth]/double($dbu)}]
  $L setCapacitance [expr {$cperum / $w}]
  puts ">>> mesh-cap override: $ly -> $cperum pF/um"
}

write_mesh_spice -clock $clk -output $odir/jpeg_backside.sp -vdd 0.7 \
    -spice_models [list $base/gt2_w31_lvt_tt_renamed.sp $gt2n/cdl/gt2_6t_w31_lvt.cdl] \
    -tsv_res 6

write_db $odir/jpeg_backside.odb
puts ">>> DONE: deck=$odir/jpeg_backside.sp odb=$odir/jpeg_backside.odb"
exit 0
