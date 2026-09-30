set design [expr {[info exists env(DESIGN)] ? $env(DESIGN) : "jpeg"}]
# PARAMETERIZED backside clock-mesh flow for JPEG.
#   FMAX   (env) -> create_sink_taps -capacity  (cluster size cap)
#   OUTDIR (env) -> where the deck / odb / guides / drc land
# Same physics as ${design}_backside_26x26.tcl (balanced PDN, x4 sink, 6ohm TSV, 25x25
# mesh, mesh-cap field-solve override). Inputs + spice models read from $base.
set base /home/wali2/backside/OpenROAD/src/cms/test/backside
set orfs /home/wali2/backside/OpenROAD-flow-scripts/flow
set res  $orfs/results/gt2n/$design/base
set plat $orfs/platforms/gt2n
set gt2n /home/wali2/backside/GT2N
set rdir [file join $base results]

set fmax [expr {[info exists env(FMAX)] ? $env(FMAX) : 16}]
set sbuf [expr {[info exists env(SBUF)] ? $env(SBUF) : "gt2_6t_buf_x4_w31_lvt"}]
set odir [expr {[info exists env(OUTDIR)] ? $env(OUTDIR) : $rdir}]
file mkdir $odir

# PDNODB / PLACE_SDC env overrides: run on an alternate floorplan (die-shrink sweep)
if {[info exists env(PDNODB)]} { read_db $env(PDNODB) } else { read_db $rdir/${design}_pdn_balanced.odb }
foreach lib [lsort [glob $plat/lib/gt2_6t_w*_tt_0p7v25c.lib]] { read_liberty $lib }
if {[info exists env(PLACE_SDC)]} { read_sdc $env(PLACE_SDC) } else { read_sdc $res/3_place.sdc }
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
# VT (env) -> threshold flavor for the WHOLE clock network (feeder+drivers+LCBs)
set vt [expr {[info exists env(VT)] ? $env(VT) : "lvt"}]
# W (env) -> device width family: w31 (default, wide) | w13 (narrow)
set wfam [expr {[info exists env(W)] ? $env(W) : "w31"}]
if {$feeder eq "low"} {
  set cts_bufs [list gt2_6t_buf_x2_${wfam}_$vt gt2_6t_buf_x3_${wfam}_$vt gt2_6t_buf_x4_${wfam}_$vt]
} else {
  set cts_bufs {}
  foreach xx {x1 x2 x3 x4 x6 x8 x10 x12} { lappend cts_bufs gt2_6t_buf_${xx}_${wfam}_$vt }
}
set pitch [expr {[info exists env(PITCH)] ? $env(PITCH) : 3.2}]
# MBUF (env) -> mesh DRIVER buffer (scale with pitch: sparser mesh -> bigger driver)
set mbuf [expr {[info exists env(MBUF)] ? $env(MBUF) : "gt2_6t_buf_x4_w31_lvt"}]
puts ">>> feeder=$feeder  pitch=$pitch  mesh_driver=$mbuf  cts_buffers=$cts_bufs"
set_placement_padding -masters $cts_bufs -left 1 -right 1
# CHECKER=1 -> drivers at every other intersection (checkerboard); wires unchanged
set mesh_flags {}
if {[info exists env(CHECKER)] && $env(CHECKER)} { lappend mesh_flags -checkerboard_buffers }
create_clock_mesh -clock $clk -h_layer BM2 -v_layer BM1 -pitch $pitch \
    -buffers [list $mbuf] -cts_buffers $cts_bufs {*}$mesh_flags
detailed_placement -max_displacement {80 6}
setup_proxy_bterms -clock $clk -proxy_layer BM2
create_sink_taps -h_layer BM2 -v_layer BM1 \
    -buffer $sbuf -capacity $fmax -tsv_master gt2_6t_TSV
break_bpr_at_tsvs -halo 0.224 -relocate_rows 1
detailed_placement -max_displacement {80 6}
# sink_tap wires: TAPROUTE=router -> leave to GRT/DRT (ordinary nets);
# default -> author special wires NOW at the buffers' final (post-legalization)
# positions, so the wires won't dangle at pre-move spots
if {[info exists env(TAPROUTE)] && $env(TAPROUTE) eq "router"} {
  connect_sink_taps -use_router
} else {
  connect_sink_taps
}

# ---- route (frontside signal M2-M9; BPR blocked over core) ----
set block  [ord::get_db_block]
set tech   [ord::get_db_tech]
set bpr    [$tech findLayer BPR]
set core_r [$block getCoreArea]
set bpr_obs [odb::dbObstruction_create $block $bpr \
    [$core_r xMin] [$core_r yMin] [$core_r xMax] [$core_r yMax]]
# CLKLAYERS (env) -> clock routing window (taps + LCB distribution + feeder);
# pin access below the window is still legal (FF/buffer pins on M1).
set clklay [expr {[info exists env(CLKLAYERS)] ? $env(CLKLAYERS) : "M2-M9"}]
puts ">>> clock routing layers: $clklay"
# DERATE (env) -> remove this fraction of GRT capacity on M4-M9, emulating a
# congested SoC where the mid/upper frontside metals carry other traffic.
# The mesh's own special wires additionally obstruct whatever remains.
if {[info exists env(DERATE)]} {
  foreach dl {M4 M5 M6 M7 M8 M9} {
    set_global_routing_layer_adjustment $dl $env(DERATE)
  }
  puts ">>> DERATE: M4-M9 capacity reduced by [expr {100*$env(DERATE)}]%"
}
set_routing_layers -signal M2-M9 -clock $clklay
global_route -guide_file $odir/${design}_bk.guide -congestion_iterations 50 -verbose -congestion_report_file $odir/congestion.rpt
detailed_route -output_drc $odir/${design}_bk_drc.rpt -droute_end_iter [expr {[info exists env(DRTITER)] ? $env(DRTITER) : 0}]
odb::dbObstruction_destroy $bpr_obs

# ---- SPICE prep ----
estimate_parasitics -global_routing
capture_mesh_arrivals -clock $clk
# feeder power (root -> mesh-driver inputs): acyclic, STA-valid. The mesh
# drivers themselves are in the SPICE deck; this covers only clkbuf_* cells
# + their wires. Parse the 'Total' line under FEEDER POWER in the log.
puts ">>> FEEDER POWER (STA, clkbuf_* instances only):"
catch { report_power -instances [get_cells clkbuf_*] } fp_msg
if {$fp_msg ne ""} { puts "feeder-power note: $fp_msg" }
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

# mesh-layer caps: MESHCAP=analytic (default) keeps the analytic setRC.tcl caps
# on BM1/BM2 — same basis as every other layer; MESHCAP=fastercap overrides them
# with FasterCap field-solved values (analytic overestimates isolated backside
# wires ~2.7x, but mesh cap is ~0.2% of clock power, so this is conservative).
set meshcap [expr {[info exists env(MESHCAP)] ? $env(MESHCAP) : "analytic"}]
if {$meshcap eq "analytic"} {
  puts ">>> mesh-cap: ANALYTIC (setRC.tcl values kept, no FasterCap override)"
} else {
  foreach {ly cperum} {BM1 5.62e-5 BM2 7.43e-5} {
    set L [$tech findLayer $ly]
    if {$L eq "NULL"} continue
    set w [expr {[$L getWidth]/double($dbu)}]
    $L setCapacitance [expr {$cperum / $w}]
    puts ">>> mesh-cap override (FasterCap): $ly -> $cperum pF/um"
  }
}

write_mesh_spice -clock $clk -output $odir/${design}_backside.sp -vdd 0.7 \
    -spice_models [list $base/gt2_${wfam}_${vt}_tt_renamed.sp $gt2n/cdl/gt2_6t_${wfam}_${vt}.cdl] \
    -tsv_res 6

# FULL deck: feeder tree (root -> clkbuf_* -> driver inputs) simulated in SPICE
# too, driven by ONE pulse at the clock root. Single-basis skew/power (no STA
# feeder split); the mesh-only deck above is kept for DSE parity.
write_mesh_spice -clock $clk -output $odir/${design}_backside_full.sp -vdd 0.7 \
    -spice_models [list $base/gt2_${wfam}_${vt}_tt_renamed.sp $gt2n/cdl/gt2_6t_${wfam}_${vt}.cdl] \
    -tsv_res 6 -full_tree

write_db $odir/${design}_backside.odb
puts ">>> DONE: deck=$odir/${design}_backside.sp odb=$odir/${design}_backside.odb"
exit 0
