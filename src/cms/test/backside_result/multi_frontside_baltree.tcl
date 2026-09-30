set design [expr {[info exists env(DESIGN)] ? $env(DESIGN) : "jpeg"}]
# FRONTSIDE clock-mesh flow for JPEG -- apples-to-apples with the BACKSIDE
# baltree sweep: SAME balanced PDN, SAME full/low LVT feeder, SAME pitch;
# only difference is the mesh is on frontside M5/M6 (no TSVs) instead of
# backside BM1/BM2. FFs connect directly to the mesh (connect_sinks_to_mesh),
# so there is no sink-buffer / F_max sweep on the frontside -- one config per
# feeder. FEEDER env: full (x1..x12) | low (x2/x3/x4).
set base /home/wali2/backside/OpenROAD/src/cms/test/backside
set orfs /home/wali2/backside/OpenROAD-flow-scripts/flow
set res  $orfs/results/gt2n/$design/base
set plat $orfs/platforms/gt2n
set gt2n /home/wali2/backside/GT2N
set rdir [file join $base results]

set odir [expr {[info exists env(OUTDIR)] ? $env(OUTDIR) : [file join $base results]}]
file mkdir $odir

# PDNODB / PLACE_SDC env overrides: run on an alternate floorplan (die-shrink sweep)
if {[info exists env(PDNODB)]} { read_db $env(PDNODB) } else { read_db $rdir/${design}_pdn_balanced.odb }
foreach lib [lsort [glob $plat/lib/gt2_6t_w*_tt_0p7v25c.lib]] { read_liberty $lib }
if {[info exists env(PLACE_SDC)]} { read_sdc $env(PLACE_SDC) } else { read_sdc $res/3_place.sdc }

detailed_placement
source $plat/setRC.tcl
set clk ""
foreach c [sta::all_clocks] { set clk [get_name $c]; break }

# feeder buffer set (single VT = LVT). full -> x1..x12 ; low -> x2/x3/x4
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
puts ">>> jpeg FRONTSIDE mesh: clock=$clk  feeder=$feeder  outdir=$odir"

# ---- frontside mesh; PITCH env (=backside) ----
set pitch [expr {[info exists env(PITCH)] ? $env(PITCH) : 3.2}]
# MBUF (env) -> mesh DRIVER buffer (scale with pitch: sparser mesh -> bigger driver)
set mbuf [expr {[info exists env(MBUF)] ? $env(MBUF) : "gt2_6t_buf_x4_w31_lvt"}]
# MESHLAYERS (env) -> which metal pair carries the mesh: M5M6 (default) or M7M8.
# v_layer = odd (vertical), h_layer = even (horizontal) in gt2n.
set meshlay [expr {[info exists env(MESHLAYERS)] ? $env(MESHLAYERS) : "M5M6"}]
if {$meshlay eq "M7M8"} {
  set v_lay M7; set h_lay M8
} else {
  set v_lay M5; set h_lay M6
}
puts ">>> FS feeder=$feeder  pitch=$pitch  mesh_driver=$mbuf  mesh_layers=$v_lay/$h_lay"
set_placement_padding -masters $cts_bufs -left 1 -right 1
# CHECKER=1 -> drivers at every other intersection (checkerboard); wires unchanged
set mesh_flags {}
if {[info exists env(CHECKER)] && $env(CHECKER)} { lappend mesh_flags -checkerboard_buffers }
create_clock_mesh -clock $clk -h_layer $h_lay -v_layer $v_lay -pitch $pitch \
    -buffers [list $mbuf] -cts_buffers $cts_bufs {*}$mesh_flags
detailed_placement -max_displacement {80 6}
setup_proxy_bterms -clock $clk -proxy_layer $h_lay
# SINKTIER (env): direct (default) -> FFs stub straight onto the mesh;
#                 lcb -> backside-style sink-buffer tier, WITHOUT TSVs: LCB
#                 input routed to a BTerm on the nearest mesh wire (-no_tsv).
set sinktier [expr {[info exists env(SINKTIER)] ? $env(SINKTIER) : "direct"}]
if {$sinktier eq "lcb"} {
  set fmax [expr {[info exists env(FMAX)] ? $env(FMAX) : 8}]
  set sbuf [expr {[info exists env(SBUF)] ? $env(SBUF) : "gt2_6t_buf_x4_w31_lvt"}]
  puts ">>> FS sink tier: LCB (buffer=$sbuf F_max=$fmax, no TSV)"
  create_sink_taps -h_layer $h_lay -v_layer $v_lay -buffer $sbuf \
      -capacity $fmax -no_tsv
  detailed_placement -max_displacement {80 6}
  connect_sink_taps -use_router
} else {
  connect_sinks_to_mesh -clock $clk -proxy_layer $h_lay
}

# MERGEROUTE=1 -> merge tap nets into the mesh net BEFORE routing, so DRT
# sees the tap junctions as legal same-net overlaps (for routability sweeps).
# SPICE decks are NOT valid after merging; pair with SKIPDECK=1.
if {[info exists env(MERGEROUTE)] && $env(MERGEROUTE)} {
  merge_mesh_nets -clock $clk
  puts ">>> MERGEROUTE: tap nets merged into mesh net before routing"
}

# ---- route: signal M2-M9; clock window via CLKLAYERS env (stock router) ----
set block  [ord::get_db_block]
set tech   [ord::get_db_tech]
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
global_route -guide_file $odir/${design}_fs.guide -congestion_iterations 50 -verbose -congestion_report_file $odir/congestion.rpt
detailed_route -output_drc $odir/${design}_fs_drc.rpt -droute_end_iter [expr {[info exists env(DRTITER)] ? $env(DRTITER) : 0}]

if {[info exists env(SKIPDECK)] && $env(SKIPDECK)} {
  write_db $odir/${design}_frontside.odb
  puts ">>> SKIPDECK: routed odb saved, SPICE prep skipped"
  exit 0
}
connect_proxy_bterms_to_mesh -clock $clk

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

# NO -tsv_res (frontside mesh, 0 TSVs)
write_mesh_spice -clock $clk -output $odir/${design}_frontside.sp -vdd 0.7 \
    -spice_models [list $base/gt2_${wfam}_${vt}_tt_renamed.sp $gt2n/cdl/gt2_6t_${wfam}_${vt}.cdl]

# FULL deck: feeder tree simulated in SPICE too (one pulse at the clock root);
# single-basis skew/power. Mesh-only deck above kept for DSE parity.
write_mesh_spice -clock $clk -output $odir/${design}_frontside_full.sp -vdd 0.7 \
    -spice_models [list $base/gt2_${wfam}_${vt}_tt_renamed.sp $gt2n/cdl/gt2_6t_${wfam}_${vt}.cdl] \
    -full_tree

write_db $odir/${design}_frontside.odb
puts ">>> FRONTSIDE DONE: deck=$odir/${design}_frontside.sp"
exit 0
