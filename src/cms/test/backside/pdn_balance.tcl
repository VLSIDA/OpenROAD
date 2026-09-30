# BALANCED PDN builder + verifier (codifies the width-vs-count method).
#
# Method (see ASSUMPTIONS.md / discussion 2026-07-05):
#   - IR needs TOTAL METAL (N*W); clock-track cost = N*(W + 2*0.224 clearance)
#     -> for fixed metal, FEWER-BUT-WIDER always wins the track budget.
#   - Width first (PDN_W, as wide as EM/DRC sensible), count from metal with
#     the redundancy floor (PDN_N >= 2), pitch DERIVED from core (never fixed).
#   - Sparseness cap = BPR reach: local drop along the 30ohm/um followpin
#     dV ~= 1/2 * j_row * R_bpr * (pitch/2)^2 must stay a small budget slice.
# Env: DESIGN (gcd), PDN_W um (0.4), PDN_N straps/net/direction (2)
set design [expr {[info exists ::env(DESIGN)] ? $::env(DESIGN) : "gcd"}]
set W      [expr {[info exists ::env(PDN_W)]  ? $::env(PDN_W)  : 0.4}]
set N      [expr {[info exists ::env(PDN_N)]  ? $::env(PDN_N)  : 2}]
if {$N < 2} { set N 2 }   ;# redundancy floor

set orfs /home/wali2/backside/OpenROAD-flow-scripts/flow
set res  $orfs/results/gt2n/$design/base
set plat $orfs/platforms/gt2n
set here [file dirname [file normalize [info script]]]
set rdir [file join $here results]

# PLACE_ODB / OUT_ODB env overrides let the die-shrink sweep run on
# alternate floorplans without touching the shared base results.
if {[info exists ::env(PLACE_ODB)]} { read_db $::env(PLACE_ODB) } else { read_db $res/3_place.odb }
foreach lib [lsort [glob $plat/lib/gt2_6t_w*_tt_0p7v25c.lib]] { read_liberty $lib }
if {[info exists ::env(PLACE_SDC)]} { read_sdc $::env(PLACE_SDC) } else { read_sdc $res/3_place.sdc }
source $plat/setRC.tcl
set block [ord::get_db_block]
set dbu   [$block getDbUnitsPerMicron]
set core  [$block getCoreArea]
set cw [expr {([$core xMax]-[$core xMin])/double($dbu)}]
set ch [expr {([$core yMax]-[$core yMin])/double($dbu)}]

# pitch derived from core and requested count (alternating vdd/vss stripes:
# pdngen pitch = same-net pitch)
# per-axis pitch so BOTH nets get N straps in BOTH directions
set pitch_v  [expr {$cw / double($N)}]
set pitch_h  [expr {$ch / double($N)}]
# offset = pitch/4 centers the alternating vdd/vss pattern INSIDE the core:
# stripe centers at cw*(2k+1)/(4N) -- equal margin both sides, none on the edge
# (offset = pitch/2 put the last stripe exactly ON core.max -> half outside)
set offset_v [expr {$pitch_v / 4.0}]
set offset_h [expr {$pitch_h / 4.0}]
puts ">>> BALANCED PDN: $design core=${cw}x${ch}um  W=${W}um N=${N}/net -> pitchV=[format %.2f $pitch_v] pitchH=[format %.2f $pitch_h]um"

# BPR-reach check: worst local drop between straps along the followpin.
# j_row (A/um of rail) ~= P/(V*core_area) * row_pitch(0.144)
# guard: local drop <= 20% of the 17.5mV budget
set Ppeak [expr {[info exists ::env(P_PEAK_W)] ? $::env(P_PEAK_W) : 0}]
if {$Ppeak > 0} {
  set jrow [expr {$Ppeak/0.7/($cw*$ch) * 0.144}]
  set dv [expr {0.5 * $jrow * 30.0 * pow($pitch_v/2.0,2)}]
  puts [format ">>> BPR-reach: local drop %.2f mV (guard 3.5 mV) %s" \
      [expr {$dv*1e3}] [expr {$dv <= 3.5e-3 ? "OK" : "TOO SPARSE - raise N"}]]
}

# strip old PDN + stale block pins, rebuild balanced grid
foreach nn {vdd vss} {
  set net [$block findNet $nn]; if {$net eq "NULL"} continue
  set sws {}; foreach sw [$net getSWires] { lappend sws $sw }
  foreach sw $sws { odb::dbSWire_destroy $sw }
  set bts {}; foreach bt [$net getBTerms] { lappend bts $bt }
  foreach bt $bts { odb::dbBTerm_destroy $bt }
}
add_global_connection -net {vdd} -inst_pattern {.*} -pin_pattern {^vdd$} -power
add_global_connection -net {vss} -inst_pattern {.*} -pin_pattern {^vss$} -ground
global_connect
set_voltage_domain -name {CORE} -power {vdd} -ground {vss}
# no -pins: pin stripes get extended past the core toward the die boundary;
# we keep ALL power inside the core (supply enters via -vsrc points instead)
define_pdn_grid -name {bal} -voltage_domains {CORE}
add_pdn_stripe -grid {bal} -layer {BPR} -width {0.032} -pitch {0.144} -offset {0} -followpins
add_pdn_stripe -grid {bal} -layer {BM1} -width $W -pitch $pitch_v -offset $offset_v
add_pdn_stripe -grid {bal} -layer {BM2} -width $W -pitch $pitch_h -offset $offset_h
add_pdn_connect -grid {bal} -layers {BPR BM1}
add_pdn_connect -grid {bal} -layers {BM1 BM2}
# -skip_trim: pdngen otherwise trims stripe ends past the last via
# connection -- with only 2 same-net crossings the H straps got chopped to
# the span between their two via columns instead of the full core width
pdngen -skip_trim
# post-pass: clip any strap that still pokes past the core box
set clipped 0
foreach nn {vdd vss} {
  set net [$block findNet $nn]
  foreach sw [$net getSWires] {
    set boxes {}
    foreach sb [$sw getWires] { lappend boxes $sb }
    foreach sb $boxes {
      if {[$sb isVia]} continue
      set L [[$sb getTechLayer] getName]
      if {$L ne "BM1" && $L ne "BM2"} continue
      set x0 [$sb xMin]; set y0 [$sb yMin]; set x1 [$sb xMax]; set y1 [$sb yMax]
      set nx0 [expr {max($x0,[$core xMin])}]; set ny0 [expr {max($y0,[$core yMin])}]
      set nx1 [expr {min($x1,[$core xMax])}]; set ny1 [expr {min($y1,[$core yMax])}]
      if {$nx0 != $x0 || $ny0 != $y0 || $nx1 != $x1 || $ny1 != $y1} {
        set lay [[ord::get_db_tech] findLayer $L]
        set wst [$sb getWireShapeType]
        odb::dbSBox_create $sw $lay $nx0 $ny0 $nx1 $ny1 $wst
        odb::dbSBox_destroy $sb
        incr clipped
      }
    }
  }
}
puts ">>> clipped $clipped straps to the core box"
foreach nn {vdd vss} {
  set c1 0; set c2 0
  foreach sw [[$block findNet $nn] getSWires] { foreach s [$sw getWires] {
    if {[$s isVia]} continue
    set L [[$s getTechLayer] getName]
    if {$L eq "BM1"} { incr c1 } elseif {$L eq "BM2"} { incr c2 } } }
  puts ">>> $nn straps: BM1=$c1 BM2=$c2"
}

# verify: activity-driven PSM at worst case, ideal strap entry
set_propagated_clock [all_clocks]
set_power_activity -global -activity 1.0
foreach {nn volt} {vdd 0.7 vss 0.0} {
  set fh [open /tmp/${nn}_bal.vsrc w]
  foreach sw [[$block findNet $nn] getSWires] { foreach sb [$sw getWires] {
    if {[$sb isVia]} continue
    if {[[$sb getTechLayer] getName] ne "BM2"} continue
    set y [expr {([$sb yMin]+[$sb yMax])/2}]
    set w2 [expr {([$sb yMax]-[$sb yMin])/double($dbu)}]
    for {set x [$sb xMin]} {$x <= [$sb xMax]} {incr x [expr {int(2.0*$dbu)}]} {
      puts $fh "[format %.4f [expr {$x/double($dbu)}]],[format %.4f [expr {$y/double($dbu)}]],[format %.3f $w2],$volt" } } }
  close $fh
}
set_pdnsim_net_voltage -net vdd -voltage 0.7
set_pdnsim_net_voltage -net vss -voltage 0.0
puts "== VDD =="; analyze_power_grid -net vdd -vsrc /tmp/vdd_bal.vsrc
puts "== VSS =="; analyze_power_grid -net vss -vsrc /tmp/vss_bal.vsrc

# clock-corridor cost report
set cost [expr {$N * ($W + 0.448)}]
puts [format ">>> clock-corridor cost: %.2f um per layer (N=%d x (W=%.2f + 0.448))" $cost $N $W]
if {[info exists ::env(OUT_ODB)]} {
  write_db $::env(OUT_ODB)
  puts ">>> saved: $::env(OUT_ODB)"
} else {
  write_db  $rdir/${design}_pdn_balanced.odb
}
if {![info exists ::env(OUT_ODB)]} {
  write_def $rdir/${design}_pdn_balanced.def
  puts ">>> saved: $rdir/${design}_pdn_balanced.odb"
}
exit 0
