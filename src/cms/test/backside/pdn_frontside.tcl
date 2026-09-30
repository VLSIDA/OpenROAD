# FRONTSIDE PDN builder + verifier for GT2N (the BSPDN-vs-FSPDN control).
#
# Architecture (classic BPR-with-frontside-delivery, pre-BSPDN era):
#   cells power from the BPR rails (their only power pins), the rails are fed
#   from ABOVE through gt2_6t_tapfspdn_* tap cells (M1 power pins bridged to
#   BPR inside the cell). Tap cells are placed in FULL-HEIGHT COLUMNS (one tap
#   per row per column) so the vertical M1 power stripes drawn over the tap
#   pins never cross another cell's M1. Stack: M1 columns -> M6 (H) straps ->
#   M7 (V) -> M8 (H, supply entry) -- classic frontside global power on the
#   thick top metals. NO backside metal is used.
#
# Env: DESIGN (ibex), PDN_W um (0.4), PDN_N straps/net/direction (2),
#      TAP_SITES tap-column pitch in sites of 0.084um (24 -> 2.016um)
set design    [expr {[info exists ::env(DESIGN)]    ? $::env(DESIGN)    : "ibex"}]
set W         [expr {[info exists ::env(PDN_W)]     ? $::env(PDN_W)     : 0.4}]
set N         [expr {[info exists ::env(PDN_N)]     ? $::env(PDN_N)     : 2}]
set tap_sites [expr {[info exists ::env(TAP_SITES)] ? $::env(TAP_SITES) : 24}]
if {$N < 2} { set N 2 }

set orfs /home/wali2/backside/OpenROAD-flow-scripts/flow
set res  $orfs/results/gt2n/$design/base
set plat $orfs/platforms/gt2n
set here [file dirname [file normalize [info script]]]
set rdir [file join $here results]
file mkdir $rdir

read_db $res/3_place.odb
foreach lib [lsort [glob $plat/lib/gt2_6t_w*_tt_0p7v25c.lib]] { read_liberty $lib }
read_sdc $res/3_place.sdc
source $plat/setRC.tcl
# setRC covers V0/BV0 but NOT the VSD cut (nTSV) -- unset via resistance makes
# PSM's G matrix structurally singular once nTSVs join the power path. The
# nano-TSV power tap resistance is a platform parameter; default 30 ohm
# (same order as BV0=25). Override with FSPDN_NTSV_R.
set ntsv_r [expr {[info exists ::env(FSPDN_NTSV_R)] ? $::env(FSPDN_NTSV_R) : 30.0}]
set_layer_rc -via VSD -resistance $ntsv_r
puts ">>> VSD (nTSV) via resistance: $ntsv_r ohm"
set block [ord::get_db_block]
set tech  [ord::get_db_tech]
set dbu   [$block getDbUnitsPerMicron]
set core  [$block getCoreArea]
set cw [expr {([$core xMax]-[$core xMin])/double($dbu)}]
set ch [expr {([$core yMax]-[$core yMin])/double($dbu)}]

set tap_master_name gt2_6t_tapfspdn_w31_lvt
set tapm [[ord::get_db] findMaster $tap_master_name]
if {$tapm eq "NULL"} { error "tap master $tap_master_name not found" }
set site_w 0.084
set tap_pitch [expr {$tap_sites * $site_w}]

# strap pitches from core + count (balanced method: width first, derived pitch)
set pitch_v  [expr {$cw / double($N)}]
set pitch_h  [expr {$ch / double($N)}]
set offset_v [expr {$pitch_v / 4.0}]
set offset_h [expr {$pitch_h / 4.0}]
puts ">>> FRONTSIDE PDN: $design core=${cw}x${ch}um W=${W} N=${N} tap_pitch=${tap_pitch}um"

# ---------- strip the existing (backside) PDN ----------
foreach nn {vdd vss} {
  set net [$block findNet $nn]; if {$net eq "NULL"} continue
  set sws {}; foreach sw [$net getSWires] { lappend sws $sw }
  foreach sw $sws { odb::dbSWire_destroy $sw }
  set bts {}; foreach bt [$net getBTerms] { lappend bts $bt }
  foreach bt $bts { odb::dbBTerm_destroy $bt }
}

# ---------- tap columns: one tapfspdn per row per column ----------
# collect FIXED/FIRM obstacles once
set fixed_boxes {}
foreach inst [$block getInsts] {
  set st [$inst getPlacementStatus]
  if {$st eq "FIRM" || $st eq "LOCKED" || $st eq "FIXED"} {
    lappend fixed_boxes [$inst getBBox]
  }
}
set ncols 0; set ntaps 0; set nskip 0
set tap_w_dbu [$tapm getWidth]
set x0 [$core xMin]
set colxs {}
for {set cx [expr {$x0 + int($tap_pitch*$dbu/2)}]} {$cx < [$core xMax]-$tap_w_dbu} \
    {incr cx [expr {int($tap_pitch*$dbu)}]} {
  lappend colxs $cx; incr ncols
}
foreach row [$block getRows] {
  set rb   [$row getBBox]
  set ry   [$rb yMin]
  set rlox [$rb xMin]
  set rhix [$rb xMax]
  set ror  [$row getOrient]
  foreach cx $colxs {
    # snap the column x to this row's site grid
    set ox [expr {$rlox + int(round(($cx - $rlox)/double(int($site_w*$dbu)))) * int($site_w*$dbu)}]
    if {$ox < $rlox || $ox + $tap_w_dbu > $rhix} { incr nskip; continue }
    # skip if it would overlap a FIXED cell
    set clash 0
    foreach fb $fixed_boxes {
      if {$ox < [$fb xMax] && [$fb xMin] < $ox+$tap_w_dbu \
          && $ry < [$fb yMax] && [$fb yMin] < $ry+int(0.144*$dbu)} { set clash 1; break }
    }
    if {$clash} { incr nskip; continue }
    set ti [odb::dbInst_create $block $tapm "fsptap_${ntaps}"]
    $ti setOrient $ror
    $ti setLocation $ox $ry
    $ti setPlacementStatus FIRM
    incr ntaps
  }
}
puts ">>> placed $ntaps tapfspdn cells in $ncols columns ($nskip sites skipped)"

# ---------- global connect + grid ----------
add_global_connection -net {vdd} -inst_pattern {.*} -pin_pattern {^vdd$} -power
add_global_connection -net {vss} -inst_pattern {.*} -pin_pattern {^vss$} -ground
global_connect
set_voltage_domain -name {CORE} -power {vdd} -ground {vss}
define_pdn_grid -name {fsp} -voltage_domains {CORE}
# BPR followpin rails: still the cell supply, now fed from above via the taps
add_pdn_stripe -grid {fsp} -layer {BPR} -width {0.032} -pitch {0.144} -offset {0} -followpins
# M1 power columns over the tap pins (vdd strip center +0.021, vss +0.063 in-cell)
set m1_off [expr {$tap_pitch/2.0 + 0.021}]
add_pdn_stripe -grid {fsp} -layer {M1} -width {0.014} -pitch $tap_pitch \
    -offset $m1_off -spacing {0.028}
# frontside global straps: M6 (H) + M7 (V) + M8 (H, supply entry)
add_pdn_stripe -grid {fsp} -layer {M6} -width $W -pitch $pitch_h -offset $offset_h
add_pdn_stripe -grid {fsp} -layer {M7} -width $W -pitch $pitch_v -offset $offset_v
add_pdn_stripe -grid {fsp} -layer {M8} -width $W -pitch $pitch_h -offset [expr {$offset_h*3.0}]
add_pdn_connect -grid {fsp} -layers {M1 M6}
add_pdn_connect -grid {fsp} -layers {M6 M7}
add_pdn_connect -grid {fsp} -layers {M7 M8}
pdngen -skip_trim

# clip straps to the core box (same post-pass as the balanced builder)
set clipped 0
foreach nn {vdd vss} {
  set net [$block findNet $nn]
  foreach sw [$net getSWires] {
    set boxes {}
    foreach sb [$sw getWires] { lappend boxes $sb }
    foreach sb $boxes {
      if {[$sb isVia]} continue
      set L [[$sb getTechLayer] getName]
      if {$L ne "M6" && $L ne "M7" && $L ne "M8" && $L ne "M1"} continue
      set bx0 [$sb xMin]; set by0 [$sb yMin]; set bx1 [$sb xMax]; set by1 [$sb yMax]
      set nx0 [expr {max($bx0,[$core xMin])}]; set ny0 [expr {max($by0,[$core yMin])}]
      set nx1 [expr {min($bx1,[$core xMax])}]; set ny1 [expr {min($by1,[$core yMax])}]
      if {$nx0 != $bx0 || $ny0 != $by0 || $nx1 != $bx1 || $ny1 != $by1} {
        set lay [$tech findLayer $L]
        set wst [$sb getWireShapeType]
        odb::dbSBox_create $sw $lay $nx0 $ny0 $nx1 $ny1 $wst
        odb::dbSBox_destroy $sb
        incr clipped
      }
    }
  }
}
puts ">>> clipped $clipped straps to the core box"

# ---------- per-tap BPR<->M1 via stacks ----------
# Each tapfspdn cell internally bridges its M1 power pin down to the BPR rail
# (M1 -V0-> M0 -VSD/nTSV-> BPR). PSM treats cell pins as current LOADS, never
# as conductors, so that internal bridge is invisible to IR analysis and the
# BPR network would float (singular G matrix). Make the bridge explicit: at
# EACH PLACED TAP, drop the real via stack into the power swires -- exactly one
# stack per tap, exactly where the tap's silicon provides it.
set v0  [$tech findVia V0_0]
set ntv [$tech findVia nTSV]
if {$v0 eq "NULL" || $ntv eq "NULL"} { error "V0_0/nTSV tech vias missing" }
foreach nn {vdd vss} {
  set tap_sw($nn) [lindex [[$block findNet $nn] getSWires] 0]
  if {$tap_sw($nn) eq ""} { error "no swire on $nn" }
}
set row_h   [expr {int(0.144*$dbu)}]
set vdd_xo  [expr {int(0.021*$dbu)}]   ;# vdd M1 strip center (cell-local)
set vss_xo  [expr {int(0.063*$dbu)}]   ;# vss M1 strip center
set nvia 0
foreach inst [$block getInsts] {
  if {![string match "fsptap_*" [$inst getName]]} { continue }
  set bb [$inst getBBox]
  set ox [$bb xMin]; set oy [$bb yMin]
  set orr [$inst getOrient]
  # R0: vdd rail at the row TOP (oy+0.144), vss at the row BOTTOM (oy).
  # MX (y-flip): swapped.
  if {$orr eq "R0"} {
    set yv [expr {$oy + $row_h}]; set yg $oy
  } else {
    set yv $oy; set yg [expr {$oy + $row_h}]
  }
  foreach {nn xo yy} [list vdd $vdd_xo $yv vss $vss_xo $yg] {
    set x [expr {$ox + $xo}]
    odb::dbSBox_create $tap_sw($nn) $v0  $x $yy "STRIPE"
    odb::dbSBox_create $tap_sw($nn) $ntv $x $yy "STRIPE"
    incr nvia 2
  }
}
puts ">>> added $nvia tap via shapes (V0+nTSV per tap per net)"

# legalize cells displaced by the tap columns
detailed_placement -max_displacement {80 6}
check_placement -verbose

# supply BTerms on the M8 straps (package bumps) -- gives check_power_grid its
# terminals and anchors PSM sources on real grid nodes
foreach {nn stype} {vdd POWER vss GROUND} {
  set net [$block findNet $nn]
  set m8 ""
  foreach sw [$net getSWires] { foreach sb [$sw getWires] {
    if {![$sb isVia] && [[$sb getTechLayer] getName] eq "M8"} { set m8 $sb; break } }
    if {$m8 ne ""} break }
  if {$m8 eq ""} { error "no M8 strap on $nn for the supply bterm" }
  set bt [odb::dbBTerm_create $net "${nn}_fsp"]
  $bt setIoType INOUT
  $bt setSigType $stype
  set bp [odb::dbBPin_create $bt]
  set xm [expr {([$m8 xMin]+[$m8 xMax])/2}]
  odb::dbBox_create $bp [$tech findLayer M8] \
      [expr {$xm-int(0.2*$dbu)}] [$m8 yMin] [expr {$xm+int(0.2*$dbu)}] [$m8 yMax]
  $bp setPlacementStatus PLACED
  puts ">>> supply bterm ${nn}_fsp placed on M8"
}

# connectivity check (now meaningful: reports floating nodes per layer)
catch { check_power_grid -net vdd } msg1; puts ">>> check_power_grid vdd: $msg1"
catch { check_power_grid -net vss } msg2; puts ">>> check_power_grid vss: $msg2"

# ---------- IR verification: activity-driven PSM, supply enters on M8 ----------
set_propagated_clock [all_clocks]
set_power_activity -global -activity 1.0
foreach {nn volt} {vdd 0.7 vss 0.0} {
  set fh [open /tmp/${nn}_fsp.vsrc w]
  foreach sw [[$block findNet $nn] getSWires] { foreach sb [$sw getWires] {
    if {[$sb isVia]} continue
    if {[[$sb getTechLayer] getName] ne "M8"} continue
    set y [expr {([$sb yMin]+[$sb yMax])/2}]
    set w2 [expr {([$sb yMax]-[$sb yMin])/double($dbu)}]
    for {set x [$sb xMin]} {$x <= [$sb xMax]} {incr x [expr {int(2.0*$dbu)}]} {
      puts $fh "[format %.4f [expr {$x/double($dbu)}]],[format %.4f [expr {$y/double($dbu)}]],[format %.3f $w2],$volt" } } }
  close $fh
}
set_pdnsim_net_voltage -net vdd -voltage 0.7
set_pdnsim_net_voltage -net vss -voltage 0.0
puts "== VDD =="; analyze_power_grid -net vdd -vsrc /tmp/vdd_fsp.vsrc
puts "== VSS =="; analyze_power_grid -net vss -vsrc /tmp/vss_fsp.vsrc

# frontside track cost report: what the PDN takes from M6/M7/M8
foreach {ly per} [list M6 $pitch_h M7 $pitch_v M8 [expr {$pitch_h}]] {
  puts [format ">>> %s PDN track cost: %.1f%% (2 nets x N=%d x W=%.2fum over %.2fum pitch)" \
      $ly [expr {100.0*2*$W/$per}] $N $W $per]
}
write_db  $rdir/${design}_pdn_frontside.odb
write_def $rdir/${design}_pdn_frontside.def
puts ">>> saved: $rdir/${design}_pdn_frontside.odb"
exit 0
