# ============================================================================
# DESIGNED fine-layer PDN for gcd @ gt2n  (no BM3/BM4 -- BM1/BM2 only)
#
# Sized by the worksheet (2026-07-02):
#   Step 1  demand:    C_switched=0.72pF (routed parasitics), 2GHz, alpha=1
#                      -> P_peak=0.353mW, I_peak=0.504mA
#   Step 2  budget:    2.5% of 0.7V = 17.5mV
#   Step 3  R_grid:    <= 35 ohm
#   Step 4  IR needs:  1 strap ~ 43 ohm > 35 -> N >= 2 per net
#   Step 5  floors:    >=1/direction (connectivity across BPR breaks),
#                      >=2 (redundancy: one cut strap never isolates a region)
#   DESIGN: 2 straps/net in EACH direction, 0.2um wide, 3um pitch
#   Step 6  verified:  VDD 0.13% / VSS 0.04% @ alpha=1, PSM-0040 connected,
#                      EM 1.26 mA/um (OK)
# ============================================================================
set orfs /home/wali2/backside/OpenROAD-flow-scripts/flow
set res  $orfs/results/gt2n/gcd/base
set plat $orfs/platforms/gt2n
set rdir [file join [file dirname [file normalize [info script]]] results]
file mkdir $rdir

read_db      $res/3_place.odb
read_liberty $plat/lib/gt2_6t_w31_lvt_tt_0p7v25c.lib
set block [ord::get_db_block]

# strip old PDN straps AND stale BM4 block pins (they hijack PSM's BFS)
foreach nn {vdd vss} {
  set net [$block findNet $nn]; if {$net eq "NULL"} continue
  set sws {}; foreach sw [$net getSWires] { lappend sws $sw }
  foreach sw $sws { odb::dbSWire_destroy $sw }
  set bts {}; foreach bt [$net getBTerms] { lappend bts $bt }
  foreach bt $bts { odb::dbBTerm_destroy $bt }
}

# ---- the designed grid ----
add_global_connection -net {vdd} -inst_pattern {.*} -pin_pattern {^vdd$} -power
add_global_connection -net {vss} -inst_pattern {.*} -pin_pattern {^vss$} -ground
global_connect
set_voltage_domain -name {CORE} -power {vdd} -ground {vss}
define_pdn_grid -name {designed} -voltage_domains {CORE} -pins {BM2}
# per-row cell rails (the workhorse for local delivery)
add_pdn_stripe -grid {designed} -layer {BPR} -width {0.032} -pitch {0.144} -offset {0} -followpins
# 2 straps/net each direction, wide, uniform pitch
add_pdn_stripe -grid {designed} -layer {BM1} -width {0.2} -pitch {3.0} -offset {1.5}
add_pdn_stripe -grid {designed} -layer {BM2} -width {0.2} -pitch {3.0} -offset {1.5}
add_pdn_connect -grid {designed} -layers {BPR BM1}
add_pdn_connect -grid {designed} -layers {BM1 BM2}
pdngen

# report what was built
foreach nn {vdd vss} {
  set c1 0; set c2 0
  foreach sw [[$block findNet $nn] getSWires] { foreach s [$sw getWires] {
    if {[$s isVia]} continue
    set L [[$s getTechLayer] getName]
    if {$L eq "BM1"} { incr c1 } elseif {$L eq "BM2"} { incr c2 } } }
  puts ">>> $nn: BM1=$c1 BM2=$c2"
}

# backside supply feed points on the BM2 straps
proc gen_vsrc {block net_name volt fname} {
  set dbu [$block getDbUnitsPerMicron]; set step [expr {int(2.0*$dbu)}]
  set fh [open $fname w]
  foreach sw [[$block findNet $net_name] getSWires] { foreach sb [$sw getWires] {
    if {[$sb isVia]} continue
    if {[[$sb getTechLayer] getName] ne "BM2"} continue
    set y [expr {([$sb yMin]+[$sb yMax])/2}]
    set w [expr {([$sb yMax]-[$sb yMin])/double($dbu)}]
    for {set x [$sb xMin]} {$x <= [$sb xMax]} {incr x $step} {
      puts $fh "[format %.4f [expr {$x/double($dbu)}]],[format %.4f [expr {$y/double($dbu)}]],[format %.3f $w],$volt"
    } } }
  close $fh
}
gen_vsrc $block vdd 0.7 /tmp/vdd_designed.vsrc
gen_vsrc $block vss 0.0 /tmp/vss_designed.vsrc

write_db  $rdir/gcd_pdn_designed.odb
write_def $rdir/gcd_pdn_designed.def
puts ">>> DESIGNED PDN saved: $rdir/gcd_pdn_designed.odb"
exit 0
