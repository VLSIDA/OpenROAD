# Pure PDN experiment (NO clock mesh): drop coarse BM3/BM4, build the power grid
# on the FINE layers BM1 (vertical) + BM2 (horizontal) + BPR followpins, with a
# reduced strap count. Writes odb + DEF (for a corrected-tap-LEF IR run) and a
# -vsrc on the BM2 straps (the backside supply feed).
set orfs /home/wali2/backside/OpenROAD-flow-scripts/flow
set res  $orfs/results/gt2n/gcd/base
set plat $orfs/platforms/gt2n
set rdir [file join [file dirname [file normalize [info script]]] results]
file mkdir $rdir

read_db      $res/3_place.odb
read_liberty $plat/lib/gt2_6t_w31_lvt_tt_0p7v25c.lib
set block [ord::get_db_block]

# strip coarse PDN straps AND the old block PDN pins. The floorplan ran
# -pins {BM4}, leaving vdd/vss BTerm BPin boxes on BM4; if kept, they float
# (no BM4 straps) and, being the highest layer, hijack PSM's connectivity BFS.
foreach nn {vdd vss} {
  set net [$block findNet $nn]; if {$net eq "NULL"} continue
  set sws {}; foreach sw [$net getSWires] { lappend sws $sw }
  foreach sw $sws { odb::dbSWire_destroy $sw }
  set bts {}; foreach bt [$net getBTerms] { lappend bts $bt }
  foreach bt $bts { odb::dbBTerm_destroy $bt }
}
puts ">>> stripped old PDN straps + block pins"

# build BM1/BM2 fine-layer PDN (reduced straps: 2.16um pitch, 0.2um wide)
add_global_connection -net {vdd} -inst_pattern {.*} -pin_pattern {^vdd$} -power
add_global_connection -net {vss} -inst_pattern {.*} -pin_pattern {^vss$} -ground
global_connect
set_voltage_domain -name {CORE} -power {vdd} -ground {vss}
set pp [expr {[info exists ::env(PDN_PITCH)] ? $::env(PDN_PITCH) : 2.16}]
set po [expr {$pp/2.0}]
define_pdn_grid -name {fgrid} -voltage_domains {CORE} -pins {BM2}
add_pdn_stripe -grid {fgrid} -layer {BPR} -width {0.032} -pitch {0.144} -offset {0} -followpins
add_pdn_stripe -grid {fgrid} -layer {BM1} -width {0.2} -pitch $pp -offset $po
add_pdn_stripe -grid {fgrid} -layer {BM2} -width {0.2} -pitch $pp -offset $po
add_pdn_connect -grid {fgrid} -layers {BPR BM1}
add_pdn_connect -grid {fgrid} -layers {BM1 BM2}
pdngen
set n1 0; set n2 0
foreach sw [[$block findNet vdd] getSWires] { foreach s [$sw getWires] {
  if {[$s isVia]} continue
  set L [[$s getTechLayer] getName]
  if {$L eq "BM1"} { incr n1 } elseif {$L eq "BM2"} { incr n2 } } }
puts ">>> built BM1/BM2 PDN @ pitch ${pp}um  (vdd straps: BM1=$n1 BM2=$n2)"

# -vsrc points along each BM2 strap (the backside supply feed)
proc gen_vsrc {block net_name volt fname} {
  set dbu [$block getDbUnitsPerMicron]
  set step [expr {int(2.0*$dbu)}]
  set fh [open $fname w]
  set net [$block findNet $net_name]
  foreach sw [$net getSWires] { foreach sb [$sw getWires] {
    if {[$sb isVia]} continue
    if {[[$sb getTechLayer] getName] ne "BM2"} continue
    set x0 [$sb xMin]; set x1 [$sb xMax]
    set yc [expr {([$sb yMin]+[$sb yMax])/2}]
    set w  [expr {([$sb yMax]-[$sb yMin])/double($dbu)}]
    for {set x $x0} {$x <= $x1} {incr x $step} {
      puts $fh "[format %.4f [expr {$x/double($dbu)}]],[format %.4f [expr {$yc/double($dbu)}]],[format %.3f $w],$volt"
    }
  }}
  close $fh
}
gen_vsrc $block vdd 0.7 /tmp/vdd_bm12.vsrc
gen_vsrc $block vss 0.0 /tmp/vss_bm12.vsrc
puts ">>> wrote /tmp/{vdd,vss}_bm12.vsrc"

write_db  $rdir/gcd_pdn_bm12.odb
write_def $rdir/gcd_pdn_bm12.def
puts ">>> saved odb + def"
exit 0
