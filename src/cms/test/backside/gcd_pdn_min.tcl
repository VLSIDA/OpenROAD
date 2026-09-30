# Minimal PDN: BPR followpins + a SINGLE vertical BM1 strap (no BM2). The one
# strap ties every BPR row and takes the backside source directly (-pins BM1).
# This is the theoretical floor: >=1 vertical strap is needed to tie the rows.
set orfs /home/wali2/backside/OpenROAD-flow-scripts/flow
set res  $orfs/results/gt2n/gcd/base
set plat $orfs/platforms/gt2n
set rdir [file join [file dirname [file normalize [info script]]] results]

read_db      $res/3_place.odb
read_liberty $plat/lib/gt2_6t_w31_lvt_tt_0p7v25c.lib
set block [ord::get_db_block]

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
define_pdn_grid -name {mgrid} -voltage_domains {CORE} -pins {BM1}
add_pdn_stripe -grid {mgrid} -layer {BPR} -width {0.032} -pitch {0.144} -offset {0} -followpins
# ONE BM1 strap: pitch >> core so a single vdd/vss pair lands near center
add_pdn_stripe -grid {mgrid} -layer {BM1} -width {0.2} -spacing {0.3} -pitch {20.0} -offset {4.0}
add_pdn_connect -grid {mgrid} -layers {BPR BM1}
pdngen
set n1 0
foreach sw [[$block findNet vdd] getSWires] { foreach s [$sw getWires] {
  if {![$s isVia] && [[$s getTechLayer] getName] eq "BM1"} { incr n1 } } }
puts ">>> MIN PDN: vdd BM1 straps = $n1 (no BM2)"

# -vsrc along each BM1 (vertical) strap
proc gen_vsrc_bm1 {block net_name volt fname} {
  set dbu [$block getDbUnitsPerMicron]; set step [expr {int(2.0*$dbu)}]
  set fh [open $fname w]
  foreach sw [[$block findNet $net_name] getSWires] { foreach sb [$sw getWires] {
    if {[$sb isVia]} continue
    if {[[$sb getTechLayer] getName] ne "BM1"} continue
    set xc [expr {([$sb xMin]+[$sb xMax])/2}]; set y0 [$sb yMin]; set y1 [$sb yMax]
    set w [expr {([$sb xMax]-[$sb xMin])/double($dbu)}]
    for {set y $y0} {$y <= $y1} {incr y $step} {
      puts $fh "[format %.4f [expr {$xc/double($dbu)}]],[format %.4f [expr {$y/double($dbu)}]],[format %.3f $w],$volt"
    }
  }}
  close $fh
}
gen_vsrc_bm1 $block vdd 0.7 /tmp/vdd_bm12.vsrc
gen_vsrc_bm1 $block vss 0.0 /tmp/vss_bm12.vsrc

write_db  $rdir/gcd_pdn_bm12.odb
write_def $rdir/gcd_pdn_bm12.def
puts ">>> saved min-PDN odb + def"
exit 0
