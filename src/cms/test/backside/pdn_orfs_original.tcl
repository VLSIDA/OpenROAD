# Rebuild the ORIGINAL upstream ORFS gt2n PDN (pre-worksheet-resize):
#   BPR followpins + BM1 0.224/pairs + BM2 0.448/pairs @1.792um, pins BM2
set orfs /home/wali2/backside/OpenROAD-flow-scripts/flow
set res  $orfs/results/gt2n/gcd/base
set plat $orfs/platforms/gt2n
set rdir [file join [file dirname [file normalize [info script]]] results]
read_db $res/3_place.odb
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
define_pdn_grid -name {grid} -voltage_domains {CORE} -pins {BM2}
add_pdn_stripe -grid {grid} -layer {BPR} -width {0.032} -pitch {0.144} -offset {0} -followpins
add_pdn_stripe -grid {grid} -layer {BM1} -width {0.224} -spacing {0.112} -pitch {1.792} -offset {0.896}
add_pdn_stripe -grid {grid} -layer {BM2} -width {0.448} -spacing {0.112} -pitch {1.792} -offset {0.896}
add_pdn_connect -grid {grid} -layers {BPR BM1}
add_pdn_connect -grid {grid} -layers {BM1 BM2}
pdngen
foreach nn {vdd vss} {
  set c1 0; set c2 0
  foreach sw [[$block findNet $nn] getSWires] { foreach s [$sw getWires] {
    if {[$s isVia]} continue
    set L [[$s getTechLayer] getName]
    if {$L eq "BM1"} { incr c1 } elseif {$L eq "BM2"} { incr c2 } } }
  puts ">>> $nn straps: BM1=$c1 BM2=$c2"
}
write_db $rdir/gcd_pdn_orfs.odb
puts ">>> saved: $rdir/gcd_pdn_orfs.odb"
exit 0
