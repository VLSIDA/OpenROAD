# IR-drop (PSM) on a finished mesh design odb. Balanced BSPDN, supply on BM2.
# Env: ODB (path to design odb), TAG (report name)
# Known risk: STA power on the cyclic mesh clock net -- catch and report.
set odb  $env(ODB)
set tag  [expr {[info exists env(TAG)] ? $env(TAG) : "ir"}]
set orfs /home/wali2/backside/OpenROAD-flow-scripts/flow
set plat $orfs/platforms/gt2n
set here [file dirname [file normalize [info script]]]

read_db $odb
foreach lib [lsort [glob $plat/lib/gt2_6t_w*_tt_0p7v25c.lib]] { read_liberty $lib }
read_liberty $plat/lib/gt2_6t_TSV.lib
read_sdc $orfs/results/gt2n/ibex/base/3_place.sdc
source $plat/setRC.tcl

set block [ord::get_db_block]
set dbu   [$block getDbUnitsPerMicron]

# worst-case activity (same convention as pdn_balance verification)
catch { set_propagated_clock [all_clocks] } pmsg
set_power_activity -global -activity 1.0

# vsrc points along the BM2 straps (backside supply feed)
foreach {nn volt} {vdd 0.7 vss 0.0} {
  set fh [open /tmp/${nn}_${tag}.vsrc w]
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
puts "== VDD ($tag) =="
catch { analyze_power_grid -net vdd -vsrc /tmp/vdd_${tag}.vsrc } m1
if {$m1 ne ""} { puts "vdd note: $m1" }
puts "== VSS ($tag) =="
catch { analyze_power_grid -net vss -vsrc /tmp/vss_${tag}.vsrc } m2
if {$m2 ne ""} { puts "vss note: $m2" }
exit 0
