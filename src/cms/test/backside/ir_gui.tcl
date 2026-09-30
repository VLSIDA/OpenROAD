# IR-drop GUI analysis v2 - sources on ALL backside straps (BM1+BM2), setRC applied.
# Needs setRC.tcl and lib/ (gt2n liberty files) next to it.
# Usage: ODB=ibex_bslcb.odb TOTAL_MW=8 openroad -gui ir_gui.tcl
set odb   [expr {[info exists env(ODB)]      ? $env(ODB)      : "ibex_bslcb.odb"}]
set libd  [expr {[info exists env(LIBDIR)]   ? $env(LIBDIR)   : "./lib"}]
set totmw [expr {[info exists env(TOTAL_MW)] ? $env(TOTAL_MW) : 8.0}]

if {[catch {set _blk [ord::get_db_block]}] || $_blk eq "NULL"} { read_db $odb }
foreach lib [lsort [glob -nocomplain $libd/gt2_6t_w*_tt_0p7v25c.lib]] { read_liberty $lib }
source ./setRC.tcl
set block [ord::get_db_block]
set dbu   [$block getDbUnitsPerMicron]

# ---- strap inventory + vsrc points on every BM1/BM2 strap ----
proc gen_vsrc {block net_name volt fname} {
  set dbu [$block getDbUnitsPerMicron]
  set step [expr {int(2.0*$dbu)}]
  set fh [open $fname w]
  set net [$block findNet $net_name]
  array set cnt {BM1 0 BM2 0}
  set n 0
  foreach sw [$net getSWires] { foreach sb [$sw getWires] {
    if {[$sb isVia]} continue
    set ln [[$sb getTechLayer] getName]
    if {$ln ne "BM1" && $ln ne "BM2"} continue
    incr cnt($ln)
    set horiz [expr {([$sb xMax]-[$sb xMin]) > ([$sb yMax]-[$sb yMin])}]
    if {$horiz} {
      set yc [expr {([$sb yMin]+[$sb yMax])/2}]
      set w  [expr {([$sb yMax]-[$sb yMin])/double($dbu)}]
      for {set x [$sb xMin]} {$x <= [$sb xMax]} {incr x $step} {
        puts $fh "[format %.4f [expr {$x/double($dbu)}]],[format %.4f [expr {$yc/double($dbu)}]],[format %.3f $w],$volt"
        incr n }
    } else {
      set xc [expr {([$sb xMin]+[$sb xMax])/2}]
      set w  [expr {([$sb xMax]-[$sb xMin])/double($dbu)}]
      for {set y [$sb yMin]} {$y <= [$sb yMax]} {incr y $step} {
        puts $fh "[format %.4f [expr {$xc/double($dbu)}]],[format %.4f [expr {$y/double($dbu)}]],[format %.3f $w],$volt"
        incr n }
    }
  }}
  close $fh
  puts ">>> $net_name: straps BM1=$cnt(BM1) BM2=$cnt(BM2), $n vsrc points -> $fname"
}
gen_vsrc $block vdd 0.7 ./vdd.vsrc
gen_vsrc $block vss 0.0 ./vss.vsrc

set_pdnsim_net_voltage -net vdd -voltage 0.7
set_pdnsim_net_voltage -net vss -voltage 0.0

set insts {}
foreach i [$block getInsts] {
  set nm [[$i getMaster] getName]
  if {[string match "*filler*" $nm] || [string match "*tap*" $nm] \
      || [string match "*TSV*" $nm] || [string match "*decap*" $nm]} continue
  lappend insts $i
}
set per [expr {$totmw*1e-3/[llength $insts]}]
foreach i $insts { catch {set_pdnsim_inst_power -inst [$i getName] -power $per} }
puts ">>> injected $totmw mW over [llength $insts] cells"

set base [file rootname [file tail $odb]]
puts "==== VDD IR ===="
analyze_power_grid -net vdd -vsrc ./vdd.vsrc -voltage_file ./${base}_vdd_ir.rpt
puts "==== VSS IR ===="
analyze_power_grid -net vss -vsrc ./vss.vsrc -voltage_file ./${base}_vss_ir.rpt
puts ">>> reports: ${base}_vdd_ir.rpt / ${base}_vss_ir.rpt"
puts ">>> GUI: Tools -> Heat Maps -> IR Drop (net vdd)"
if {![gui::enabled]} { exit 0 }
