# IR with an INJECTED total power (bypasses the broken gt2n liberty power model).
# Distributes $env(TOTAL_MW) milliwatts uniformly over the placed cells and
# solves IR on the current PDN (results/gcd_pdn_bm12.def). Uniform is a first-cut
# model -- real hotspots would be worse -- but it lets us see IR vs. real current.
set plat /home/wali2/backside/OpenROAD-flow-scripts/flow/platforms/gt2n
set res  /home/wali2/backside/OpenROAD-flow-scripts/flow/results/gt2n/gcd/base
set rdir [file join [file dirname [file normalize [info script]]] results]

read_lef     $plat/lef/gt2_tech.lef
read_lef     /home/wali2/backside/OpenROAD/src/cms/test/gt2n/gt2_6t_w31_lvt_nofront_tap.lef
read_def     $rdir/gcd_pdn_bm12.def
read_liberty $plat/lib/gt2_6t_w31_lvt_tt_0p7v25c.lib
read_sdc     $res/3_place.sdc
source       $plat/setRC.tcl
set_pdnsim_net_voltage -net vdd -voltage 0.7
set_pdnsim_net_voltage -net vss -voltage 0.0

set tot [expr {$::env(TOTAL_MW) * 1e-3}]
# power-drawing cells only (skip fillers / taps / TSVs)
set insts {}
foreach i [[ord::get_db_block] getInsts] {
  set nm [[$i getMaster] getName]
  if {[string match "*filler*" $nm] || [string match "*tap*" $nm] \
      || [string match "*TSV*" $nm] || [string match "*decap*" $nm]} continue
  lappend insts $i
}
set per [expr {$tot/[llength $insts]}]
foreach i $insts { catch {set_pdnsim_inst_power -inst [$i getName] -power $per} }
puts ">>> INJECTED $::env(TOTAL_MW) mW over [llength $insts] cells ([format %.3f [expr {$per*1e6}]] uW each)"

analyze_power_grid -net vdd -vsrc /tmp/vdd_bm12.vsrc
exit 0
