# Verification for the DESIGNED PDN: inject worst-case power (alpha=1) and
# check IR + connectivity on both nets. Corrected tap LEF + backside source.
set plat /home/wali2/backside/OpenROAD-flow-scripts/flow/platforms/gt2n
set res  /home/wali2/backside/OpenROAD-flow-scripts/flow/results/gt2n/gcd/base
set rdir [file join [file dirname [file normalize [info script]]] results]

read_lef     $plat/lef/gt2_tech.lef
read_lef     /home/wali2/backside/OpenROAD/src/cms/test/gt2n/gt2_6t_w31_lvt_nofront_tap.lef
read_def     $rdir/gcd_pdn_designed.def
read_liberty $plat/lib/gt2_6t_w31_lvt_tt_0p7v25c.lib
read_sdc     $res/3_place.sdc
source       $plat/setRC.tcl
set_pdnsim_net_voltage -net vdd -voltage 0.7
set_pdnsim_net_voltage -net vss -voltage 0.0

# worst-case power P_peak = 0.353 mW (alpha=1), uniform over logic cells
set tot 3.53e-4
set insts {}
foreach i [[ord::get_db_block] getInsts] {
  set nm [[$i getMaster] getName]
  if {[string match "*filler*" $nm] || [string match "*tap*" $nm] \
      || [string match "*TSV*" $nm] || [string match "*decap*" $nm]} continue
  lappend insts $i
}
set per [expr {$tot/[llength $insts]}]
foreach i $insts { catch {set_pdnsim_inst_power -inst [$i getName] -power $per} }
puts ">>> injected [format %.3f [expr {$tot*1e3}]] mW over [llength $insts] cells"

puts "==== VDD ===="
analyze_power_grid -net vdd -vsrc /tmp/vdd_designed.vsrc
puts "==== VSS ===="
analyze_power_grid -net vss -vsrc /tmp/vss_designed.vsrc
exit 0
