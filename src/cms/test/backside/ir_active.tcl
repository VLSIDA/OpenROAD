# IR with REALISTIC switching activity so PSM's current reflects real operation
# (not the ~0-current default). Same corrected-tap-LEF + backside-source setup.
set plat /home/wali2/backside/OpenROAD-flow-scripts/flow/platforms/gt2n
set res  /home/wali2/backside/OpenROAD-flow-scripts/flow/results/gt2n/gcd/base
set rdir [file join [file dirname [file normalize [info script]]] results]

read_lef     $plat/lef/gt2_tech.lef
read_lef     /home/wali2/backside/OpenROAD/src/cms/test/gt2n/gt2_6t_w31_lvt_nofront_tap.lef
read_def     $rdir/gcd_pdn_bm12.def
read_liberty $plat/lib/gt2_6t_w31_lvt_tt_0p7v25c.lib
read_sdc     $res/3_place.sdc
source       $plat/setRC.tcl

set_propagated_clock [all_clocks]
# realistic activity: clock toggles every cycle; data nets ~20% switching
set_power_activity -global -activity 0.2 -duty 0.5

set_pdnsim_net_voltage -net vdd -voltage 0.7
set_pdnsim_net_voltage -net vss -voltage 0.0

puts "==================== report_power (activity 0.2) ===================="
report_power
puts "==================== VDD IR ===================="
analyze_power_grid -net vdd -vsrc /tmp/vdd_bm12.vsrc
puts "==================== VSS IR ===================="
analyze_power_grid -net vss -vsrc /tmp/vss_bm12.vsrc
exit 0
