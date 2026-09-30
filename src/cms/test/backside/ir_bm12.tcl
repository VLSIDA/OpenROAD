# Correct IR on the BM1/BM2 PDN: read from LEF+DEF so the tap master comes from
# the CORRECTED LEF (no frontside M1 power -> no floating islands that hijack
# PSM's connectivity BFS). Supply fed on the BM2 backside straps via -vsrc.
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

puts "================== VDD =================="
analyze_power_grid -net vdd -vsrc /tmp/vdd_bm12.vsrc -voltage_file $rdir/vdd_ir.rpt
puts "================== VSS =================="
analyze_power_grid -net vss -vsrc /tmp/vss_bm12.vsrc -voltage_file $rdir/vss_ir.rpt
exit 0
