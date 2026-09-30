set plat /home/wali2/backside/OpenROAD-flow-scripts/flow/platforms/gt2n
set res  /home/wali2/backside/OpenROAD-flow-scripts/flow/results/gt2n/gcd/base
read_lef     $plat/lef/gt2_tech.lef
read_lef     gt2_6t_w31_lvt_nofront_tap.lef
read_lef     $plat/lef/gt2_6t_TSV.lef
read_lef     $plat/lef/gt2_ntsv_via.lef
read_def     results/integ.def
read_liberty $plat/lib/gt2_6t_w31_lvt_tt_0p7v25c.lib
read_liberty $plat/lib/gt2_6t_TSV.lib
read_sdc     $res/3_place.sdc
source       $plat/setRC.tcl
set_pdnsim_net_voltage -net vdd -voltage 0.7
set_pdnsim_net_voltage -net vss -voltage 0.0
puts "===================== VDD ====================="
analyze_power_grid -net vdd -vsrc /tmp/vdd.vsrc
puts "===================== VSS ====================="
analyze_power_grid -net vss -vsrc /tmp/vss.vsrc
exit 0
