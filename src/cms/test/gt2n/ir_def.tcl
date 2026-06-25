# Baseline static IR on clean gcd, read from LEF+DEF so the tap master is built
# from the CORRECTED LEF (frontside M1 power removed -> no floating islands that
# fragment the net and hijack PSM's connectivity BFS). Supply source is fed on
# the backside BM4 straps via -vsrc (PSM's auto source targets the frontside top
# layer, wrong for backside power).
set plat /home/wali2/backside/OpenROAD-flow-scripts/flow/platforms/gt2n
set res  /home/wali2/backside/OpenROAD-flow-scripts/flow/results/gt2n/gcd/base

read_lef     $plat/lef/gt2_tech.lef
read_lef     gt2_6t_w31_lvt_nofront_tap.lef
read_def     $res/6_final.def
read_liberty $plat/lib/gt2_6t_w31_lvt_tt_0p7v25c.lib
read_sdc     $res/6_final.sdc
source       $plat/setRC.tcl   ;# per-layer R/C incl. backside (needed by PSM)

set_pdnsim_net_voltage -net vdd -voltage 0.7
set_pdnsim_net_voltage -net vss -voltage 0.0

puts "================== VDD =================="
analyze_power_grid -net vdd -vsrc /tmp/vdd.vsrc
puts "================== VSS =================="
analyze_power_grid -net vss -vsrc /tmp/vss.vsrc
exit 0
