# Baseline static IR-drop on the CLEAN gcd (no clock mesh / TSV), straight from
# the standard ORFS flow's 6_final.odb. Mirrors scripts/final_report.tcl.
set orfs /home/wali2/backside/OpenROAD-flow-scripts/flow
set res  $orfs/results/gt2n/gcd/base
set plat $orfs/platforms/gt2n
set rdir [file join [file dirname [file normalize [info script]]] results]
file mkdir $rdir

read_db      $res/6_final.odb
read_liberty $plat/lib/gt2_6t_w31_lvt_tt_0p7v25c.lib
read_sdc     $res/6_final.sdc

set_pdnsim_net_voltage -net vdd -voltage 0.7
set_pdnsim_net_voltage -net vss -voltage 0.0

# No bumps on gt2n -> use the top PDN layer (BM4) straps as the supply source.
puts ">>> ===================== VDD IR ====================="
analyze_power_grid -net vdd -source_type STRAPS \
  -error_file $rdir/ir_vdd.rpt -voltage_file $rdir/ir_vdd_volt.rpt
puts ">>> ===================== VSS IR ====================="
analyze_power_grid -net vss -source_type STRAPS \
  -error_file $rdir/ir_vss.rpt -voltage_file $rdir/ir_vss_volt.rpt
exit 0
