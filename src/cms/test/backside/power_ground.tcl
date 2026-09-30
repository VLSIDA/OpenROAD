# Ground dynamic power in REAL net caps (report_power is broken on this PDK).
# Extract parasitics from the routed design, sum each net's total capacitance,
# split clock vs data, and compute P = 1/2 * C * V^2 * f * activity at 2 GHz.
set plat /home/wali2/backside/OpenROAD-flow-scripts/flow/platforms/gt2n
set res  /home/wali2/backside/OpenROAD-flow-scripts/flow/results/gt2n/gcd/base

read_db      $res/5_route.odb
read_liberty $plat/lib/gt2_6t_w31_lvt_tt_0p7v25c.lib
read_sdc     $res/5_route.sdc
source       $plat/setRC.tcl
estimate_parasitics -global_routing

set V   0.7
set f   2.0e9          ;# 500ps period, time_unit 1ps -> 2 GHz
set a_d 0.2            ;# data switching activity (transitions/cycle)
set a_c 2.0            ;# clock toggles twice per cycle

set Cc 0.0; set Cd 0.0; set nn 0
foreach net [get_nets *] {
  if {[catch {set cap [get_property $net capacitance]}]} { continue }
  if {$cap eq "" || $cap <= 0} { continue }
  incr nn
  # is this a clock net? (driven through the clock) -- approximate by name
  set name [get_property $net name]
  if {[string match -nocase "*clk*" $name] || [string match -nocase "*clock*" $name]} {
    set Cc [expr {$Cc + $cap}]
  } else {
    set Cd [expr {$Cd + $cap}]
  }
}
set Ctot [expr {$Cc + $Cd}]
set Pc [expr {0.5 * $Cc * $V*$V * $f * $a_c}]
set Pd [expr {0.5 * $Cd * $V*$V * $f * $a_d}]
set P  [expr {$Pc + $Pd}]
puts "==================== GROUNDED POWER ===================="
puts [format ">>> nets=%d  C_clock=%.4g F  C_data=%.4g F  C_total=%.4g F" $nn $Cc $Cd $Ctot]
puts [format ">>> f=%.2g Hz  V=%.2g V  a_clk=%.1f  a_data=%.2f" $f $V $a_c $a_d]
puts [format ">>> P_clock=%.4g W  P_data=%.4g W  P_total=%.4g W (%.3f mW)" $Pc $Pd $P [expr {$P*1e3}]]
exit 0
