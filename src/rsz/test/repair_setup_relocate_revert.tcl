# Regression for the relocate-move accept/reject contract.
#
# The relocate move places a critical gate at a new location; when that move
# worsens global timing the framework must reject it and fully restore the
# design.  The revert restores the cell origin through the odb ECO journal,
# but a moved gate also changes the geometry (and hence the estimated RC) of
# every net it touches.  Nothing re-derived those parasitics on the revert, so
# a rejected relocation used to leave the wire RC frozen at the moved location:
# the timing of every later endpoint was then evaluated against stale, badly
# degraded parasitics, and relocate-only repair drove WNS/TNS far below the
# starting point (e.g. aes post-CTS WNS -3.84 -> -11.41 ns).
#
# gcd_nangate45 at this clock has several setup violations that the relocate
# move tries (and mostly must reject).  With the parasitics correctly
# invalidated on every instance move -- forward and revert -- relocate-only
# repair leaves setup timing no worse than the baseline.  -skip_last_gasp keeps
# the last-gasp phase from mixing in other move types, so this exercises
# "relocate" in isolation.
source "helpers.tcl"
read_liberty Nangate45/Nangate45_typ.lib
read_lef Nangate45/Nangate45.lef
read_def gcd_nangate45_placed.def
create_clock -period 0.3 [get_ports clk]

source Nangate45/Nangate45.rc
set_wire_rc -layer metal3
estimate_parasitics -placement

set init_wns [sta::worst_slack_cmd max]
set init_tns [sta::total_negative_slack_cmd max]
puts "initial QoR"
report_worst_slack -max
report_tns -digits 3

repair_timing -setup -sequence "relocate" -skip_last_gasp

set final_wns [sta::worst_slack_cmd max]
set final_tns [sta::total_negative_slack_cmd max]
puts "post relocate QoR"
report_worst_slack -max
report_tns -digits 3

# Relocate-only repair must never leave setup timing worse than the start.
# Allow a small numerical tolerance (1 ps) for STA noise.
set tol 1e-12
if { $final_wns < $init_wns - $tol } {
  puts "FAIL: relocate worsened WNS [sta::format_time $init_wns 3] ->\
 [sta::format_time $final_wns 3]"
} elseif { $final_tns < $init_tns - $tol } {
  puts "FAIL: relocate worsened TNS [sta::format_time $init_tns 3] ->\
 [sta::format_time $final_tns 3]"
} else {
  puts "PASS: relocate did not worsen setup timing"
}
