# Probe RelocateMove vs RelocateGenerator/RelocateCandidate behavior.
# The relocate move places a critical gate at the midpoint between the driver
# of its most-critical input and its critical output load, shortening the wire
# along the critical path.  It only changes placement (no resizing, buffering,
# or netlist change), so it improves timing while leaving the design area
# unchanged.  -skip_last_gasp keeps the last-gasp phase (which uses the full
# default move set) from mixing in other move types, so this exercises
# "relocate" in isolation.
source "helpers.tcl"
read_liberty Nangate45/Nangate45_typ.lib
read_lef Nangate45/Nangate45.lef
read_def repair_setup1.def
create_clock -period 0.3 clk

source Nangate45/Nangate45.rc
set_wire_rc -layer metal3
estimate_parasitics -placement

puts "initial QoR"
report_worst_slack -max
report_tns -digits 3

repair_timing -setup -sequence "relocate" -skip_last_gasp

puts "post relocate QoR"
report_worst_slack -max
report_tns -digits 3
