source "helpers.tcl"
read_lef Nangate45/Nangate45.lef
read_liberty Nangate45/Nangate45_typ.lib
read_def "16sinks.def"

create_clock -period 5 clk

set_wire_rc -clock -layer metal3

report_cms -out_file results/CMSTestsOut

diff_files results/CMSTestsOut CMSTests.ok
