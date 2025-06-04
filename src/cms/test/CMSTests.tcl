# constant propgation thru registers
source "helpers.tcl"
read_liberty Nangate45/Nangate45_typ.lib
read_lef Nangate45/Nangate45.lef
read_verilog constant1.v
link_design top
report_cms