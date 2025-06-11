source "helpers.tcl"
read_lef Nangate45/Nangate45.lef
read_liberty Nangate45/Nangate45_typ.lib
read_def "16sinks.def"

create_clock -period 5 clk

set_wire_rc -clock -layer metal3
create_mesh
report_cms

set def_file [make_result_file mesh_create.def]
write_def $def_file
diff_files mesh_create.defok $def_file
