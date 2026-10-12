# Create a clock mesh with the set-cover strategy: buffers are placed only at
# the intersections needed to reach every sink within the buffers' load limit.
source "helpers.tcl"
read_lef Nangate45/Nangate45.lef
read_liberty Nangate45/Nangate45_typ.lib
read_def "16sinks.def"
source Nangate45/Nangate45.tracks

create_clock -period 5 clk
set_wire_rc -clock -layer metal3
set_layer_rc -layer metal4 -capacitance 0.2 -resistance 0.5
set_layer_rc -layer metal5 -capacitance 0.2 -resistance 0.5

create_clock_mesh -clock clk \
  -h_layer metal5 \
  -v_layer metal4 \
  -pitch 2.0 \
  -buffers {CLKBUF_X1 CLKBUF_X2 CLKBUF_X3} \
  -cts_buffers CLKBUF_X3 \
  -mesh_strategy set_cover

set block [ord::get_db_block]
set counts [dict create]
foreach inst [$block getInsts] {
  if { [string match "mesh_buf_*" [$inst getName]] } {
    dict incr counts [[$inst getMaster] getName]
  }
}
foreach master [lsort [dict keys $counts]] {
  puts "Mesh buffers $master: [dict get $counts $master]"
}

set def_file [make_result_file create_mesh_set_cover_out.def]
write_def $def_file
diff_files create_mesh_set_cover_out.defok $def_file
