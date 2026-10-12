# Create a uniform clock mesh with a buffer at every grid intersection
# and a CTS tree from the clock root to the mesh buffers.
source "helpers.tcl"
read_lef Nangate45/Nangate45.lef
read_liberty Nangate45/Nangate45_typ.lib
read_def "16sinks.def"
source Nangate45/Nangate45.tracks

create_clock -period 5 clk
set_wire_rc -clock -layer metal3

create_clock_mesh -clock clk \
  -h_layer metal5 \
  -v_layer metal4 \
  -pitch 2.0 \
  -buffers CLKBUF_X3 \
  -cts_buffers CLKBUF_X3

set block [ord::get_db_block]

set mesh_buffers 0
foreach inst [$block getInsts] {
  if { [string match "mesh_buf_*" [$inst getName]] } {
    incr mesh_buffers
  }
}
puts "Mesh buffers: $mesh_buffers"

set mesh_net [$block findNet "clk_mesh"]
set swires 0
set shapes 0
foreach swire [$mesh_net getSWires] {
  incr swires
  incr shapes [llength [$swire getWires]]
}
puts "Mesh net [$mesh_net getName]: $swires swires, $shapes shapes"

set def_file [make_result_file create_mesh_out.def]
write_def $def_file
diff_files create_mesh_out.defok $def_file
