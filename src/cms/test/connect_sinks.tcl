# Create a clock mesh, then add proxy BTerms for the mesh buffers and
# sink BTerms for the flip-flops so a router can connect them to the mesh.
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

setup_proxy_bterms -clock clk -proxy_layer metal6
connect_sinks_to_mesh -clock clk -proxy_layer metal6

set block [ord::get_db_block]

set proxy_bterms 0
set sink_bterms 0
set unconnected 0
foreach bterm [$block getBTerms] {
  if { [$bterm getNet] == "NULL" } {
    incr unconnected
  }
  if { [string match "proxy_*" [$bterm getName]] } {
    incr proxy_bterms
  } elseif { [string match "sink_bterm_*" [$bterm getName]] } {
    incr sink_bterms
  }
}
puts "BTerms: [llength [$block getBTerms]] total,\
 $proxy_bterms proxy, $sink_bterms sink, $unconnected unconnected"

set def_file [make_result_file connect_sinks_out.def]
write_def $def_file
diff_files connect_sinks_out.defok $def_file
