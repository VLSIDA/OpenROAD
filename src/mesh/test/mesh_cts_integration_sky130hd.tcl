source "helpers.tcl"
read_db "3_3_place_gp.odb"
read_liberty "sky130hd/lib/sky130_fd_sc_hd__tt_025C_1v80.lib"
read_sdc "3_place.sdc"
set clk_name "core_clock"
run_mesh -clock $clk_name
set buffer_list [list sky130_fd_sc_hd__clkbuf_1]


# creates mesh
create_clock_mesh \
    -clock $clk_name \
    -h_layer met2 \
    -v_layer met3 \
    -wire_width 0.1 \
    -pitch 5.0 \
    -buffers $buffer_list \
    -tree_layer met4

puts " Legalizing buffer placement..."
detailed_placement -max_displacement 1000
puts "  Buffer placement legalized"
connect_sinks_to_mesh -clock $clk_name
connect_buffers_to_mesh -clock $clk_name
puts "Routing the design."
set_routing_layers -signal met1-met5 -clock met1-met5
set_wire_rc -signal -layer met2
set_wire_rc -clock -layer met2
global_route -guide_file [make_result_file "mesh_cts_route.guide"] \
    -congestion_iterations 50
detailed_route -output_drc [make_result_file "mesh_cts_drc.rpt"] \
    -droute_end_iter 5

set odb_file [make_result_file "mesh_cts_integration_sky130hd.odb"]
puts "  -> Writing ODB to: $odb_file"
write_db $odb_file


set def_file [make_result_file "mesh_cts_integration_sky130hd.def"]
puts "  -> Writing DEF to: $def_file"
write_def $def_file


# set timing_file [make_result_file "mesh_cts_integration_sky130hd_timing.rpt"]
# puts "  -> Writing timing report to: $timing_file"
# set timing_fp [open $timing_file w]
# puts $timing_fp "Clock Mesh + CTS Integration Timing Report"
# puts $timing_fp "=========================================="
# puts $timing_fp "Clock: $clk_name"
# puts $timing_fp "Date: [clock format [clock seconds]]"
# puts $timing_fp ""
# close $timing_fp

# puts "OK: All output files written successfully!"

# puts "\nStep 7: Running validation checks..."

# set block [ord::get_db_block]
# set mesh_net [$block findNet "clk"]

# if { $mesh_net == "NULL" } {
#   utl::error MESH 400 "Mesh net 'clk' not found!"
# }
# puts "  OK: Mesh net 'clk' found"

# # Check clock root connection
# set root [$mesh_net get1stBTerm]
# if { $root != "NULL" } {
#   puts "  OK: Clock root connected: [$root getName]"
# } else {
#   puts "  WARN: Warning: No BTerm connected to mesh net"
# }

# # Check sink connections
# set sinks [$mesh_net getITerms]
# set expected_sinks 35
# set actual_sinks [llength $sinks]

# puts "  -> Sinks expected: $expected_sinks"
# puts "  -> Sinks connected: $actual_sinks"

# if { $actual_sinks >= $expected_sinks } {
#   puts "  OK: All expected sinks connected!"
# } else {
#   utl::warn MESH 403 "Expected $expected_sinks sinks, got $actual_sinks"
# }

# # Check for buffer instances (CTS integration)
# set all_insts [$block getInsts]
# set buffer_count 0
# set mesh_buffer_count 0
# set tree_buffer_count 0

# foreach inst $all_insts {
#   set master_name [[$inst getMaster] getName]
#   set inst_name [$inst getName]
#   if { [string match "*buf_*" $master_name] } {
#     incr buffer_count
#     if { [string match "mesh_buf_*" $inst_name] } {
#       incr mesh_buffer_count
#     } elseif { [string match "tree_L*" $inst_name] } {
#       incr tree_buffer_count
#     }
#   }
# }

# puts "  -> Total buffers in design: $buffer_count"
# puts "  -> CTS tree buffers (tree_L*): $tree_buffer_count"
# puts "  -> Mesh grid buffers (mesh_buf_*): $mesh_buffer_count"

# if { $tree_buffer_count > 0 } {
#   puts "  OK: CTS integration: Tree level buffers placed!"
# } else {
#   puts "  WARN: Warning: No tree buffers found"
# }

# if { $mesh_buffer_count > 0 } {
#   puts "  OK: CTS integration: Grid buffers placed at intersections!"
# } else {
#   puts "  WARN: Warning: No mesh buffers found"
# }

# # Check mesh wires on special wire (SWire)
# set swires [$mesh_net getSWires]
# set swire_count [llength $swires]
# puts "  -> Special wires (mesh grid): $swire_count"

# if { $swire_count > 0 } {
#   puts "  OK: Mesh grid wires created"
# } else {
#   utl::error MESH 404 "No mesh grid wires found!"
# }

# # ============================================================================
# # Step 8: Statistics Report
# # ============================================================================
# puts "\nStep 8: Final Statistics"
# puts "========================================"
# puts "Clock Network: $clk_name"
# puts "  Mesh net: clk"
# puts "  CTS tree buffers: $tree_buffer_count"
# puts "  Grid buffers: $mesh_buffer_count"
# puts "  Total CTS+Mesh buffers: [expr {$tree_buffer_count + $mesh_buffer_count}]"
# puts "  Clock sinks: $actual_sinks"
# puts "  Special wires: $swire_count"
# if { $root != "NULL" } {
#   puts "  Root connected: Yes"
# } else {
#   puts "  Root connected: No"
# }
# puts "========================================"

# # ============================================================================
# # Step 9: Optional - Compare with Golden Reference
# # ============================================================================
# puts "\nStep 9: Optional: Comparing with golden reference files..."

# # Note: Golden reference files would need to be generated first
# if { [file exists "mesh_cts_integration_sky130hd.defok"] } {
#   puts "  -> Comparing DEF to golden reference..."
#   diff_files "mesh_cts_integration_sky130hd.defok" $def_file
#   puts "  OK: DEF matches golden reference"
# } else {
#   puts "  WARN: No golden reference DEF found (skipping comparison)"
# }

# # Guide comparison disabled (guides no longer created)
# # if { [file exists "mesh_cts_integration_sky130hd.guideok"] } {
# #   puts "  -> Comparing guides to golden reference..."
# #   diff_files "mesh_cts_integration_sky130hd.guideok" $guide_file
# #   puts "  OK: Guides match golden reference"
# # } else {
# #   puts "  WARN: No golden reference guides found (skipping comparison)"
# # }

# # ============================================================================
# # Test Complete
# # ============================================================================
# puts "\n========================================"
# puts "OK: MESH + CTS INTEGRATION TEST PASSED!"
# puts "========================================"
# puts ""
# puts "Output files generated:"
# puts "  1. $odb_file"
# puts "  2. $def_file"
# puts "  3. $timing_file"
# puts ""
# puts "To view the results:"
# puts "  - Open GUI: ~/OpenROAD/build/bin/openroad -gui $odb_file"
# puts "  - View DEF: klayout $def_file"
# puts ""
# puts "Note: Mesh grid is stored as special wires (SWires) in the database"
# puts ""

 exit 0
