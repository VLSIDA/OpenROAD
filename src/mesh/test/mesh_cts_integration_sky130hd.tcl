source "helpers.tcl"
read_db "3_3_place_gp.odb"
read_liberty "sky130hd/lib/sky130_fd_sc_hd__tt_025C_1v80.lib"
read_sdc "3_place.sdc"

set clk_name "core_clock"
run_mesh -clock $clk_name
set buffer_list [list sky130_fd_sc_hd__clkbuf_1]

create_clock_mesh \
    -clock $clk_name \
    -h_layer met2 \
    -v_layer met3 \
    -pitch 5.0 \
    -buffers $buffer_list \
    -tree_layer met4

puts "Legalizing buffer placement..."
detailed_placement -max_displacement 1000
puts "Buffer placement legalized"

# Connect sinks to mesh (special routing)
connect_sinks_to_mesh -clock $clk_name
puts "Setting up proxy BTERMs on met4 for router-based buffer connections..."
setup_proxy_bterms -clock $clk_name -proxy_layer met4
puts "Routing the design..."
set_routing_layers -signal met1-met4 -clock met1-met4
set_wire_rc -signal -layer met2
set_wire_rc -clock -layer met2

global_route -guide_file [make_result_file "mesh_cts_route.guide"] \
    -congestion_iterations 50

detailed_route -output_drc [make_result_file "mesh_cts_drc.rpt"] \
    -droute_end_iter 2

# After routing, connect the proxy BTERMs to the mesh via via stacks
puts "Connecting proxy BTERMs to mesh..."
connect_proxy_bterms_to_mesh -clock $clk_name

# # === DRC Check Section ===
# puts "\n========================================"
# puts "DRC Validation"
# puts "========================================"

# set drc_file [make_result_file "mesh_cts_drc.rpt"]
# if {[file exists $drc_file]} {
#     set fp [open $drc_file r]
#     set drc_content [read $fp]
#     close $fp

#     set drc_trimmed [string trim $drc_content]
#     if {[string length $drc_trimmed] > 0} {
#         puts "WARNING: DRC violations found!"
#         puts "----------------------------------------"
#         # Show first 50 lines
#         set lines [split $drc_content "\n"]
#         set count 0
#         foreach line $lines {
#             puts $line
#             incr count
#             if {$count >= 50} {
#                 puts "... (truncated, see full report)"
#                 break
#             }
#         }
#         puts "----------------------------------------"
#         puts "Full DRC report: $drc_file"
#     } else {
#         puts "OK: No DRC violations!"
#     }
# } else {
#     puts "Note: DRC report file not found"
# }

# puts "========================================"

# Write outputs
set odb_file [make_result_file "mesh_cts_integration_sky130hd.odb"]
puts "Writing ODB to: $odb_file"
write_db $odb_file

set def_file [make_result_file "mesh_cts_integration_sky130hd.def"]
puts "Writing DEF to: $def_file"
write_def $def_file

# puts "\n========================================"
# puts "Test Complete"
# puts "========================================"
# puts "Output files:"
# puts "  - ODB: $odb_file"
# puts "  - DEF: $def_file"
# puts "  - DRC: [make_result_file "mesh_cts_drc.rpt"]"
# puts ""
# puts "To view in GUI:"
# puts "  ~/OpenROAD/build/bin/openroad -gui $odb_file"
# puts ""
# puts "To run external DRC (Magic):"
# puts "  magic -T sky130A $def_file"
# puts "========================================"

# exit 0
