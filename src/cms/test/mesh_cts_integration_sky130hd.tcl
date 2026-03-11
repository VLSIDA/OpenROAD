source "helpers.tcl"
read_db "3_3_place_gp.odb"
read_liberty "sky130hd/lib/sky130_fd_sc_hd__tt_025C_1v80.lib"
read_sdc "3_place.sdc"

set clk_name "core_clock"
set buffer_list [list sky130_fd_sc_hd__clkbuf_1]

create_clock_mesh \
    -clock $clk_name \
    -h_layer met2 \
    -v_layer met3 \
    -pitch 1.0 \
    -buffers $buffer_list

puts "Legalizing buffer placement..."
detailed_placement -max_displacement 1000
puts "Buffer placement legalized"

# Setup proxy BTERMs for buffer outputs first (so sinks can share them)
puts "Setting up proxy BTERMs on met4 for buffer connections..."
setup_proxy_bterms -clock $clk_name -proxy_layer met4

# Connect sinks via router (places BTerms at grid intersections)
# Sinks that map to buffer intersections will share the buffer's BTerm
puts "Setting up sink BTerms for router-based connections..."
connect_sinks_to_mesh -clock $clk_name -proxy_layer met4
puts "Routing the design..."
set_routing_layers -signal met1-met4 -clock met1-met4
set_wire_rc -signal -layer met2
set_wire_rc -clock -layer met2

global_route -guide_file [make_result_file "mesh_cts_route.guide"] \
    -congestion_iterations 50

detailed_route -output_drc [make_result_file "mesh_cts_drc.rpt"] \
    -droute_end_iter 1

# After routing, connect the proxy BTERMs to the mesh via via stacks
puts "Connecting proxy BTERMs to mesh..."
connect_proxy_bterms_to_mesh -clock $clk_name

# Capture CTS leaf arrival times from STA (must be before merge)
puts "Capturing CTS leaf arrival times for skew analysis..."
capture_mesh_arrivals -clock $clk_name

# Merge buffer and sink nets into clk_mesh for parasitic extraction
puts "Merging buffer and sink nets into mesh net..."
merge_mesh_nets -clock $clk_name

# Convert SWires to regular wires for OpenRCX extraction
puts "Converting mesh SWires to regular wires..."
convert_mesh_swire -clock $clk_name

# Extract parasitics using OpenRCX (model-based extraction with coupling caps)
puts "Setting up via resistances..."
set_layer_rc -via mcon -resistance 9.249146E-3
set_layer_rc -via via  -resistance 4.5E-3
set_layer_rc -via via2 -resistance 3.368786E-3
set_layer_rc -via via3 -resistance 0.376635E-3
set_layer_rc -via via4 -resistance 0.00580E-3

puts "Extracting parasitics with coupling capacitance..."
extract_parasitics -ext_model_file ./sky130hd/rcx_patterns.rules \
    -cc_model 10 \
    -coupling_threshold 0.1 \
    -max_res 0 \
    -skip_over_cell

# Write SPICE netlist from extracted parasitics
# Include sky130 transistor models (TT corner) and cell subcircuit definitions
set sky130_pdk "$::env(HOME)/.ciel/ciel/sky130/versions/54435919abffb937387ec956209f9cf5fd2dfbee/sky130A"
set spice_models [list \
    "${sky130_pdk}/libs.tech/ngspice/corners/tt.spice" \
    "${sky130_pdk}/libs.ref/sky130_fd_sc_hd/spice/sky130_fd_sc_hd.spice"]

set spice_file [make_result_file "mesh_cts_integration_sky130hd.spice"]
puts "Writing SPICE netlist to: $spice_file"
write_mesh_spice -clock $clk_name -output $spice_file -spice_models $spice_models

# Verify merge: check that clk_mesh has all connections
set block [ord::get_db_block]
set mesh_net_name "clk_mesh"
set mesh_net [$block findNet $mesh_net_name]
if { $mesh_net == "NULL" } {
    puts "ERROR: Mesh net '$mesh_net_name' not found after merge!"
} else {
    set iterms [$mesh_net getITerms]
    puts "Mesh net '$mesh_net_name' has [llength $iterms] ITerms after merge"

    # Check that no buffer/sink nets remain
    set remaining_buf 0
    set remaining_sink 0
    foreach net [$block getNets] {
        set name [$net getName]
        if { [string match "${clk_name}_buf_*" $name] } {
            incr remaining_buf
        }
        if { [string match "sink_*" $name] && ![string match "*bterm*" $name] } {
            incr remaining_sink
        }
    }
    puts "Remaining buffer nets: $remaining_buf (expected 0)"
    puts "Remaining sink nets: $remaining_sink (expected 0)"
}

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

# Write Verilog netlist (after merge, write_verilog is sufficient)
set verilog_file [make_result_file "mesh_cts_integration_sky130hd.v"]
puts "Writing Verilog netlist..."
write_verilog -include_pwr_gnd $verilog_file
puts "Verilog netlist: $verilog_file"


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

 exit 0
