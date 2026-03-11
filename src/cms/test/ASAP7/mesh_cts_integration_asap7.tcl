source "../helpers.tcl"
read_db "3_3_place_gp.odb"

# Load ASAP7 liberty files (TT corner, RVT)
read_liberty "asap7/lib/NLDM/asap7sc7p5t_AO_RVT_TT_nldm_211120.lib.gz"
read_liberty "asap7/lib/NLDM/asap7sc7p5t_INVBUF_RVT_TT_nldm_220122.lib.gz"
read_liberty "asap7/lib/NLDM/asap7sc7p5t_OA_RVT_TT_nldm_211120.lib.gz"
read_liberty "asap7/lib/NLDM/asap7sc7p5t_SIMPLE_RVT_TT_nldm_211120.lib.gz"
read_liberty "asap7/lib/NLDM/asap7sc7p5t_SEQ_RVT_TT_nldm_220123.lib"

read_sdc "3_place.sdc"

# Legalize initial placement before mesh creation
puts "Legalizing initial placement..."
detailed_placement

# Set wire RC before CTS (CTS needs these for proper clock tree optimization)
source "asap7/setRC.tcl"

set clk_name "core_clock"
set buffer_list [list BUFx4_ASAP7_75t_R]

# Create clock mesh on M4 (horizontal) and M5 (vertical)
create_clock_mesh \
    -clock $clk_name \
    -h_layer M4 \
    -v_layer M5 \
    -pitch 0.7 \
    -buffers $buffer_list

# Legalize buffer placement
puts "Legalizing buffer placement..."
detailed_placement -max_displacement 1000
puts "Buffer placement legalized"

# Setup proxy BTERMs for buffer outputs on mesh layer
puts "Setting up proxy BTERMs for buffer connections..."
setup_proxy_bterms -clock $clk_name -proxy_layer M6

# Connect sinks to mesh grid via router
puts "Setting up sink BTerms for router-based connections..."
connect_sinks_to_mesh -clock $clk_name -proxy_layer M6

# Route the design
puts "Routing the design..."
set_routing_layers -signal M2-M7 -clock M2-M7

global_route -guide_file [make_result_file "asap7_mesh_route.guide"] \
    -congestion_iterations 50

detailed_route -output_drc [make_result_file "asap7_mesh_drc.rpt"] \
    -droute_end_iter 1

# Connect proxy BTERMs to mesh
puts "Connecting proxy BTERMs to mesh..."
connect_proxy_bterms_to_mesh -clock $clk_name

# Capture CTS leaf arrival times from STA (before merge)
puts "Capturing CTS leaf arrival times for skew analysis..."
capture_mesh_arrivals -clock $clk_name

# Merge buffer and sink nets into mesh net
puts "Merging buffer and sink nets into mesh net..."
merge_mesh_nets -clock $clk_name

# Convert SWires to regular wires for extraction
puts "Converting mesh SWires to regular wires..."
convert_mesh_swire -clock $clk_name

# Extract parasitics
puts "Extracting parasitics..."
extract_parasitics -ext_model_file asap7/rcx_patterns.rules \
    -cc_model 10 \
    -coupling_threshold 0.1 \
    -max_res 0 \
    -skip_over_cell

# Write SPICE netlist
set spice_file [make_result_file "mesh_cts_integration_asap7.spice"]
puts "Writing SPICE netlist to: $spice_file"
set script_dir [file dirname [file normalize [info script]]]
write_mesh_spice -clock $clk_name -output $spice_file -vdd 0.7 \
    -spice_models [list $script_dir/asap7_buf_ideal.spice]

# Verify merge
set block [ord::get_db_block]
set mesh_net [$block findNet "clk_mesh"]
if { $mesh_net == "NULL" } {
    puts "ERROR: Mesh net 'clk_mesh' not found after merge!"
} else {
    set iterms [$mesh_net getITerms]
    puts "Mesh net 'clk_mesh' has [llength $iterms] ITerms after merge"
}

# Write outputs
set odb_file [make_result_file "mesh_cts_integration_asap7.odb"]
puts "Writing ODB to: $odb_file"
write_db $odb_file

set def_file [make_result_file "mesh_cts_integration_asap7.def"]
puts "Writing DEF to: $def_file"
write_def $def_file

exit 0
