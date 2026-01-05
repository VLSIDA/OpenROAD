# Test script for clock mesh grid creation - ASAP7
puts "========================================="
puts "Clock Mesh Grid Creation Test (ASAP7)"
puts "========================================="

puts "\nLoading database..."
read_db "/home/wajid/OpenROAD-flow-scripts/flow/results/asap7/gcd/base/3_place.odb"

set block [ord::get_db_block]
puts "✓ Loaded: [$block getName]"

puts "\nLoading liberty libraries..."
# Load all ASAP7 liberty files
read_liberty "/home/wajid/OpenROAD-flow-scripts/flow/platforms/asap7/lib/NLDM/asap7sc7p5t_AO_RVT_FF_nldm_211120.lib.gz"
read_liberty "/home/wajid/OpenROAD-flow-scripts/flow/platforms/asap7/lib/NLDM/asap7sc7p5t_INVBUF_RVT_FF_nldm_220122.lib.gz"
read_liberty "/home/wajid/OpenROAD-flow-scripts/flow/platforms/asap7/lib/NLDM/asap7sc7p5t_OA_RVT_FF_nldm_211120.lib.gz"
read_liberty "/home/wajid/OpenROAD-flow-scripts/flow/platforms/asap7/lib/NLDM/asap7sc7p5t_SIMPLE_RVT_FF_nldm_211120.lib.gz"
read_liberty "/home/wajid/OpenROAD-flow-scripts/flow/platforms/asap7/lib/NLDM/asap7sc7p5t_SEQ_RVT_FF_nldm_220123.lib"
puts "✓ Liberty libraries loaded"

puts "\nLoading SDC..."
read_sdc "/home/wajid/OpenROAD-flow-scripts/flow/results/asap7/gcd/base/3_place.sdc"
puts "✓ SDC loaded"

# For ASAP7, the clock name in SDC is "core_clock"
set clk_name "core_clock"
puts "\n========================================="
puts "Finding Clock Sinks"
puts "========================================="
run_mesh -clock $clk_name

puts "\n========================================="
puts "Creating Clock Mesh Grid"
puts "========================================="
puts "Clock: $clk_name"
puts "H-layer: M7, V-layer: M7"
puts "Note: Testing with both layers on M7 (highest routing layer)"

create_clock_mesh \
    -clock $clk_name \
    -h_layer M7 \
    -v_layer M7 \
    -wire_width 0.05 \
    -pitch 1.0

set script_dir [file dirname [file normalize [info script]]]
set output_db "${script_dir}/mesh_grid_output_asap7.odb"
puts "\nSaving output to: $output_db"
write_db $output_db

puts "\n========================================="
puts "Test Complete!"
puts "========================================="

exit
