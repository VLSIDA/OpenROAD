# Clock Mesh Grid Creation Test
# Tests the complete mesh grid creation workflow

puts "========================================="
puts "Clock Mesh Grid Creation Test"
puts "========================================="

# Load database
puts "\nLoading database..."
read_db "/home/wajid/OpenROAD-flow-scripts/flow/results/sky130hd/gcd/base/3_place.odb"

set block [ord::get_db_block]
puts "✓ Loaded: [$block getName]"

# Load liberty
puts "\nLoading liberty..."
read_liberty "/home/wajid/OpenROAD-flow-scripts/flow/platforms/sky130hd/lib/sky130_fd_sc_hd__tt_025C_1v80.lib"
puts "✓ Liberty loaded"

# Load SDC
puts "\nLoading SDC..."
read_sdc "/home/wajid/OpenROAD-flow-scripts/flow/results/sky130hd/gcd/base/3_place.sdc"
puts "✓ SDC loaded"

# Find clock sinks
set clk_name "core_clock"
puts "\n========================================="
puts "Finding Clock Sinks"
puts "========================================="
run_mesh -clock $clk_name

# Create clock mesh
puts "\n========================================="
puts "Creating Clock Mesh Grid"
puts "========================================="
puts "Clock: $clk_name"
puts "H-layer: met4, V-layer: met5"
puts "Width: 0.48 um, Pitch: 5.0 um"

create_clock_mesh \
    -clock $clk_name \
    -h_layer met4 \
    -v_layer met5 \
    -wire_width 0.48 \
    -pitch 5.0

# Save output
set script_dir [file dirname [file normalize [info script]]]
set output_db "${script_dir}/mesh_grid_output.odb"
puts "\nSaving output to: $output_db"
write_db $output_db

puts "\n========================================="
puts "Test Complete!"
puts "========================================="

exit
