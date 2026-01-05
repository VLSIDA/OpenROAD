# Test script for clock mesh grid creation - Sky130HD
puts "========================================="
puts "Clock Mesh Grid Creation Test (Sky130HD)"
puts "========================================="

puts "\nLoading database..."
read_db "/home/wajid/OpenROAD-flow-scripts/flow/results/sky130hd/gcd/base/3_place.odb"

set block [ord::get_db_block]
puts "✓ Loaded: [$block getName]"

puts "\nLoading liberty..."
read_liberty "/home/wajid/OpenROAD-flow-scripts/flow/platforms/sky130hd/lib/sky130_fd_sc_hd__tt_025C_1v80.lib"
puts "✓ Liberty loaded"

puts "\nLoading SDC..."
read_sdc "/home/wajid/OpenROAD-flow-scripts/flow/results/sky130hd/gcd/base/3_place.sdc"
puts "✓ SDC loaded"

set clk_name "core_clock"
puts "\n========================================="
puts "Finding Clock Sinks"
puts "========================================="
run_mesh -clock $clk_name

puts "\n========================================="
puts "Creating Clock Mesh Grid"
puts "========================================="
puts "Clock: $clk_name"
puts "H-layer: met4, V-layer: met5"
puts "Width: 0.48 um, Pitch: 5.0 um"

create_clock_mesh \
    -clock $clk_name \
    -h_layer met4 \
    -v_layer met4 \
    -wire_width 0.1 \
    -pitch 2.0

set script_dir [file dirname [file normalize [info script]]]
set output_db "${script_dir}/mesh_grid_output.odb"
puts "\nSaving output to: $output_db"
write_db $output_db

puts "\n========================================="
puts "Test Complete!"
puts "========================================="

exit
