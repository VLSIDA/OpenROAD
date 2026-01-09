# Extract MESH sink locations by running MESH
# MESH will log "MESH_SINK_DATA: <name> <x> <y> <net>" for each sink

puts "========================================"
puts "MESH Sink Extraction"
puts "========================================"

# Load database
puts "\nLoading database..."
read_db "/home/wajid/OPenroad/OpenROAD-flow-scripts/flow/results/sky130hd/gcd/base/3_place.odb"

# Load liberty
puts "Loading liberty..."
read_liberty "/home/wajid/OPenroad/OpenROAD-flow-scripts/flow/platforms/sky130hd/lib/sky130_fd_sc_hd__tt_025C_1v80.lib"

# Load SDC
puts "Loading SDC..."
read_sdc "/home/wajid/OPenroad/OpenROAD-flow-scripts/flow/results/sky130hd/gcd/base/3_place.sdc"

# Get clock name
set clk_name [sta::get_name [lindex [sta::all_clocks] 0]]
puts "\nUsing clock: $clk_name"

# Run MESH - it will log sink data internally
puts "\n========================================"
puts "Running MESH..."
puts "========================================"

run_mesh -clock $clk_name

puts "\n========================================"
puts "MESH completed"
puts "========================================"

exit
