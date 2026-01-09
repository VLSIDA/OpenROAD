# Run CTS with sink logging enabled
# CTS will log "CTS_SINK_DATA: <name> <x> <y> <net>" for each sink

puts "========================================"
puts "Running CTS with Sink Logging"
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

# Run CTS on clock net "clk"
puts "\n========================================"
puts "Running CTS..."
puts "========================================"

clock_tree_synthesis -clk_nets clk

puts "\n========================================"
puts "CTS completed"
puts "========================================"

exit
