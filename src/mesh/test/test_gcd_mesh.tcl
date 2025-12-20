# Test mesh tool with GCD design after placement

# Path to ORFS results
set orfs_base "$::env(HOME)/OPenroad/OpenROAD-flow-scripts/flow"
set design_path "${orfs_base}/results/asap7/gcd/base"
set platform_path "${orfs_base}/platforms/asap7"

# Load the placed database
puts "Loading database..."
read_db "${design_path}/3_place.odb"

# Verify database loaded
set db [ord::get_db]
set block [ord::get_db_block]
if { $block == "NULL" } {
    puts "ERROR: Block is NULL after reading database!"
    puts "Database object: $db"
    exit 1
}
puts "✓ Database loaded successfully"
puts "  Block name: [$block getName]"
puts "  Design has [llength [$block getInsts]] instances"

# Load liberty files
puts "\nLoading liberty files..."
# ASAP7 uses CCS libraries by default, FF corner for timing
# Only load the libraries needed for clock sink identification
set lib_files [list \
    "${platform_path}/lib/CCS/asap7sc7p5t_SEQ_RVT_FF_ccs_220123.lib" \
    "${platform_path}/lib/CCS/asap7sc7p5t_INVBUF_RVT_FF_ccs_220122.lib.gz" \
    "${platform_path}/lib/CCS/asap7sc7p5t_SIMPLE_RVT_FF_ccs_211120.lib.gz" \
]

set num_libs 0
foreach lib_file $lib_files {
    if { [file exists $lib_file] } {
        # Quietly read liberty files (suppress read_liberty output)
        read_liberty $lib_file
        incr num_libs
    }
}
puts "✓ Loaded $num_libs liberty files"

# Load SDC constraints (contains clock definitions)
puts "Loading SDC constraints..."
read_sdc "${design_path}/3_place.sdc"

# Test 1: Find all clock sinks
puts "\n=========================================="
puts "TEST 1: Finding all clock sinks"
puts "=========================================="
find_clock_sinks

# Test 2: Print summary
puts "\n=========================================="
puts "TEST 2: Print clock sink summary"
puts "=========================================="
print_clock_sinks

# Test 3: Get statistics
puts "\n=========================================="
puts "TEST 3: Clock statistics"
puts "=========================================="
set num_clocks [get_num_clocks]
puts "Total number of clocks: $num_clocks"

# Get all clock names and their sink counts
if { $num_clocks > 0 } {
    # Get all clocks using STA
    set clocks [sta::all_clocks]
    foreach clk $clocks {
        set clk_name [sta::get_name $clk]
        set num_sinks [get_num_sinks $clk_name]
        puts "  Clock '$clk_name': $num_sinks sinks"

        # Test individual clock
        puts "\n  Running mesh for clock: $clk_name"
        mesh -name $clk_name
    }
} else {
    puts "WARNING: No clocks found in design!"
}

puts "\n=========================================="
puts "MESH TOOL TEST COMPLETE"
puts "=========================================="
