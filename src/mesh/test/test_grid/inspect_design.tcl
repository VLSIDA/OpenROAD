# Inspect Design Contents
# This script examines what's in your database

puts "========================================="
puts "Design Inspection"
puts "========================================="

set block [ord::get_db_block]

if {$block == "NULL"} {
    puts "ERROR: No block loaded"
    exit 1
}

puts "Block name: [$block getName]"
puts ""

# Count instances by type
set inst_count 0
set master_types {}

puts "Sampling first 20 instances:"
puts "-------------------------------------------"
set sample_count 0
foreach inst [$block getInsts] {
    incr inst_count
    set master [$inst getMaster]
    if {$master != "NULL"} {
        set master_name [$master getName]
        if {[lsearch $master_types $master_name] == -1} {
            lappend master_types $master_name
        }

        if {$sample_count < 20} {
            puts "Instance: [$inst getName], Master: $master_name"
            incr sample_count
        }
    }
}

puts "\n========================================="
puts "Total instances: $inst_count"
puts "Unique master cell types: [llength $master_types]"
puts ""

# Show all unique master types
puts "All master cell types found:"
puts "-------------------------------------------"
foreach master [lsort $master_types] {
    puts "  - $master"
}

# Look for nets
puts "\n========================================="
puts "Examining Nets"
puts "========================================="
set nets [$block getNets]
set net_count [llength $nets]
puts "Total nets: $net_count"
puts ""

puts "First 30 nets:"
puts "-------------------------------------------"
set count 0
foreach net [lrange $nets 0 29] {
    set net_name [$net getName]
    puts "  [expr $count + 1]. $net_name"
    incr count
}

# Look specifically for clock-like nets
puts "\nNets containing 'clk', 'clock', or 'CLK':"
puts "-------------------------------------------"
set clk_net_count 0
foreach net $nets {
    set net_name [$net getName]
    if {[regexp -nocase {clk|clock} $net_name]} {
        puts "  - $net_name"
        incr clk_net_count

        # Show connections for the first clock net
        if {$clk_net_count == 1} {
            set iterms [$net getITerms]
            puts "    -> Connects to [llength $iterms] pins"
        }
    }
}

if {$clk_net_count == 0} {
    puts "  (No clock nets found)"
}

puts "\n========================================="
puts "Inspection Complete!"
puts "========================================="
