# Highlight Sequential Elements and Clock Pins in GUI
# Load this after opening the design in GUI

puts "========================================="
puts "Highlighting Sequential Elements & Clock Pins"
puts "========================================="

set block [ord::get_db_block]

# Find all sequential instances (DFF, DFFE, latch, etc.)
set seq_inst_count 0

foreach inst [$block getInsts] {
    set master [$inst getMaster]
    if {$master == "NULL"} {
        continue
    }

    set master_name [$master getName]

    # Match common sequential cell patterns
    if {[regexp -nocase {dff|dffe|dfxtp|edfxtp|latch|sdff|dfr} $master_name]} {
        # Highlight the sequential instance
        set inst_name [$inst getName]
        gui::highlight_inst $inst_name 1
        incr seq_inst_count
    }
}

puts "Found $seq_inst_count sequential elements"

# Find and highlight clock net
set clk_name "clk"
puts "\nFinding clock net: $clk_name"

set clk_net [$block findNet $clk_name]
if {$clk_net != "NULL"} {
    puts "Highlighting clock net: $clk_name"
    gui::highlight_net $clk_name 3
    puts "Clock net highlighted"

    # Show clock net connectivity
    set iterms [$clk_net getITerms]
    puts "Clock net connects to [llength $iterms] instance pins"
} else {
    puts "Clock net '$clk_name' not found"
    puts "Available clock nets:"
    set nets [$block getNets]
    set count 0
    foreach net [lrange $nets 0 10] {
        set net_name [$net getName]
        if {[regexp -nocase {clk|clock} $net_name]} {
            puts "  - $net_name"
            incr count
        }
    }
    if {$count == 0} {
        puts "  (No nets matching 'clk' or 'clock' pattern found)"
    }
}

puts "\n========================================="
puts "Highlighting Complete!"
puts "========================================="
puts "Group 1 = Sequential elements (flip-flops with clock pins)"
puts "Group 3 = Clock net"
puts ""
puts "Use GUI menu: View -> Clear Highlights to clear"
puts "========================================="
