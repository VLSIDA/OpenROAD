# Quick audit: how many clock-input pins exist in the design, and how
# many are on each clock-ish net? Helps explain a sink-count mismatch.

set odb_path [expr {[info exists ::env(ODB)] ? $::env(ODB) : ""}]
if { $odb_path eq "" } {
    puts "usage: ODB=<odb> openroad -no_init -exit audit_clock_sinks.tcl"
    exit 1
}

read_db $odb_path
set block [ord::get_db_block]

# All CLOCK-typed input iterms across all sequentials.
set total_clk_pins 0
foreach inst [$block getInsts] {
    foreach it [$inst getITerms] {
        set mt [$it getMTerm]
        if { $mt == "NULL" } { continue }
        if { [$mt getSigType] eq "CLOCK" && [$it getIoType] eq "INPUT" } {
            incr total_clk_pins
        }
    }
}
puts "Total CLOCK-input pins in design: $total_clk_pins"

# Per-net counts for anything clock-related.
foreach n [$block getNets] {
    set nm [$n getName]
    if { ![string match "*clk*" $nm] && ![string match "*_mesh" $nm] } {
        continue
    }
    set n_clk 0
    set n_in 0
    foreach it [$n getITerms] {
        set mt [$it getMTerm]
        if { $mt == "NULL" } { continue }
        if { [$it getIoType] eq "INPUT" } {
            incr n_in
            if { [$mt getSigType] eq "CLOCK" } { incr n_clk }
        }
    }
    if { $n_in == 0 } { continue }
    puts [format "  net %-40s  input_iterms=%4d  of_which_clock=%4d" \
              $nm $n_in $n_clk]
}
exit 0
