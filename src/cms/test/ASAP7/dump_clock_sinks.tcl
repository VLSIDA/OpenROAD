# Dump clock-net sink (x_dbu, y_dbu) for the post-mesh ODB.
#
# Usage (paths passed via env vars; OpenROAD's CLI doesn't forward argv):
#   ODB=design.odb NET=clk_i_mesh OUT=sinks.csv \
#       openroad -no_init -exit dump_clock_sinks.tcl
#
# Picks the average of the iterm's bbox as the sink location.

set odb_path  [expr {[info exists ::env(ODB)] ? $::env(ODB) : ""}]
set net_name  [expr {[info exists ::env(NET)] ? $::env(NET) : ""}]
set out_csv   [expr {[info exists ::env(OUT)] ? $::env(OUT) : ""}]
if { $odb_path eq "" || $net_name eq "" || $out_csv eq "" } {
    puts "usage: ODB=<odb> NET=<net> OUT=<csv> openroad -no_init -exit dump_clock_sinks.tcl"
    exit 1
}

read_db $odb_path
set block [ord::get_db_block]

set net [$block findNet $net_name]
if { $net == "NULL" } {
    # Fallback: pick any *_mesh net (post-merge) or the largest fanout net.
    foreach n [$block getNets] {
        if { [string match "*_mesh" [$n getName]] } { set net $n; break }
    }
}
if { $net == "NULL" } {
    puts "ERROR: net $net_name not found"
    exit 1
}
puts "Dumping sinks for net [$net getName]"

# Helper: dump CLOCK-type input iterms on a given net.
proc dump_clock_inputs {fh net kind} {
    set n 0
    foreach it [$net getITerms] {
        set mt [$it getMTerm]
        if { $mt == "NULL" } { continue }
        if { [$it getIoType] ne "INPUT" } { continue }
        if { [$mt getSigType] ne "CLOCK" } { continue }
        set inst [$it getInst]
        set name [$inst getName]
        if { [string match "mesh_buf_*" $name] } { continue }
        set bbox [$it getBBox]
        set x [expr {([$bbox xMin] + [$bbox xMax]) / 2}]
        set y [expr {([$bbox yMin] + [$bbox yMax]) / 2}]
        puts $fh "$name,$x,$y,$kind"
        incr n
    }
    return $n
}

set fh [open $out_csv w]
puts $fh "name,x_dbu,y_dbu,kind"

# 1) Mesh sinks: every CLOCK input directly on the mesh net.
set n_mesh [dump_clock_inputs $fh $net "mesh"]

# 2) Gated sinks: walk every cell whose CLOCK input is on the mesh, follow
#    its OUTPUT nets, recurse. This captures clock-gating ANDs/ICGs and any
#    chain of them. SIGNAL outputs whose sinks include CLOCK-typed inputs
#    are treated as gated clocks.
set n_gated 0
set visited [dict create [$net getName] 1]
set work [list]
foreach it [$net getITerms] {
    set mt [$it getMTerm]
    if { $mt == "NULL" } { continue }
    if { [$it getIoType] ne "INPUT" } { continue }
    if { [$mt getSigType] ne "CLOCK" } { continue }
    set inst [$it getInst]
    if { [string match "mesh_buf_*" [$inst getName]] } { continue }
    # If this instance has a SIGNAL output, follow it.
    foreach oit [$inst getITerms] {
        if { [$oit getIoType] ne "OUTPUT" } { continue }
        set onet [$oit getNet]
        if { $onet == "NULL" } { continue }
        lappend work $onet
    }
}
while { [llength $work] > 0 } {
    set onet [lindex $work 0]
    set work [lrange $work 1 end]
    set nm [$onet getName]
    if { [dict exists $visited $nm] } { continue }
    dict set visited $nm 1
    set added [dump_clock_inputs $fh $onet "gated"]
    incr n_gated $added
    if { $added == 0 } { continue }
    # Recurse: gated clock could itself feed another ICG.
    foreach it [$onet getITerms] {
        set mt [$it getMTerm]
        if { $mt == "NULL" } { continue }
        if { [$it getIoType] ne "INPUT" } { continue }
        if { [$mt getSigType] ne "CLOCK" } { continue }
        set inst [$it getInst]
        foreach oit [$inst getITerms] {
            if { [$oit getIoType] ne "OUTPUT" } { continue }
            set on2 [$oit getNet]
            if { $on2 == "NULL" } { continue }
            if { ![dict exists $visited [$on2 getName]] } {
                lappend work $on2
            }
        }
    }
}
close $fh
puts "Wrote $n_mesh mesh sinks + $n_gated gated sinks = [expr {$n_mesh + $n_gated}] total to $out_csv"
exit 0
