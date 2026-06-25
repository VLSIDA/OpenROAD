source "../helpers.tcl"
read_db "3_3_place_gp.odb"

# Load ASAP7 liberty files (TT corner, RVT)
read_liberty "asap7/lib/NLDM/asap7sc7p5t_AO_RVT_TT_nldm_211120.lib.gz"
read_liberty "asap7/lib/NLDM/asap7sc7p5t_INVBUF_RVT_TT_nldm_220122.lib.gz"
read_liberty "asap7/lib/NLDM/asap7sc7p5t_OA_RVT_TT_nldm_211120.lib.gz"
read_liberty "asap7/lib/NLDM/asap7sc7p5t_SIMPLE_RVT_TT_nldm_211120.lib.gz"
read_liberty "asap7/lib/NLDM/asap7sc7p5t_SEQ_RVT_TT_nldm_220123.lib"

read_sdc "3_place.sdc"

puts "Legalizing initial placement..."
detailed_placement

source "asap7/setRC.tcl"

set clk_name core_clock
set buffer_list [list BUFx4_ASAP7_75t_R]

# --- Inject a routing blockage in the middle of the core ---
# This simulates a macro for blockage-handling testing without needing
# a macro-bearing design.
set block [ord::get_db_block]
set core [$block getCoreArea]
set cx1 [$core xMin]; set cy1 [$core yMin]
set cx2 [$core xMax]; set cy2 [$core yMax]
set core_w [expr {$cx2 - $cx1}]
set core_h [expr {$cy2 - $cy1}]

# 20% x 20% blockage centered in core (plenty of rails run through it)zzzz
set bx1 [expr {$cx1 + $core_w * 4 / 10}]
set by1 [expr {$cy1 + $core_h * 4 / 10}]
set bx2 [expr {$cx1 + $core_w * 6 / 10}]
set by2 [expr {$cy1 + $core_h * 6 / 10}]

puts "Core: ($cx1,$cy1)-($cx2,$cy2)"
puts "Injected blockage: ($bx1,$by1)-($bx2,$by2)"

set blk [odb::dbBlockage_create $block $bx1 $by1 $bx2 $by2]

# Collect all blockage rects + macro rects for later verification
set blockage_rects [list [list $bx1 $by1 $bx2 $by2 "injected"]]
foreach inst [$block getInsts] {
    if { [$inst isBlock] } {
        set bb [$inst getBBox]
        lappend blockage_rects [list [$bb xMin] [$bb yMin] [$bb xMax] [$bb yMax] [$inst getName]]
    }
}
puts "Total blockage rects to check against: [llength $blockage_rects]"

# Create clock mesh with 0.5um halo
create_clock_mesh \
    -clock $clk_name \
    -h_layer M4 \
    -v_layer M5 \
    -pitch 0.7 \
    -buffers $buffer_list \
    -macro_halo 0.5

# --- Verify: no mesh SWire sits inside any blockage bbox ---
# The SDC clock name (core_clock) binds to a net called 'clk' in this design,
# so the mesh net is 'clk_mesh' — same as the existing working test.
set mesh_net [$block findNet "clk_mesh"]
if { $mesh_net == "NULL" } {
    puts "ERROR: mesh net 'clk_mesh' not found"
    exit 1
}

set wire_violations 0
foreach swire [$mesh_net getSWires] {
    foreach sbox [$swire getWires] {
        if { [$sbox isVia] } { continue }
        set wx1 [$sbox xMin]; set wy1 [$sbox yMin]
        set wx2 [$sbox xMax]; set wy2 [$sbox yMax]
        foreach r $blockage_rects {
            lassign $r mx1 my1 mx2 my2 mname
            if { $wx1 < $mx2 && $wx2 > $mx1 && $wy1 < $my2 && $wy2 > $my1 } {
                puts "VIOLATION wire: ($wx1,$wy1)-($wx2,$wy2) overlaps $mname"
                incr wire_violations
            }
        }
    }
}

# --- Verify: no buffer inst placed inside a blockage ---
set buf_violations 0
set buf_count 0
foreach inst [$block getInsts] {
    set iname [$inst getName]
    if { ! [string match "mesh_buf_*" $iname] } { continue }
    incr buf_count
    set origin [$inst getOrigin]
    set bx [lindex $origin 0]
    set by [lindex $origin 1]
    foreach r $blockage_rects {
        lassign $r mx1 my1 mx2 my2 mname
        if { $bx >= $mx1 && $bx <= $mx2 && $by >= $my1 && $by <= $my2 } {
            puts "VIOLATION buffer: $iname at ($bx,$by) inside $mname"
            incr buf_violations
        }
    }
}

puts ""
puts "===== Blockage check summary ====="
puts "Buffers placed:    $buf_count"
puts "Wire violations:   $wire_violations"
puts "Buffer violations: $buf_violations"
puts "=================================="

if { $wire_violations > 0 || $buf_violations > 0 } {
    puts "FAIL"
    exit 1
}
puts "PASS"
exit 0
