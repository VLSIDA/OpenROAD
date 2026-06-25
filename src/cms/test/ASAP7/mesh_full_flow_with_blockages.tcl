# Full end-to-end mesh flow with blockages — mirrors mesh_cts_integration_asap7.tcl
# but injects blockages and one real macro, then runs the complete flow all the way
# through routing + extraction + SPICE + ODB/DEF so you can open the final layout in GUI.

source "../helpers.tcl"

read_db "3_3_place_gp.odb"

read_liberty "asap7/lib/NLDM/asap7sc7p5t_AO_RVT_TT_nldm_211120.lib.gz"
read_liberty "asap7/lib/NLDM/asap7sc7p5t_INVBUF_RVT_TT_nldm_220122.lib.gz"
read_liberty "asap7/lib/NLDM/asap7sc7p5t_OA_RVT_TT_nldm_211120.lib.gz"
read_liberty "asap7/lib/NLDM/asap7sc7p5t_SIMPLE_RVT_TT_nldm_211120.lib.gz"
read_liberty "asap7/lib/NLDM/asap7sc7p5t_SEQ_RVT_TT_nldm_220123.lib"

read_sdc "3_place.sdc"

puts "Legalizing initial placement..."
detailed_placement

source "asap7/setRC.tcl"

# --- Inject three dbBlockages sized for this small design ---
set block [ord::get_db_block]
set core [$block getCoreArea]
set cx1 [$core xMin]; set cy1 [$core yMin]
set cx2 [$core xMax]; set cy2 [$core yMax]
set w [expr {$cx2 - $cx1}]; set h [expr {$cy2 - $cy1}]

# Top-left
odb::dbBlockage_create $block \
    [expr {$cx1 + $w*15/100}] [expr {$cy1 + $h*65/100}] \
    [expr {$cx1 + $w*30/100}] [expr {$cy1 + $h*85/100}]
puts "Injected blockage 1 (top-left)"

# Bottom-right
odb::dbBlockage_create $block \
    [expr {$cx1 + $w*65/100}] [expr {$cy1 + $h*15/100}] \
    [expr {$cx1 + $w*80/100}] [expr {$cy1 + $h*30/100}]
puts "Injected blockage 2 (bottom-right)"

# Center (a bigger one — simulates a real macro without the pin-routing issue)
odb::dbBlockage_create $block \
    [expr {$cx1 + $w*40/100}] [expr {$cy1 + $h*40/100}] \
    [expr {$cx1 + $w*60/100}] [expr {$cy1 + $h*60/100}]
puts "Injected blockage 3 (center)"

set clk_name core_clock
set buffer_list [list BUFx4_ASAP7_75t_R]

# --- Build clock mesh avoiding all blockages (0.5um halo) ---
create_clock_mesh \
    -clock $clk_name \
    -h_layer M4 \
    -v_layer M5 \
    -pitch 0.7 \
    -buffers $buffer_list \
    -macro_halo 0.5

puts "Legalizing buffer placement..."
detailed_placement -max_displacement 1000

puts "Setting up proxy BTERMs for buffer connections..."
setup_proxy_bterms -clock $clk_name -proxy_layer M6

puts "Setting up sink BTerms for router-based connections..."
connect_sinks_to_mesh -clock $clk_name -proxy_layer M6

puts "Routing the design..."
set_routing_layers -signal M2-M7 -clock M2-M7

global_route -guide_file [make_result_file "blockage_flow.guide"] \
    -congestion_iterations 50

detailed_route -output_drc [make_result_file "blockage_flow_drc.rpt"] \
    -droute_end_iter 1

puts "Connecting proxy BTERMs to mesh..."
connect_proxy_bterms_to_mesh -clock $clk_name

puts "Estimating parasitics for CTS arrival analysis..."
estimate_parasitics -global_routing

puts "Capturing CTS leaf arrival times..."
capture_mesh_arrivals -clock $clk_name

puts "Merging buffer/sink nets into mesh net..."
merge_mesh_nets -clock $clk_name

puts "Converting mesh SWires to regular wires..."
convert_mesh_swire -clock $clk_name

puts "Extracting parasitics..."
extract_parasitics -ext_model_file asap7/rcx_patterns.rules \
    -cc_model 10 -coupling_threshold 0.1 -max_res 0 -skip_over_cell

set spice_file [make_result_file "blockage_flow.spice"]
set script_dir [file dirname [file normalize [info script]]]
write_mesh_spice -clock $clk_name -output $spice_file -vdd 0.7 \
    -spice_models [list $script_dir/asap7_buf_ideal.spice]

# --- Verify no mesh geometry lands in any blockage or macro ---
set mesh_net [$block findNet "clk_mesh"]
set blockage_rects {}
foreach blk [$block getBlockages] {
    set bb [$blk getBBox]
    lappend blockage_rects [list [$bb xMin] [$bb yMin] [$bb xMax] [$bb yMax] "blockage"]
}
foreach inst [$block getInsts] {
    if { [$inst isBlock] } {
        set bb [$inst getBBox]
        lappend blockage_rects [list [$bb xMin] [$bb yMin] [$bb xMax] [$bb yMax] [$inst getName]]
    }
}

set wire_v 0
foreach sbox_path [list "SWires" "Wires"] {
    if { $sbox_path eq "SWires" } {
        foreach sw [$mesh_net getSWires] {
            foreach sb [$sw getWires] {
                if { [$sb isVia] } { continue }
                set wx1 [$sb xMin]; set wy1 [$sb yMin]
                set wx2 [$sb xMax]; set wy2 [$sb yMax]
                foreach r $blockage_rects {
                    lassign $r mx1 my1 mx2 my2 mname
                    if { $wx1 < $mx2 && $wx2 > $mx1 && $wy1 < $my2 && $wy2 > $my1 } {
                        puts "VIOLATION wire: ($wx1,$wy1)-($wx2,$wy2) overlaps $mname"
                        incr wire_v
                    }
                }
            }
        }
    }
}

set buf_v 0
foreach inst [$block getInsts] {
    if { ! [string match "mesh_buf_*" [$inst getName]] } { continue }
    set o [$inst getOrigin]
    set bx [lindex $o 0]; set by [lindex $o 1]
    foreach r $blockage_rects {
        lassign $r mx1 my1 mx2 my2 mname
        if { $bx >= $mx1 && $bx <= $mx2 && $by >= $my1 && $by <= $my2 } {
            puts "VIOLATION buf: [$inst getName] at ($bx,$by) inside $mname"
            incr buf_v
        }
    }
}

puts ""
puts "===== Final verification ====="
puts "Blockage rects:    [llength $blockage_rects]"
puts "Wire violations:   $wire_v"
puts "Buffer violations: $buf_v"
puts "=============================="

set odb_file [make_result_file "blockage_flow.odb"]
puts "Writing ODB: $odb_file"
write_db $odb_file

set def_file [make_result_file "blockage_flow.def"]
puts "Writing DEF: $def_file"
write_def $def_file

puts ""
puts "To view the layout:"
puts "  openroad -gui"
puts "  In GUI: File > Read > $odb_file"
puts "  Load liberties + SDC as in this script"
puts ""
puts "Or launch directly:"
puts "  openroad -gui $odb_file"

exit [expr {($wire_v == 0 && $buf_v == 0) ? 0 : 1}]
