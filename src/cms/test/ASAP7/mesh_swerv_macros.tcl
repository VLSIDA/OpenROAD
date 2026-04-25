# Full mesh flow on swerv_wrapper — a real CPU with multiple fakeram macros.
# Bigger than aes-block, exercises the blockage path on a non-trivial design.

source "../helpers.tcl"

set orfs /home/wajid/OpenROAD-flow-scripts/flow
set results $orfs/results/asap7/swerv_wrapper/base
set plat $orfs/platforms/asap7

read_db $results/3_3_place_gp.odb

# Standard cells
read_liberty $plat/lib/NLDM/asap7sc7p5t_AO_RVT_TT_nldm_211120.lib.gz
read_liberty $plat/lib/NLDM/asap7sc7p5t_INVBUF_RVT_TT_nldm_220122.lib.gz
read_liberty $plat/lib/NLDM/asap7sc7p5t_OA_RVT_TT_nldm_211120.lib.gz
read_liberty $plat/lib/NLDM/asap7sc7p5t_SIMPLE_RVT_TT_nldm_211120.lib.gz
read_liberty $plat/lib/NLDM/asap7sc7p5t_SEQ_RVT_TT_nldm_220123.lib

# swerv-specific fakeram libs
foreach lib [glob $orfs/designs/asap7/swerv_wrapper/lib/*.lib] {
    read_liberty $lib
}

read_sdc $results/3_place.sdc

puts "Legalizing initial placement..."
detailed_placement

source $plat/setRC.tcl

# --- Report real macros in the design ---
set block [ord::get_db_block]
set macro_count 0
foreach inst [$block getInsts] {
    if { [$inst isBlock] } {
        set bb [$inst getBBox]
        puts "Macro: [$inst getName]  master=[[$inst getMaster] getName]  bbox=([$bb xMin],[$bb yMin])-([$bb xMax],[$bb yMax])"
        incr macro_count
    }
}
puts "Total real macros: $macro_count"

# Detect the SDC clock name dynamically (swerv typically uses 'clk' or 'core_clock')
set clk_name ""
foreach c [sta::all_clocks] {
    set clk_name [get_name $c]
    break
}
if { $clk_name eq "" } {
    puts "ERROR: no clock defined in SDC"
    exit 1
}
puts "Using clock: $clk_name"

set buffer_list [list BUFx4_ASAP7_75t_R]

# Pitch scaled to die size — 0.7um is sensible for a tiny test design but
# creates hundreds of thousands of buffers on swerv_wrapper (550x600um).
# 10um gives ~2k buffers instead of ~400k, keeping DRT runtimes reasonable.
create_clock_mesh \
    -clock $clk_name \
    -h_layer M4 \
    -v_layer M5 \
    -pitch 10.0 \
    -buffers $buffer_list \
    -macro_halo 0.5

puts "Legalizing buffer placement..."
detailed_placement -max_displacement 1000

puts "Setting up proxy BTERMs..."
setup_proxy_bterms -clock $clk_name -proxy_layer M6

puts "Setting up sink BTerms..."
connect_sinks_to_mesh -clock $clk_name -proxy_layer M6

puts "Routing..."
set_routing_layers -signal M2-M7 -clock M2-M7

global_route -guide_file [make_result_file "swerv_mesh.guide"] \
    -congestion_iterations 50

detailed_route -output_drc [make_result_file "swerv_mesh_drc.rpt"] \
    -droute_end_iter 1

connect_proxy_bterms_to_mesh -clock $clk_name
estimate_parasitics -global_routing
capture_mesh_arrivals -clock $clk_name
merge_mesh_nets -clock $clk_name
convert_mesh_swire -clock $clk_name

extract_parasitics -ext_model_file $plat/rcx_patterns.rules \
    -cc_model 10 -coupling_threshold 0.1 -max_res 0 -skip_over_cell

set spice_file [make_result_file "swerv_mesh.spice"]
set script_dir [file dirname [file normalize [info script]]]
write_mesh_spice -clock $clk_name -output $spice_file -vdd 0.7 \
    -spice_models [list $script_dir/asap7_buf_ideal.spice]

# --- Verify ---
set mesh_net [$block findNet "clk_mesh"]
if { $mesh_net == "NULL" } {
    # The mesh net inherits the underlying clock-net name, not the SDC clock name
    foreach n [$block getNets] {
        if { [string match "*_mesh" [$n getName]] } { set mesh_net $n; break }
    }
}
if { $mesh_net == "NULL" } {
    puts "ERROR: no *_mesh net found"
    exit 1
}
puts "Mesh net: [$mesh_net getName]"

set macro_rects {}
foreach inst [$block getInsts] {
    if { [$inst isBlock] } {
        set bb [$inst getBBox]
        lappend macro_rects [list [$bb xMin] [$bb yMin] [$bb xMax] [$bb yMax] [$inst getName]]
    }
}

set wire_v 0
foreach sw [$mesh_net getSWires] {
    foreach sb [$sw getWires] {
        if { [$sb isVia] } { continue }
        set wx1 [$sb xMin]; set wy1 [$sb yMin]
        set wx2 [$sb xMax]; set wy2 [$sb yMax]
        foreach r $macro_rects {
            lassign $r mx1 my1 mx2 my2 mname
            if { $wx1 < $mx2 && $wx2 > $mx1 && $wy1 < $my2 && $wy2 > $my1 } {
                puts "VIOLATION wire: ($wx1,$wy1)-($wx2,$wy2) overlaps $mname"
                incr wire_v
            }
        }
    }
}

set buf_v 0
foreach inst [$block getInsts] {
    if { ! [string match "mesh_buf_*" [$inst getName]] } { continue }
    set o [$inst getOrigin]
    set bx [lindex $o 0]; set by [lindex $o 1]
    foreach r $macro_rects {
        lassign $r mx1 my1 mx2 my2 mname
        if { $bx >= $mx1 && $bx <= $mx2 && $by >= $my1 && $by <= $my2 } {
            puts "VIOLATION buf: [$inst getName] at ($bx,$by) inside $mname"
            incr buf_v
        }
    }
}

puts ""
puts "===== swerv mesh verification ====="
puts "Real macros:       [llength $macro_rects]"
puts "Wire violations:   $wire_v"
puts "Buffer violations: $buf_v"
puts "===================================="

set odb_file [make_result_file "swerv_mesh.odb"]
puts "Writing ODB: $odb_file"
write_db $odb_file

set def_file [make_result_file "swerv_mesh.def"]
write_def $def_file

puts "View layout with:  openroad -gui $odb_file"

exit [expr {($wire_v == 0 && $buf_v == 0) ? 0 : 1}]
