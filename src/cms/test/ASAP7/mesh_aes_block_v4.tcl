# mesh_aes_block_v4.tcl
# 8x8 clock mesh on the aes-block top design.
# Uses -rows 8 -cols 8 instead of a fixed pitch; the C++ layer computes
# H-pitch and V-pitch from the sink bounding box automatically.

source "../helpers.tcl"

set orfs /home/wali2/OpenROAD-flow-scripts/flow
set results $orfs/results/asap7/aes-block/base
set plat $orfs/platforms/asap7

read_db $results/3_3_place_gp.odb

foreach blk {aes-block_aes_rcon aes-block_aes_sbox} {
    set base $orfs/results/asap7/$blk/base
    set nick [string range $blk 10 end]
    read_liberty $base/${nick}_typ.lib
}

read_liberty $plat/lib/NLDM/asap7sc7p5t_AO_RVT_TT_nldm_211120.lib.gz
read_liberty $plat/lib/NLDM/asap7sc7p5t_INVBUF_RVT_TT_nldm_220122.lib.gz
read_liberty $plat/lib/NLDM/asap7sc7p5t_OA_RVT_TT_nldm_211120.lib.gz
read_liberty $plat/lib/NLDM/asap7sc7p5t_SIMPLE_RVT_TT_nldm_211120.lib.gz
read_liberty $plat/lib/NLDM/asap7sc7p5t_SEQ_RVT_TT_nldm_220123.lib

read_sdc $results/3_place.sdc

puts "Legalizing initial placement..."
detailed_placement

source $plat/setRC.tcl

# --- Report macros ---
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

set clk_name clk
set buffer_list [list BUFx4_ASAP7_75t_R]

# Compute pitch from core area to produce an 8x8 grid.
# pitch = core_dimension / (N - 1) where N = 8 lines.
# Blockages only clip wire segments; all 8 H-lines and 8 V-lines
# are still placed, giving up to 64 buffer positions.
set grid_n 8
set core  [$block getCoreArea]
set core_w [ord::dbu_to_microns [expr { [$core xMax] - [$core xMin] }]]
set core_h [ord::dbu_to_microns [expr { [$core yMax] - [$core yMin] }]]
set pitch  [expr { min($core_w, $core_h) / ($grid_n - 1.0) }]

puts [format "Core area    : %.3f x %.3f um" $core_w $core_h]
puts [format "Target grid  : %dx%d" $grid_n $grid_n]
puts [format "Computed pitch: %.4f um  (= min(%.3f,%.3f) / %d)" \
      $pitch $core_w $core_h [expr {$grid_n - 1}]]

create_clock_mesh \
    -clock $clk_name \
    -h_layer M4 \
    -v_layer M5 \
    -pitch $pitch \
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

global_route -guide_file [make_result_file "aes_block_mesh_v4.guide"] \
    -congestion_iterations 50

detailed_route -output_drc [make_result_file "aes_block_mesh_v4_drc.rpt"] \
    -droute_end_iter 64

puts "Connecting proxy BTERMs to mesh..."
connect_proxy_bterms_to_mesh -clock $clk_name

estimate_parasitics -global_routing
capture_mesh_arrivals -clock $clk_name

set_propagated_clock [get_clocks $clk_name]
puts ""
puts "===== Mesh v4 8x8 power (pre-merge) ====="
report_power
puts "========================================="
puts ""

merge_mesh_nets -clock $clk_name
convert_mesh_swire -clock $clk_name

extract_parasitics -ext_model_file $plat/rcx_patterns.rules \
    -cc_model 10 -coupling_threshold 0.1 -max_res 0 -skip_over_cell

# Write SPEF for wire-cap dynamic power analysis
set spef_file [make_result_file "aes_block_mesh_v4.spef"]
puts "Writing SPEF: $spef_file"
write_spef $spef_file

set script_dir [file dirname [file normalize [info script]]]

set spice_file [make_result_file "aes_block_mesh_v4.spice"]
write_mesh_spice -clock $clk_name -output $spice_file -vdd 0.7 \
    -spice_models [list $script_dir/asap7_buf.spice]

set spice_zd [make_result_file "aes_block_mesh_v4_zero_delay.spice"]
write_mesh_spice -clock $clk_name -output $spice_zd -vdd 0.7 \
    -spice_models [list $script_dir/asap7_buf.spice] \
    -zero_delay

# --- Liberty pin cap helper ---
proc get_pin_cap_fF { master_name pin_name } {
    set lib_pins [get_lib_pins "${master_name}/${pin_name}"]
    if { [llength $lib_pins] == 0 } { return 0.0 }
    set cap [get_property [lindex $lib_pins 0] capacitance]
    if { [string is double -strict $cap] } { return $cap }
    return 0.0
}

# --- P1 pin caps: mesh buffer input pins (clknet_* nets) ---
puts "Computing mesh buffer input pin caps..."
set C_buf_pin_fF 0.0
set n_buf_pins   0
foreach net [$block getNets] {
    if { ![string match "clknet_*" [$net getName]] } { continue }
    foreach iterm [$net getITerms] {
        set inst [$iterm getInst]
        if { ![string match "mesh_buf_*" [$inst getName]] } { continue }
        set cap [get_pin_cap_fF \
            [[$inst getMaster] getName] \
            [[$iterm getMTerm] getName]]
        set C_buf_pin_fF [expr { $C_buf_pin_fF + $cap }]
        incr n_buf_pins
    }
}

# --- P2 pin caps: FF sink clock pins (clk_mesh net) ---
puts "Computing FF sink clock pin caps..."
set C_sink_pin_fF 0.0
set n_sinks       0
set mesh_net_pow [$block findNet "clk_mesh"]
foreach iterm [$mesh_net_pow getITerms] {
    set inst [$iterm getInst]
    if { [string match "mesh_buf_*" [$inst getName]] } { continue }
    set cap [get_pin_cap_fF \
        [[$inst getMaster] getName] \
        [[$iterm getMTerm] getName]]
    set C_sink_pin_fF [expr { $C_sink_pin_fF + $cap }]
    incr n_sinks
}

set pin_file [make_result_file "aes_block_mesh_v4_pin_caps.txt"]
set f [open $pin_file w]
puts $f "# Liberty pin caps (femtofarads, ASAP7 capacitive_load_unit = 1 fF)"
puts $f "BUF_PIN_CAP_fF  $C_buf_pin_fF"
puts $f "BUF_PIN_COUNT   $n_buf_pins"
puts $f "SINK_PIN_CAP_fF $C_sink_pin_fF"
puts $f "SINK_COUNT      $n_sinks"
puts $f "VDD             0.7"
puts $f "FREQ_HZ         2.222e9"
close $f

puts ""
puts [format "  Mesh buf input pins : %d  total = %.2f fF" $n_buf_pins $C_buf_pin_fF]
puts [format "  FF sink clock pins  : %d  total = %.2f fF" $n_sinks    $C_sink_pin_fF]
puts "Pin caps written: $pin_file"
puts ""
puts "Run dynamic power analysis with:"
puts "  python3 parse_spef_power.py $spef_file $pin_file"

# --- Verify no mesh geometry inside any macro ---
set mesh_net [$block findNet "clk_mesh"]
if { $mesh_net == "NULL" } {
    puts "ERROR: clk_mesh not found"
    exit 1
}

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
puts "===== aes-block mesh v4 (8x8) verification ====="
puts "Real macros:       [llength $macro_rects]"
puts "Wire violations:   $wire_v"
puts "Buffer violations: $buf_v"
puts "================================================="

set odb_file [make_result_file "aes_block_mesh_v4.odb"]
puts "Writing ODB: $odb_file"
write_db $odb_file

set def_file [make_result_file "aes_block_mesh_v4.def"]
puts "Writing DEF: $def_file"
write_def $def_file

puts ""
puts "View layout with:  openroad -gui $odb_file"

exit [expr {($wire_v == 0 && $buf_v == 0) ? 0 : 1}]
