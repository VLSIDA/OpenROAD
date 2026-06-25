# Full mesh flow on ibex (asap7) — pure standard-cell design, no macros.
# Fixed pitch 2.0 um. Same flow as mesh_swerv_macros.tcl but with
# -droute_end_iter 1 and outputs landing in results/ibex/.

source "../helpers.tcl"

# Redirect make_result_file outputs into results/ibex_12x12/
set result_dir [file join [file dirname [file normalize [info script]]] \
                          ".." "results" "ibex_12x12"]
file mkdir $result_dir

set orfs /home/wali2/OpenROAD-flow-scripts/flow
set results $orfs/results/asap7/ibex/base
set plat $orfs/platforms/asap7

read_db $results/3_3_place_gp.odb

read_liberty $plat/lib/NLDM/asap7sc7p5t_AO_RVT_TT_nldm_211120.lib.gz
read_liberty $plat/lib/NLDM/asap7sc7p5t_INVBUF_RVT_TT_nldm_220122.lib.gz
read_liberty $plat/lib/NLDM/asap7sc7p5t_OA_RVT_TT_nldm_211120.lib.gz
read_liberty $plat/lib/NLDM/asap7sc7p5t_SIMPLE_RVT_TT_nldm_211120.lib.gz
read_liberty $plat/lib/NLDM/asap7sc7p5t_SEQ_RVT_TT_nldm_220123.lib

# --- Bypass clock-gating cells -----------------------------------------
# Per advisor: in a mesh CDN, no gating is allowed between mesh and sinks
# (only at the root). The flops downstream of ICGs have built-in EN pins,
# so bypassing the ICG keeps functionality; we only lose the per-region
# clock-tree power saving. MUST run before read_sdc so STA builds its
# clock graph from the already-bypassed netlist (otherwise CMS only sees
# the pre-bypass 995 sinks).
puts "Bypassing clock-gating cells..."
set block [ord::get_db_block]
set root_net [$block findNet clk_i]
if { $root_net == "NULL" } {
    puts "ERROR: clk_i net not found"
    exit 1
}
set bypass_sinks 0
set bypass_nets 0
foreach n [$block getNets] {
    if { $n == $root_net } { continue }
    if { [string match "*_mesh" [$n getName]] } { continue }
    set sinks {}
    foreach it [$n getITerms] {
        set mt [$it getMTerm]
        if { $mt == "NULL" } { continue }
        if { [$it getIoType] ne "INPUT" } { continue }
        if { [$mt getSigType] ne "CLOCK" } { continue }
        lappend sinks $it
    }
    if { [llength $sinks] == 0 } { continue }
    foreach it $sinks {
        $it disconnect
        $it connect $root_net
    }
    puts "  bypassed [$n getName]: moved [llength $sinks] sinks to clk_i"
    incr bypass_nets
    incr bypass_sinks [llength $sinks]
}
puts "Bypass: moved $bypass_sinks gated sinks across $bypass_nets nets onto clk_i"

read_sdc $results/3_place.sdc

puts "Legalizing initial placement..."
detailed_placement

source $plat/setRC.tcl

set block [ord::get_db_block]
set macro_count 0
foreach inst [$block getInsts] {
    if { [$inst isBlock] } { incr macro_count }
}
puts "Real macros: $macro_count"

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

create_clock_mesh \
    -clock $clk_name \
    -h_layer M4 \
    -v_layer M5 \
    -pitch 6.78 \
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

global_route -guide_file [make_result_file "ibex_mesh.guide"] \
    -congestion_iterations 50

detailed_route -output_drc [make_result_file "ibex_mesh_drc.rpt"] \
    -droute_end_iter 0

connect_proxy_bterms_to_mesh -clock $clk_name
estimate_parasitics -global_routing
capture_mesh_arrivals -clock $clk_name
merge_mesh_nets -clock $clk_name
convert_mesh_swire -clock $clk_name

extract_parasitics -ext_model_file $plat/rcx_patterns.rules \
    -cc_model 10 -coupling_threshold 0.1 -max_res 0 -skip_over_cell

set spef_file [make_result_file "ibex_mesh.spef"]
puts "Writing SPEF: $spef_file"
write_spef $spef_file

set script_dir [file dirname [file normalize [info script]]]
set spice_file [make_result_file "ibex_mesh.spice"]
write_mesh_spice -clock $clk_name -output $spice_file -vdd 0.7 \
    -spice_models [list $script_dir/asap7_buf_ideal.spice]

set spice_zd [make_result_file "ibex_mesh_zero_delay.spice"]
write_mesh_spice -clock $clk_name -output $spice_zd -vdd 0.7 \
    -spice_models [list $script_dir/asap7_buf_ideal.spice] \
    -zero_delay

set mesh_net [$block findNet "clk_mesh"]
if { $mesh_net == "NULL" } {
    foreach n [$block getNets] {
        if { [string match "*_mesh" [$n getName]] } { set mesh_net $n; break }
    }
}
if { $mesh_net == "NULL" } {
    puts "ERROR: no *_mesh net found"
    exit 1
}
puts "Mesh net: [$mesh_net getName]"

set odb_file [make_result_file "ibex_mesh.odb"]
puts "Writing ODB: $odb_file"
write_db $odb_file

set def_file [make_result_file "ibex_mesh.def"]
write_def $def_file

puts "View layout with:  openroad -gui $odb_file"
exit 0
