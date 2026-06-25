# mesh_aes_block_v4_adaptive.tcl
# Same flow as mesh_aes_block_v4.tcl but exercises the new clocksyn-style
# load-adaptive buffer sizing (LoadAdaptiveBuffer.cc). The buffer_list
# contains multiple BUFx masters; ClockMesh::createMeshGrid auto-dispatches
# to placeBuffersLoadAdaptive() whenever buffer_list.size() > 1, picking the
# smallest master whose Liberty max_capacitance can drive each intersection's
# local load.

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

set block [ord::get_db_block]
set clk_name clk

# Candidate library spans 12x drive ratio — load-adaptive sizing picks per
# intersection. ASAP7 75t RVT BUF family available in the loaded liberty:
#   BUFx2 BUFx3 BUFx4 BUFx5 BUFx8 BUFx10 BUFx12 BUFx24
set buffer_list [list \
    BUFx2_ASAP7_75t_R \
    BUFx4_ASAP7_75t_R \
    BUFx8_ASAP7_75t_R \
    BUFx12_ASAP7_75t_R \
    BUFx24_ASAP7_75t_R]

set grid_n 8
set core  [$block getCoreArea]
set core_w [ord::dbu_to_microns [expr { [$core xMax] - [$core xMin] }]]
set core_h [ord::dbu_to_microns [expr { [$core yMax] - [$core yMin] }]]
set pitch  [expr { min($core_w, $core_h) / ($grid_n - 1.0) }]

puts [format "Core area    : %.3f x %.3f um" $core_w $core_h]
puts [format "Target grid  : %dx%d" $grid_n $grid_n]
puts [format "Computed pitch: %.4f um" $pitch]
puts "Buffer library: $buffer_list"

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

puts "Connecting sinks to mesh..."
connect_sinks_to_mesh -clock $clk_name -proxy_layer M6

set_routing_layers -signal M2-M7 -clock M2-M7
global_route -guide_file [make_result_file "aes_block_mesh_v4_adaptive.guide"] \
    -congestion_iterations 50
detailed_route -output_drc [make_result_file "aes_block_mesh_v4_adaptive_drc.rpt"] \
    -droute_end_iter 0

connect_proxy_bterms_to_mesh -clock $clk_name

estimate_parasitics -global_routing
capture_mesh_arrivals -clock $clk_name

set_propagated_clock [get_clocks $clk_name]

# --- Report what load-adaptive actually picked ---
puts ""
puts "===== Adaptive buffer placement summary ====="
array set counts {}
foreach inst [$block getInsts] {
    if { ! [string match "mesh_buf_*" [$inst getName]] } { continue }
    set m [[$inst getMaster] getName]
    if { ![info exists counts($m)] } { set counts($m) 0 }
    incr counts($m)
}
foreach m [lsort [array names counts]] {
    puts [format "  %-22s : %d instances" $m $counts($m)]
}
puts "============================================="
puts ""

merge_mesh_nets -clock $clk_name
convert_mesh_swire -clock $clk_name

extract_parasitics -ext_model_file $plat/rcx_patterns.rules \
    -cc_model 10 -coupling_threshold 0.1 -max_res 0 -skip_over_cell

set spef_file [make_result_file "aes_block_mesh_v4_adaptive.spef"]
write_spef $spef_file

set script_dir [file dirname [file normalize [info script]]]

set spice_file [make_result_file "aes_block_mesh_v4_adaptive.spice"]
write_mesh_spice -clock $clk_name -output $spice_file -vdd 0.7 \
    -spice_models [list $script_dir/asap7_buf.spice]

set odb_file [make_result_file "aes_block_mesh_v4_adaptive.odb"]
write_db $odb_file

puts ""
puts "Wrote: $spice_file"
puts "Wrote: $spef_file"
puts "Wrote: $odb_file"
