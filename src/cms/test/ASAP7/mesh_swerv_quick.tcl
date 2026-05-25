# Fast mesh-only variant for swerv_wrapper — skip routing/extraction/spice
# so you can view the layout quickly. Produces an ODB right after mesh
# creation + buffer placement + BTerm setup.

source "../helpers.tcl"

set orfs /home/wali2/OpenROAD-flow-scripts/flow
set results $orfs/results/asap7/swerv_wrapper/base
set plat $orfs/platforms/asap7

read_db $results/3_3_place_gp.odb

read_liberty $plat/lib/NLDM/asap7sc7p5t_AO_RVT_TT_nldm_211120.lib.gz
read_liberty $plat/lib/NLDM/asap7sc7p5t_INVBUF_RVT_TT_nldm_220122.lib.gz
read_liberty $plat/lib/NLDM/asap7sc7p5t_OA_RVT_TT_nldm_211120.lib.gz
read_liberty $plat/lib/NLDM/asap7sc7p5t_SIMPLE_RVT_TT_nldm_211120.lib.gz
read_liberty $plat/lib/NLDM/asap7sc7p5t_SEQ_RVT_TT_nldm_220123.lib
foreach lib [glob $orfs/designs/asap7/swerv_wrapper/lib/*.lib] {
    read_liberty $lib
}
read_sdc $results/3_place.sdc

detailed_placement
source $plat/setRC.tcl

set block [ord::get_db_block]
set macro_count 0
foreach inst [$block getInsts] {
    if { [$inst isBlock] } { incr macro_count }
}
puts "Real macros: $macro_count"

set clk_name ""
foreach c [sta::all_clocks] { set clk_name [get_name $c]; break }
puts "Using clock: $clk_name"

create_clock_mesh \
    -clock $clk_name \
    -h_layer M4 \
    -v_layer M5 \
    -pitch 10.0 \
    -buffers [list BUFx4_ASAP7_75t_R] \
    -macro_halo 0.5

detailed_placement -max_displacement 1000

set odb_file [make_result_file "swerv_mesh_quick.odb"]
puts "Writing ODB (mesh only, no routing): $odb_file"
write_db $odb_file

puts "View layout with:  openroad -gui $odb_file"
exit 0
