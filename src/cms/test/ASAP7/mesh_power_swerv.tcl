# Quick mesh power report for swerv_wrapper.
# Reads the saved mesh ODB and runs estimate_parasitics -placement (fast).
# Compare Clock row against cts_baseline power.

source "../helpers.tcl"

set orfs /home/wali2/OpenROAD-flow-scripts/flow
set plat $orfs/platforms/asap7
set sdc_dir $orfs/results/asap7/swerv_wrapper/base

set mesh_odb [make_result_file "swerv_mesh_v2.odb"]
if { ![file exists $mesh_odb] } {
    puts "ERROR: $mesh_odb not found — run mesh_swerv_macros.tcl first"
    exit 1
}
read_db $mesh_odb

read_liberty $plat/lib/NLDM/asap7sc7p5t_AO_RVT_TT_nldm_211120.lib.gz
read_liberty $plat/lib/NLDM/asap7sc7p5t_INVBUF_RVT_TT_nldm_220122.lib.gz
read_liberty $plat/lib/NLDM/asap7sc7p5t_OA_RVT_TT_nldm_211120.lib.gz
read_liberty $plat/lib/NLDM/asap7sc7p5t_SIMPLE_RVT_TT_nldm_211120.lib.gz
read_liberty $plat/lib/NLDM/asap7sc7p5t_SEQ_RVT_TT_nldm_220123.lib

foreach lib [glob $orfs/designs/asap7/swerv_wrapper/lib/*.lib] {
    read_liberty $lib
}

read_sdc $sdc_dir/3_place.sdc
source $plat/setRC.tcl

estimate_parasitics -placement

puts ""
puts "=========================================="
puts "  Power report  (swerv MESH, placement parasitics)"
puts "=========================================="
report_power
puts "=========================================="

exit 0
