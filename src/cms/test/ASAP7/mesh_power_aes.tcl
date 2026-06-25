# Quick mesh power report for aes-block.
# Reads the saved mesh ODB and runs estimate_parasitics -placement (fast).
# Compare Clock row against cts_baseline power.

source "../helpers.tcl"

set orfs /home/wali2/OpenROAD-flow-scripts/flow
set plat $orfs/platforms/asap7
set sdc_dir $orfs/results/asap7/aes-block/base

set mesh_odb [make_result_file "aes_block_mesh_v3.odb"]
if { ![file exists $mesh_odb] } {
    puts "ERROR: $mesh_odb not found — run mesh_aes_block_macros.tcl first"
    exit 1
}
read_db $mesh_odb

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

read_sdc $sdc_dir/3_place.sdc
source $plat/setRC.tcl

estimate_parasitics -placement

puts ""
puts "=========================================="
puts "  Power report  (aes MESH, placement parasitics)"
puts "=========================================="
report_power
puts "=========================================="
puts ""
puts "  CTS baseline reference: Clock = 2.10e-03 W  Total = 6.60e-03 W"
puts "=========================================="

exit 0
