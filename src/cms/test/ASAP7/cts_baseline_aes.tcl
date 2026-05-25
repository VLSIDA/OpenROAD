# CTS baseline for aes-block — runs TritonCTS, reports STA skew.
# Compare against mesh SPICE result (43.5 ps).

source "../helpers.tcl"

set orfs /home/wali2/OpenROAD-flow-scripts/flow
set results $orfs/results/asap7/aes-block/base
set plat $orfs/platforms/asap7

# Same placed ODB the mesh flow uses
read_db $results/3_place.odb

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
source $plat/setRC.tcl

repair_clock_inverters

puts "Running TritonCTS..."
clock_tree_synthesis \
    -sink_clustering_enable \
    -repair_clock_nets

set_propagated_clock [all_clocks]

detailed_placement

set_routing_layers -signal M2-M7 -clock M2-M7
global_route -congestion_iterations 50
estimate_parasitics -global_routing
puts ""
puts "=========================================="
puts "  Power report  (aes CTS baseline)"
puts "=========================================="
report_power
puts "=========================================="
puts ""
puts "=========================================="
puts "  TritonCTS skew report  (aes-block)"
puts "=========================================="
report_clock_skew
puts ""
puts "  Clock latency per endpoint:"
report_clock_latency
puts ""
puts "=========================================="
puts ""
puts "  Mesh SPICE skew (reference): 43.5 ps"
puts "=========================================="

write_db [make_result_file "aes_cts_baseline.odb"]
exit 0
