# CTS baseline for swerv_wrapper.
# Uses ORFS postroute.odb (CTS + routing already done) for accurate parasitics.
# Compare against mesh SPICE skew result.

source "../helpers.tcl"

set orfs /home/wali2/OpenROAD-flow-scripts/flow
set results $orfs/results/asap7/swerv_wrapper/base
set plat $orfs/platforms/asap7

read_db $results/postroute.odb

read_liberty $plat/lib/NLDM/asap7sc7p5t_AO_RVT_TT_nldm_211120.lib.gz
read_liberty $plat/lib/NLDM/asap7sc7p5t_INVBUF_RVT_TT_nldm_220122.lib.gz
read_liberty $plat/lib/NLDM/asap7sc7p5t_OA_RVT_TT_nldm_211120.lib.gz
read_liberty $plat/lib/NLDM/asap7sc7p5t_SIMPLE_RVT_TT_nldm_211120.lib.gz
read_liberty $plat/lib/NLDM/asap7sc7p5t_SEQ_RVT_TT_nldm_220123.lib

foreach lib [glob $orfs/designs/asap7/swerv_wrapper/lib/*.lib] {
    read_liberty $lib
}

read_sdc $results/postroute.sdc
source $plat/setRC.tcl

estimate_parasitics -placement

puts ""
puts "=========================================="
puts "  Power report  (swerv CTS baseline)"
puts "=========================================="
report_power
puts "=========================================="
puts ""
puts "=========================================="
puts "  TritonCTS skew report  (swerv_wrapper)"
puts "=========================================="
report_clock_skew
puts ""
puts "  Clock latency per endpoint:"
report_clock_latency
puts ""
puts "=========================================="

write_db [make_result_file "swerv_cts_baseline.odb"]
exit 0
