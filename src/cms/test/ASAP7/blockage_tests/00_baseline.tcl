source _common.tcl
init_design
# No blockages injected — pure baseline to give reference counts
set result [run_and_verify 0.0]
report_and_exit "Baseline (no blockages)" $result
