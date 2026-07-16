# repair_setup6 with -neighbor_check and a legacy lambda argument.  Under the
# WNS-degradation rule the lambda is parsed but ignored, and no fanout of this
# design ever becomes the local region's governing worst slack, so the output
# matches plain repair_setup6 -- locking in both the option compatibility and
# the rule change (the lambda-era rule vetoed a rebuffer here).
source "helpers.tcl"
set repair_args [list -neighbor_check -neighbor_check_lambda 5]
source "repair_setup6.tcl"
