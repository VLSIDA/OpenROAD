# repair_setup6 with a lambda high enough that the neighbor check vetoes a
# rebuffer candidate, forcing the optimizer onto a different repair route.
source "helpers.tcl"
set repair_args [list -neighbor_check -neighbor_check_lambda 5]
source "repair_setup6.tcl"
