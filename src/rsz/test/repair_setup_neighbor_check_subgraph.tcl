# repair_setup4 with -neighbor_check: exercises the subgraph-STA neighbor
# evaluation on a case with real fanin stages and multi-load frontiers
# (cell-swap evaluations with up to 10 neighbors on this design).
source "helpers.tcl"
set repair_args [list -neighbor_check]
source "repair_setup4.tcl"
