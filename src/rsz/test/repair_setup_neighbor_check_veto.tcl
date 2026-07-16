# repair_setup6 with -neighbor_check and a legacy lambda argument (parsed but
# ignored by the WNS rule).  With the tree's own annotated per-leaf timing and
# the tree-critical leaf excluded (it is the on-path story, owned by the
# accept machinery), no rebuffer veto fires on this design and the output
# matches plain repair_setup6 -- locking in option compatibility and the
# absence of self-vetoes against recoverArea's intentional slack giveback.
source "helpers.tcl"
set repair_args [list -neighbor_check -neighbor_check_lambda 5]
source "repair_setup6.tcl"
