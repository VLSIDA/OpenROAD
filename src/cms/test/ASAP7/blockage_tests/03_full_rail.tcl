source _common.tcl
init_design

# One blockage spanning the full core width — tests that an entire h-rail
# may be fully removed (clipWireByBlockages returns empty) without crashing.
set block [ord::get_db_block]
set core [$block getCoreArea]
set cx1 [$core xMin]; set cy1 [$core yMin]
set cx2 [$core xMax]; set cy2 [$core yMax]
set h [expr {$cy2 - $cy1}]

# Full-width horizontal band near middle
odb::dbBlockage_create $block \
    $cx1 [expr {$cy1 + $h*48/100}] \
    $cx2 [expr {$cy1 + $h*55/100}]

set result [run_and_verify 0.5]
report_and_exit "Full-width horizontal blockage" $result
