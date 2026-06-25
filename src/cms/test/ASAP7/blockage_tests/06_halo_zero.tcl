source _common.tcl
init_design

# Same as scenario 1 but halo = 0. Verifies blockage logic works without halo.
set block [ord::get_db_block]
set core [$block getCoreArea]
set cx1 [$core xMin]; set cy1 [$core yMin]
set cx2 [$core xMax]; set cy2 [$core yMax]
set w [expr {$cx2 - $cx1}]; set h [expr {$cy2 - $cy1}]

odb::dbBlockage_create $block \
    [expr {$cx1 + $w*35/100}] [expr {$cy1 + $h*35/100}] \
    [expr {$cx1 + $w*65/100}] [expr {$cy1 + $h*65/100}]

set result [run_and_verify 0.0]
report_and_exit "Zero-halo blockage" $result
