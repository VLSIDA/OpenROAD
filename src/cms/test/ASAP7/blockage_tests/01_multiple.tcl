source _common.tcl
init_design

# Three non-overlapping blockages in different parts of the core
set block [ord::get_db_block]
set core [$block getCoreArea]
set cx1 [$core xMin]; set cy1 [$core yMin]
set cx2 [$core xMax]; set cy2 [$core yMax]
set w [expr {$cx2 - $cx1}]; set h [expr {$cy2 - $cy1}]

# Top-left, top-right, bottom-center
odb::dbBlockage_create $block \
    [expr {$cx1 + $w*15/100}] [expr {$cy1 + $h*70/100}] \
    [expr {$cx1 + $w*30/100}] [expr {$cy1 + $h*85/100}]
odb::dbBlockage_create $block \
    [expr {$cx1 + $w*65/100}] [expr {$cy1 + $h*70/100}] \
    [expr {$cx1 + $w*80/100}] [expr {$cy1 + $h*85/100}]
odb::dbBlockage_create $block \
    [expr {$cx1 + $w*40/100}] [expr {$cy1 + $h*15/100}] \
    [expr {$cx1 + $w*60/100}] [expr {$cy1 + $h*30/100}]

set result [run_and_verify 0.5]
report_and_exit "Multiple non-overlapping blockages" $result
