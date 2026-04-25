source _common.tcl
init_design

# Two overlapping blockages — exercises interval merge in clipWireByBlockages
set block [ord::get_db_block]
set core [$block getCoreArea]
set cx1 [$core xMin]; set cy1 [$core yMin]
set cx2 [$core xMax]; set cy2 [$core yMax]
set w [expr {$cx2 - $cx1}]; set h [expr {$cy2 - $cy1}]

# Rect A: 30-55% x, 40-60% y
odb::dbBlockage_create $block \
    [expr {$cx1 + $w*30/100}] [expr {$cy1 + $h*40/100}] \
    [expr {$cx1 + $w*55/100}] [expr {$cy1 + $h*60/100}]
# Rect B: 45-70% x, 50-70% y  (overlaps A in corner)
odb::dbBlockage_create $block \
    [expr {$cx1 + $w*45/100}] [expr {$cy1 + $h*50/100}] \
    [expr {$cx1 + $w*70/100}] [expr {$cy1 + $h*70/100}]

set result [run_and_verify 0.5]
report_and_exit "Overlapping blockages (merge)" $result
