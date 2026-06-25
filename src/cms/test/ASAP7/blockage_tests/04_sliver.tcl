source _common.tcl
init_design

# Blockage positioned so that only a small sliver of rail remains at one edge,
# smaller than min_segment_length (= 2 * wire_width). The sliver should be
# filtered out by clipWireByBlockages.
set block [ord::get_db_block]
set core [$block getCoreArea]
set cx1 [$core xMin]; set cy1 [$core yMin]
set cx2 [$core xMax]; set cy2 [$core yMax]

# M4 wire width on ASAP7 is ~18 DBU in this tech. min_segment_length = 36 DBU.
# Leave only ~20 DBU of rail between core xMin and blockage xMin → sliver.
# (Exact threshold doesn't matter; we just verify no violation + no crash.)
odb::dbBlockage_create $block \
    [expr {$cx1 + 20}] [expr {$cy1 + 100}] \
    [expr {$cx2 - 100}] [expr {$cy2 - 100}]

set result [run_and_verify 0.5]
report_and_exit "Sliver-producing blockage" $result
