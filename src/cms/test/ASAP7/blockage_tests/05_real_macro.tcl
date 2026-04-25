source _common.tcl
init_design

# Read fakeram LEF AFTER read_db so layer refs bind to the ODB's tech
read_lef "/home/wajid/OpenROAD-flow-scripts/flow/platforms/asap7/lef/fakeram7_256x32.lef"

# Create a real macro instance in the design AFTER detailed_placement.
# This exercises the dbInst::isBlock() path in collectBlockageRects.
set db [ord::get_db]
set block [ord::get_db_block]
set master [$db findMaster fakeram7_256x32]
if { $master == "NULL" } {
    puts "ERROR: fakeram7_256x32 master not found"
    exit 1
}

set core [$block getCoreArea]
set cx1 [$core xMin]; set cy1 [$core yMin]
set cx2 [$core xMax]; set cy2 [$core yMax]
set w [expr {$cx2 - $cx1}]; set h [expr {$cy2 - $cy1}]

set macro_inst [odb::dbInst_create $block $master "test_macro_inst"]
# Place in the upper-right quadrant of the core
$macro_inst setLocation \
    [expr {$cx1 + $w*55/100}] [expr {$cy1 + $h*55/100}]
$macro_inst setPlacementStatus FIRM

puts "Injected real macro instance: [$macro_inst getName]"
puts "  isBlock = [$macro_inst isBlock]"
set bb [$macro_inst getBBox]
puts "  bbox    = ([$bb xMin],[$bb yMin])-([$bb xMax],[$bb yMax])"

set result [run_and_verify 0.5]
report_and_exit "Real macro via dbInst::isBlock()" $result
