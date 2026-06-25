set orfs /home/wali2/backside/OpenROAD-flow-scripts/flow
set plat $orfs/platforms/gt2n
read_db $orfs/results/gt2n/gcd/base/3_place.odb
read_liberty $plat/lib/gt2_6t_w31_lvt_tt_0p7v25c.lib
read_sdc $orfs/results/gt2n/gcd/base/3_place.sdc
detailed_placement
source $plat/setRC.tcl
set clk ""; foreach c [sta::all_clocks] { set clk [get_name $c]; break }
create_clock_mesh -clock $clk -h_layer BM2 -v_layer BM1 -pitch 1.0 -buffers {gt2_6t_buf_x4_w31_lvt} -cts_buffers {gt2_6t_buf_x4_w31_lvt}
detailed_placement -max_displacement 1000
set b [ord::get_db_block]
set die [$b getDieArea]
set core [$b getCoreArea]
puts "DIE  ([$die xMin],[$die yMin])-([$die xMax],[$die yMax])"
puts "CORE ([$core xMin],[$core yMin])-([$core xMax],[$core yMax])"
# rows
set nrows [llength [$b getRows]]
puts "ROWS=$nrows"
set r0 [lindex [$b getRows] 0]
puts "row0 site h=[[$r0 getSite] getHeight] origin=[$r0 getOrigin]"
# instance count + total cell area vs core area (utilization)
set ninst 0; set carea 0
foreach inst [$b getInsts] {
  incr ninst
  set bb [$inst getBBox]; set carea [expr {$carea + ([$bb xMax]-[$bb xMin])*([$bb yMax]-[$bb yMin])}]
}
set ca [expr {([$core xMax]-[$core xMin])*([$core yMax]-[$core yMin])}]
puts "INSTS=$ninst cell_area=$carea core_area=$ca util=[format %.1f [expr {100.0*$carea/$ca}]]%"
# mesh buffer locations (sample 4)
set nb 0
foreach inst [$b getInsts] {
  if {[string match "*mesh*" [$inst getName]] || [string match "*clkbuf*mesh*" [$inst getName]]} {
    if {$nb < 4} { set bb [$inst getBBox]; puts "MESHBUF [$inst getName] master=[[$inst getMaster] getName] ll=([$bb xMin],[$bb yMin])" }
    incr nb
  }
}
puts "MESHBUF total matched=$nb"
exit 0
