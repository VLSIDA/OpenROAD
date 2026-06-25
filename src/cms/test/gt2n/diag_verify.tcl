set rdir [file join [file dirname [file normalize [info script]]] results]
read_db $rdir/gcd_front.odb
set b [ord::get_db_block]

# instance occupancy
set insts {}
foreach inst [$b getInsts] {
  set bb [$inst getBBox]
  lappend insts [list [$bb xMin] [$bb yMin] [$bb xMax] [$bb yMax]]
}
proc inside {rects x y} {
  foreach r $rects { lassign $r x0 y0 x1 y1
    if {$x >= $x0 && $x <= $x1 && $y >= $y0 && $y <= $y1} { return 1 } }
  return 0
}

# mesh (clk_mesh) special-wire rects on backside layers
set mesh {}
set mnet [$b findNet clk_mesh]
if {$mnet ne "NULL"} {
  foreach sw [$mnet getSWires] { foreach w [$sw getWires] {
    if {![$w isVia]} {
      set ly [$w getTechLayer]
      if {$ly ne "NULL" && [$ly isBackside]} {
        lappend mesh [list [$w xMin] [$w yMin] [$w xMax] [$w yMax]]
      }
    }
  }}
}
puts "clk_mesh backside wire segments: [llength $mesh]"

set total 0; set in_cell 0; set with_via 0; set off_mesh 0
set bboxes {}
foreach net [$b getNets] {
  set nm [$net getName]
  if {![string match "sink_*" $nm] && ![string match "*_buf_*" $nm]} continue
  incr total
  foreach bt [$net getBTerms] { foreach bp [$bt getBPins] { foreach box [$bp getBoxes] {
    set cx [expr {([$box xMin]+[$box xMax])/2}]
    set cy [expr {([$box yMin]+[$box yMax])/2}]
    if {[inside $insts $cx $cy]} { incr in_cell }
    if {![inside $mesh $cx $cy]} { incr off_mesh }
    lappend bboxes [list [$box xMin] [$box yMin] [$box xMax] [$box yMax]]
  }}}
  set nv 0
  foreach sw [$net getSWires] { incr nv [llength [$sw getWires]] }
  if {$nv > 0} { incr with_via }
}
# pairwise BTerm-box overlap
set overlaps 0
for {set i 0} {$i < [llength $bboxes]} {incr i} {
  lassign [lindex $bboxes $i] ax0 ay0 ax1 ay1
  for {set j [expr {$i+1}]} {$j < [llength $bboxes]} {incr j} {
    lassign [lindex $bboxes $j] bx0 by0 bx1 by1
    if {$ax0 < $bx1 && $bx0 < $ax1 && $ay0 < $by1 && $by0 < $ay1} { incr overlaps }
  }
}
puts "CROSSING nets: $total | over-cell: $in_cell | off-mesh: $off_mesh | column: $with_via | BTerm-pair-overlaps: $overlaps"
exit 0
