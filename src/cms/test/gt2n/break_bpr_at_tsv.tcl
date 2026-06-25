# Stage 2: break the BPR power rails (BOTH vdd & vss) at every gt2_6t_TSV cell.
#
# Geometry: the TSV's signal BPR pad overlaps one row rail and sits 0.092um from
# the other (< 0.112 min spacing), and a 0.144 row can't fit the pad with legal
# clearance to either -> both rails must be cut locally. We trim each BPR
# followpin segment over [cell.xMin - halo, cell.xMax + halo] (halo = 0.112um),
# leaving stubs that meet end-to-end spacing.
#
# Usage: openroad break_bpr_at_tsv.tcl   (reads stage1 odb, writes *_break.odb)
set in   /home/wali2/backside/OpenROAD/src/cms/test/gt2n/results/gcd_tsv_stage1.odb
set out  /home/wali2/backside/OpenROAD/src/cms/test/gt2n/results/gcd_tsv_break.odb

read_db $in
set block [ord::get_db_block]
set tech  [ord::get_db_tech]
set bpr   [$tech findLayer BPR]
if {$bpr eq "NULL"} { puts "ERROR: no BPR layer"; exit 1 }
set dbu  [$block getDbUnitsPerMicron]
set halo [expr {int(round(0.112 * $dbu))}]

# Cut windows: one per gt2_6t_TSV. X expanded by halo each side; Y = full cell
# height so both the bottom (y=cell.yMin) and top (y=cell.yMax) row rails fall in.
set cuts {}
foreach inst [$block getInsts] {
  if {[[$inst getMaster] getName] ne "gt2_6t_TSV"} continue
  set bb [$inst getBBox]
  lappend cuts [list [expr {[$bb xMin]-$halo}] [$bb yMin] [expr {[$bb xMax]+$halo}] [$bb yMax]]
}
puts ">>> [llength $cuts] TSV cut windows (halo = $halo dbu = 0.112 um)"

proc overlap {alo ahi blo bhi} { return [expr {$alo < $bhi && $blo < $ahi}] }

set bpr_before 0; set bpr_after 0; set trimmed 0
foreach net [$block getNets] {
  set st [$net getSigType]
  if {$st ne "POWER" && $st ne "GROUND"} continue
  foreach swire [$net getSWires] {
    set boxes {}
    foreach sb [$swire getWires] { lappend boxes $sb }
    foreach sb $boxes {
      if {[$sb isVia]} continue
      if {[[$sb getTechLayer] getName] ne "BPR"} continue
      incr bpr_before
      set rxl [$sb xMin]; set ryl [$sb yMin]; set rxh [$sb xMax]; set ryh [$sb yMax]
      # x-intervals to remove (windows overlapping this rail in Y)
      set rem {}
      foreach c $cuts {
        lassign $c wxl wyl wxh wyh
        if {![overlap $wyl $wyh $ryl $ryh]} continue
        set lo [expr {max($wxl,$rxl)}]; set hi [expr {min($wxh,$rxh)}]
        if {$lo < $hi} { lappend rem [list $lo $hi] }
      }
      if {![llength $rem]} { incr bpr_after; continue }
      # merge removal intervals
      set rem [lsort -integer -index 0 $rem]
      set merged {}
      foreach iv $rem {
        lassign $iv a b
        if {[llength $merged] && $a <= [lindex [lindex $merged end] 1]} {
          lset merged end 1 [expr {max([lindex [lindex $merged end] 1],$b)}]
        } else { lappend merged $iv }
      }
      # surviving stub intervals
      set surv {}; set cur $rxl
      foreach iv $merged {
        lassign $iv a b
        if {$a > $cur} { lappend surv [list $cur $a] }
        if {$b > $cur} { set cur $b }
      }
      if {$cur < $rxh} { lappend surv [list $cur $rxh] }
      # recreate stubs (same layer/y/shape), then destroy the original rail seg
      set wst [$sb getWireShapeType]
      foreach s $surv {
        lassign $s a b
        odb::dbSBox_create $swire $bpr $a $ryl $b $ryh $wst
        incr bpr_after
      }
      odb::dbSBox_destroy $sb
      incr trimmed
    }
  }
}
puts ">>> BPR followpin segs: before=$bpr_before  after=$bpr_after  (trimmed $trimmed, +[expr {$bpr_after-$bpr_before+$trimmed}] stubs)"
write_db $out
puts ">>> wrote $out"
exit 0
