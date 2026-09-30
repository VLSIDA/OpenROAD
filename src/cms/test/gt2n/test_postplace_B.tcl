# Approach B (post-placement): start from the fully placed + PDN'd gcd, insert
# TSVs at the frozen-grid intersections, BREAK the BPR rails at each TSV row
# (real swire surgery), then RELOCATE the std cells that lost power to nearby
# legal positions (detailed_placement honors the placement blockages).
set orfs /home/wali2/backside/OpenROAD-flow-scripts/flow
set res  $orfs/results/gt2n/gcd/base
set plat $orfs/platforms/gt2n

read_db  $res/3_place.odb
read_lef $plat/lef/gt2_6t_TSV.lef

# 1) Insert TSVs + keepouts at every intersection (placement blockages drive the
#    relocation). keepout_w 0.31 == the BPR break window (cell + 2*0.112).
reserve_clock_mesh -h_layer BM2 -v_layer BM1 -pitch 1.0 \
    -strap_pitch 2.16 -strap_offset 1.08 -strap_width 0.36 \
    -tsv_master gt2_6t_TSV -spacing 0 -keepout_w 0.31 -keepout_h 0.16

set block [ord::get_db_block]
set tech  [ord::get_db_tech]
set bpr   [$tech findLayer BPR]
set dbu   [$block getDbUnitsPerMicron]
set halo  [expr {int(round(0.112 * $dbu))}]

proc rov {alo ahi blo bhi} { return [expr {$alo < $bhi && $blo < $ahi}] }

# 2) How many std cells currently sit in a keepout (these must relocate)?
set keeps {}
foreach b [$block getBlockages] {
  set bb [$b getBBox]
  lappend keeps [list [$bb xMin] [$bb yMin] [$bb xMax] [$bb yMax]]
}
set in_keepout 0
foreach inst [$block getInsts] {
  set m [[$inst getMaster] getName]
  if {$m eq "gt2_6t_TSV"} continue
  if {![$inst isCore]} continue
  set ib [$inst getBBox]
  foreach k $keeps {
    lassign $k kx0 ky0 kx1 ky1
    if {[rov [$ib xMin] [$ib xMax] $kx0 $kx1] && [rov [$ib yMin] [$ib yMax] $ky0 $ky1]} {
      incr in_keepout; break
    }
  }
}
puts ">>> std cells sitting in a keepout (to relocate): $in_keepout"

# 3) Break BPR rails at each TSV (window = cell.x +/- halo, full cell y).
set cuts {}
foreach inst [$block getInsts] {
  if {[[$inst getMaster] getName] ne "gt2_6t_TSV"} continue
  set bb [$inst getBBox]
  lappend cuts [list [expr {[$bb xMin]-$halo}] [$bb yMin] [expr {[$bb xMax]+$halo}] [$bb yMax]]
}
set trimmed 0
foreach net [$block getNets] {
  set st [$net getSigType]
  if {$st ne "POWER" && $st ne "GROUND"} continue
  foreach swire [$net getSWires] {
    set boxes {}
    foreach sb [$swire getWires] { lappend boxes $sb }
    foreach sb $boxes {
      if {[$sb isVia]} continue
      if {[[$sb getTechLayer] getName] ne "BPR"} continue
      set rxl [$sb xMin]; set ryl [$sb yMin]; set rxh [$sb xMax]; set ryh [$sb yMax]
      set rem {}
      foreach c $cuts {
        lassign $c wxl wyl wxh wyh
        if {![rov $wyl $wyh $ryl $ryh]} continue
        set lo [expr {max($wxl,$rxl)}]; set hi [expr {min($wxh,$rxh)}]
        if {$lo < $hi} { lappend rem [list $lo $hi] }
      }
      if {![llength $rem]} continue
      set rem [lsort -integer -index 0 $rem]
      set merged {}
      foreach iv $rem {
        lassign $iv a b
        if {[llength $merged] && $a <= [lindex [lindex $merged end] 1]} {
          lset merged end 1 [expr {max([lindex [lindex $merged end] 1],$b)}]
        } else { lappend merged $iv }
      }
      set surv {}; set cur $rxl
      foreach iv $merged { lassign $iv a b
        if {$a > $cur} { lappend surv [list $cur $a] }
        if {$b > $cur} { set cur $b } }
      if {$cur < $rxh} { lappend surv [list $cur $rxh] }
      set wst [$sb getWireShapeType]
      foreach s $surv { lassign $s a b
        odb::dbSBox_create $swire $bpr $a $ryl $b $ryh $wst }
      odb::dbSBox_destroy $sb
      incr trimmed
    }
  }
}
puts ">>> trimmed $trimmed BPR rail segments at TSVs"

# 4) Delete LOCKED taps stranded in keepouts. They sit on the now-broken rails
#    (useless ties) and the legalizer can't move them; no std cell remains in the
#    keepout to need a well tie, so removing them is safe.
set dead {}
foreach inst [$block getInsts] {
  if {[[$inst getMaster] getName] ne "gt2_6t_tap_w31_lvt"} continue
  set ib [$inst getBBox]
  foreach k $keeps {
    lassign $k kx0 ky0 kx1 ky1
    if {[rov [$ib xMin] [$ib xMax] $kx0 $kx1] && [rov [$ib yMin] [$ib yMax] $ky0 $ky1]} {
      lappend dead $inst; break
    }
  }
}
foreach inst $dead { odb::dbInst_destroy $inst }
puts ">>> deleted [llength $dead] taps stranded in keepouts"

# 5) Relocate the displaced std cells (legalizer respects the blockages + FIRM TSVs).
detailed_placement

write_db results/gcd_postplace_B.odb
puts ">>> approach B done: results/gcd_postplace_B.odb"
exit 0
