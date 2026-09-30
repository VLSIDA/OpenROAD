# Approach B1, refined: per fragment-row, remove the BPR rails CONTINUOUSLY
# across the span of that fragment's TSVs (one clean gap, not per-TSV), keeping
# BPR intact BETWEEN fragments (at the straps, where the via columns feed). The
# placement blockage covers the same continuous span so every std cell on a
# removed rail is relocated (no floating cells). TSVs come from the scan-real-PDN
# frozen grid (shifted off the straps).
set orfs /home/wali2/backside/OpenROAD-flow-scripts/flow
set res  $orfs/results/gt2n/gcd/base
set plat $orfs/platforms/gt2n

read_db  $res/3_place.odb
read_lef $plat/lef/gt2_6t_TSV.lef

# Place TSVs at the (scan-real-PDN) frozen grid; small per-TSV keepouts which we
# then replace with continuous per-fragment spans.
reserve_clock_mesh -h_layer BM2 -v_layer BM1 -pitch 1.0 \
    -strap_pitch 2.16 -strap_offset 1.08 -strap_width 0.36 \
    -tsv_master gt2_6t_TSV -spacing 0 -keepout_w 0.084 -keepout_h 0.144

set block [ord::get_db_block]
set tech  [ord::get_db_tech]
set bpr   [$tech findLayer BPR]
set dbu   [$block getDbUnitsPerMicron]
set halo  [expr {int(round(0.112 * $dbu))}]
proc rov {alo ahi blo bhi} { return [expr {$alo < $bhi && $blo < $ahi}] }

# Drop reserve's per-TSV keepouts; we rebuild continuous per-fragment ones.
foreach b [$block getBlockages]   { odb::dbBlockage_destroy $b }
foreach o [$block getObstructions] { odb::dbObstruction_destroy $o }

# REAL BM3 vertical strap x-bands (fragment boundaries on a row).
set bm3 {}
foreach net [$block getNets] {
  if {[$net getSigType] ne "POWER" && [$net getSigType] ne "GROUND"} continue
  foreach sw [$net getSWires] { foreach sb [$sw getWires] {
    if {[$sb isVia]} continue
    set L [$sb getTechLayer]; if {$L eq "NULL"} continue
    if {[$L getName] eq "BM3" && ([$sb yMax]-[$sb yMin])>([$sb xMax]-[$sb xMin])} {
      lappend bm3 [expr {([$sb xMin]+[$sb xMax])/2}]
    }
  }}
}

# Group TSVs by row.
array set byrow {}
foreach inst [$block getInsts] {
  if {[[$inst getMaster] getName] ne "gt2_6t_TSV"} continue
  set bb [$inst getBBox]
  lappend byrow([$bb yMin]) [list [$bb xMin] [$bb xMax] [$bb yMin] [$bb yMax]]
}

# Build continuous windows: per row, split TSVs into clusters wherever a BM3
# strap lies between consecutive TSVs (= a fragment boundary). One window per
# cluster, spanning first..last TSV (+halo), full cell height (both rails).
set windows {}
foreach y [array names byrow] {
  set ts [lsort -integer -index 0 $byrow($y)]
  set cl {}
  set prevmax {}
  foreach t $ts {
    lassign $t xl xh yl yh
    if {$prevmax ne {}} {
      set sep 0
      foreach c $bm3 { if {$c > $prevmax && $c < $xl} { set sep 1; break } }
      if {$sep} { lappend windows $cl; set cl {} }
    }
    lappend cl $t
    set prevmax $xh
  }
  if {[llength $cl]} { lappend windows $cl }
}
puts ">>> [llength $windows] continuous BPR-removal windows (per fragment-row)"

# For each window: placement blockage over the span + collect BPR cut rect.
set cuts {}
foreach cl $windows {
  set xl [lindex [lindex $cl 0] 0]
  set xh [lindex [lindex $cl end] 1]
  set yl [lindex [lindex $cl 0] 2]
  set yh [lindex [lindex $cl 0] 3]
  set wx0 [expr {$xl-$halo}]; set wx1 [expr {$xh+$halo}]
  odb::dbBlockage_create $block $wx0 $yl $wx1 $yh
  lappend cuts [list $wx0 $yl $wx1 $yh]
}

# Trim BPR rails over the cut windows (continuous spans).
set trimmed 0
foreach net [$block getNets] {
  if {[$net getSigType] ne "POWER" && [$net getSigType] ne "GROUND"} continue
  foreach swire [$net getSWires] {
    set boxes {}
    foreach sb [$swire getWires] { lappend boxes $sb }
    foreach sb $boxes {
      if {[$sb isVia]} continue
      if {[[$sb getTechLayer] getName] ne "BPR"} continue
      set rxl [$sb xMin]; set ryl [$sb yMin]; set rxh [$sb xMax]; set ryh [$sb yMax]
      set rem {}
      foreach c $cuts { lassign $c wxl wyl wxh wyh
        if {![rov $wyl $wyh $ryl $ryh]} continue
        set lo [expr {max($wxl,$rxl)}]; set hi [expr {min($wxh,$rxh)}]
        if {$lo < $hi} { lappend rem [list $lo $hi] } }
      if {![llength $rem]} continue
      set rem [lsort -integer -index 0 $rem]
      set merged {}
      foreach iv $rem { lassign $iv a b
        if {[llength $merged] && $a <= [lindex [lindex $merged end] 1]} {
          lset merged end 1 [expr {max([lindex [lindex $merged end] 1],$b)}]
        } else { lappend merged $iv } }
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
puts ">>> trimmed $trimmed BPR rails (continuous per-fragment spans)"

# Connectivity cleanup: a BPR segment with no via-column feed (no via overlaps
# it) is electrically floating -- power reaches BPR only via the column at a
# strap, and the alternating vdd/vss straps can leave an edge piece of one net
# with no column. Blockage over each floater (+ the rows it borders) so the
# cells that lost their only vdd/vss relocate, then delete the dead stub.
set allry {}
foreach r [$block getRows] { lappend allry [lindex [$r getOrigin] 1] }
set allry [lsort -integer -unique $allry]
set rh [expr {[llength $allry] >= 2 ? [lindex $allry 1]-[lindex $allry 0] : 288}]
set nfloat 0
foreach net [$block getNets] {
  if {[$net getSigType] ne "POWER" && [$net getSigType] ne "GROUND"} continue
  set vias {}
  foreach sw [$net getSWires] { foreach sb [$sw getWires] {
    if {[$sb isVia]} { lappend vias [list [$sb xMin] [$sb yMin] [$sb xMax] [$sb yMax]] } } }
  set floaters {}
  foreach sw [$net getSWires] { foreach sb [$sw getWires] {
    if {[$sb isVia]} continue
    if {[[$sb getTechLayer] getName] ne "BPR"} continue
    set fed 0
    foreach v $vias { lassign $v vx0 vy0 vx1 vy1
      if {[rov [$sb xMin] [$sb xMax] $vx0 $vx1] && [rov [$sb yMin] [$sb yMax] $vy0 $vy1]} { set fed 1; break } }
    if {!$fed} { lappend floaters $sb }
  } }
  foreach sb $floaters {
    odb::dbBlockage_create $block [$sb xMin] [expr {[$sb yMin]-$rh}] [$sb xMax] [expr {[$sb yMax]+$rh}]
    odb::dbSBox_destroy $sb
    incr nfloat
  }
}
puts ">>> removed $nfloat floating BPR stubs (+blockage so their cells relocate)"

# Delete taps stranded in the windows (on removed rails).
set dead {}
foreach inst [$block getInsts] {
  if {[[$inst getMaster] getName] ne "gt2_6t_tap_w31_lvt"} continue
  set ib [$inst getBBox]
  foreach c $cuts { lassign $c wx0 wy0 wx1 wy1
    if {[rov [$ib xMin] [$ib xMax] $wx0 $wx1] && [rov [$ib yMin] [$ib yMax] $wy0 $wy1]} {
      lappend dead $inst; break } }
}
foreach inst $dead { odb::dbInst_destroy $inst }
puts ">>> deleted [llength $dead] taps in removal spans"

detailed_placement
write_db results/gcd_postplace_B3.odb
puts ">>> B3 (continuous per-fragment) done"
exit 0
