# Approach B5: per-TSV BPR break (rail stays intact between TSVs, so adjacent
# rows keep power there) + relocate the 3-row band (TSV row + row above + row
# below) at each TSV, since the TSV's two rails are shared with those adjacent
# rows. Plus floating-stub cleanup. TSVs from the scan-real-PDN frozen grid.
set orfs /home/wali2/backside/OpenROAD-flow-scripts/flow
set res  $orfs/results/gt2n/gcd/base
set plat $orfs/platforms/gt2n

read_db  $res/3_place.odb
read_lef $plat/lef/gt2_6t_TSV.lef

reserve_clock_mesh -h_layer BM2 -v_layer BM1 -pitch 1.0 \
    -strap_pitch 2.16 -strap_offset 1.08 -strap_width 0.36 \
    -tsv_master gt2_6t_TSV -spacing 0 -keepout_w 0.084 -keepout_h 0.144

set block [ord::get_db_block]
set tech  [ord::get_db_tech]
set bpr   [$tech findLayer BPR]
set dbu   [$block getDbUnitsPerMicron]
set halo  [expr {int(round(0.112 * $dbu))}]
proc rov {alo ahi blo bhi} { return [expr {$alo < $bhi && $blo < $ahi}] }

foreach b [$block getBlockages]   { odb::dbBlockage_destroy $b }
foreach o [$block getObstructions] { odb::dbObstruction_destroy $o }

# row height
set allry {}
foreach r [$block getRows] { lappend allry [lindex [$r getOrigin] 1] }
set allry [lsort -integer -unique $allry]
set rh [expr {[llength $allry] >= 2 ? [lindex $allry 1]-[lindex $allry 0] : 288}]

# Per-TSV cut window + 3-row placement blockage (TSV row +/- 1 row).
set cuts {}
foreach inst [$block getInsts] {
  if {[[$inst getMaster] getName] ne "gt2_6t_TSV"} continue
  set bb [$inst getBBox]
  set wx0 [expr {[$bb xMin]-$halo}]; set wx1 [expr {[$bb xMax]+$halo}]
  lappend cuts [list $wx0 [$bb yMin] $wx1 [$bb yMax]]
  # blockage spans the row below + TSV row + row above so adjacent stranded
  # cells (they share the broken boundary rails) relocate too.
  odb::dbBlockage_create $block $wx0 [expr {[$bb yMin]-$rh}] $wx1 [expr {[$bb yMax]+$rh}]
}
puts ">>> [llength $cuts] per-TSV cut windows (+3-row blockages)"

# Trim BPR over the per-TSV windows.
set trimmed 0
foreach net [$block getNets] {
  if {[$net getSigType] ne "POWER" && [$net getSigType] ne "GROUND"} continue
  foreach swire [$net getSWires] {
    set boxes {}; foreach sb [$swire getWires] { lappend boxes $sb }
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
puts ">>> trimmed $trimmed BPR rails (per-TSV)"

# Floating-stub cleanup (segments with no via-column feed).
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
    if {!$fed} { lappend floaters $sb } } }
  foreach sb $floaters {
    odb::dbBlockage_create $block [$sb xMin] [expr {[$sb yMin]-$rh}] [$sb xMax] [expr {[$sb yMax]+$rh}]
    odb::dbSBox_destroy $sb; incr nfloat }
}
puts ">>> removed $nfloat floating BPR stubs"

# Delete taps in blockages, then relocate everything.
set dead {}
foreach inst [$block getInsts] {
  if {[[$inst getMaster] getName] ne "gt2_6t_tap_w31_lvt"} continue
  set ib [$inst getBBox]
  foreach b [$block getBlockages] { set bb [$b getBBox]
    if {[rov [$ib xMin] [$ib xMax] [$bb xMin] [$bb xMax]] && [rov [$ib yMin] [$ib yMax] [$bb yMin] [$bb yMax]]} { lappend dead $inst; break } }
}
foreach inst $dead { odb::dbInst_destroy $inst }
puts ">>> deleted [llength $dead] taps in blockages"

detailed_placement
write_db results/gcd_postplace_B5.odb
puts ">>> B5 (per-TSV + 3-row relocate) done"
exit 0
