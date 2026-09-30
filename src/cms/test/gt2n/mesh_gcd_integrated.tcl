# INTEGRATED flow: one odb with mesh + TSVs + BPR break + relocation + routing.
#   create_clock_mesh (mesh + TSVs + buffers)
#   setup_proxy_bterms (TSV.Y -> mesh net directly; no proxy BTerm -> no shorts)
#   B5 surgery: per-TSV BPR break + 3-row relocate + floating-stub cleanup + tap del
#   detailed_placement (relocate stranded std cells/buffers)
#   global + detailed route
set orfs    /home/wali2/backside/OpenROAD-flow-scripts/flow
set res     $orfs/results/gt2n/gcd/base
set plat    $orfs/platforms/gt2n
set rdir    [file join [file dirname [file normalize [info script]]] results]
file mkdir $rdir

read_db      $res/3_place.odb
read_liberty $plat/lib/gt2_6t_w31_lvt_tt_0p7v25c.lib
read_sdc     $res/3_place.sdc
read_lef     $plat/lef/gt2_6t_TSV.lef
read_liberty $plat/lib/gt2_6t_TSV.lib
read_lef     $plat/lef/gt2_ntsv_via.lef

detailed_placement
source $plat/setRC.tcl
set clk ""
foreach c [sta::all_clocks] { set clk [get_name $c]; break }

create_clock_mesh -clock $clk -h_layer BM2 -v_layer BM1 -pitch 1.0 \
    -buffers {gt2_6t_buf_x4_w31_lvt} -cts_buffers {gt2_6t_buf_x4_w31_lvt}
detailed_placement -max_displacement 1000
setup_proxy_bterms -clock $clk -proxy_layer BM2

# ---------------- B5 surgery: BPR break + relocate ----------------
set block [ord::get_db_block]
set tech  [ord::get_db_tech]
set bpr   [$tech findLayer BPR]
set dbu   [$block getDbUnitsPerMicron]
set halo  [expr {int(round(0.112 * $dbu))}]
proc rov {alo ahi blo bhi} { return [expr {$alo < $bhi && $blo < $ahi}] }
set allry {}
foreach r [$block getRows] { lappend allry [lindex [$r getOrigin] 1] }
set allry [lsort -integer -unique $allry]
set rh [expr {[llength $allry] >= 2 ? [lindex $allry 1]-[lindex $allry 0] : 288}]

# per-TSV cut window + 3-row placement blockage
set cuts {}
foreach inst [$block getInsts] {
  if {[[$inst getMaster] getName] ne "gt2_6t_TSV"} continue
  set bb [$inst getBBox]
  set wx0 [expr {[$bb xMin]-$halo}]; set wx1 [expr {[$bb xMax]+$halo}]
  lappend cuts [list $wx0 [$bb yMin] $wx1 [$bb yMax]]
  odb::dbBlockage_create $block $wx0 [expr {[$bb yMin]-$rh}] $wx1 [expr {[$bb yMax]+$rh}]
}
puts ">>> [llength $cuts] drive-TSV cut windows"

# ===================== SINK TAPS =====================
# One sink-tap per gap between adjacent V mesh wires, on each H mesh wire:
# sink-TSV on the H-wire midpoint (Y->clk_mesh, router vias BM1->BM2) + a
# sink-buffer beside it on intact BPR. Keepout over the strip; BPR broken only
# at the TSV footprint (folded into the same trim via $cuts).
set db   [ord::get_db]
set mesh [$block findNet clk_mesh]
set vxs {}; set hsegs {}
foreach sw [$mesh getSWires] { foreach sb [$sw getWires] {
  if {[$sb isVia]} continue
  set ln [[$sb getTechLayer] getName]
  set ddx [expr {[$sb xMax]-[$sb xMin]}]; set ddy [expr {[$sb yMax]-[$sb yMin]}]
  if {$ln eq "BM1" && $ddy > $ddx} { lappend vxs [expr {([$sb xMin]+[$sb xMax])/2}] }
  if {$ln eq "BM2" && $ddx > $ddy} { lappend hsegs [list [expr {([$sb yMin]+[$sb yMax])/2}] [$sb xMin] [$sb xMax]] }
}}
set vxs [lsort -integer -unique $vxs]
set tsvm [$db findMaster gt2_6t_TSV]
set bufm [$db findMaster gt2_6t_buf_x4_w31_lvt]
set tw [$tsvm getWidth]; set bufw [$bufm getWidth]
set bm2 [$tech findLayer BM2]; set bhw [expr {[$bm2 getWidth]/2}]
set ymt [$tsvm findMTerm Y]; set ypoff [expr {$tw/2}]
if {$ymt ne "NULL"} { set yb [$ymt getBBox]; set ypoff [expr {([$yb xMin]+[$yb xMax])/2}] }
set sitew $tw
foreach r [$block getRows] { set st [$r getSite]; if {$st ne "NULL"} { set sitew [$st getWidth]; break } }
set cxmin [[$block getCoreArea] xMin]
set rowsy {}
foreach r [$block getRows] { lappend rowsy [list [lindex [$r getOrigin] 1] [$r getOrient]] }
proc near_rowy {rowsy y} { set bd 1000000000; set best {}
  foreach e $rowsy { lassign $e yy oo; set d [expr {abs($yy-$y)}]; if {$d<$bd} {set bd $d; set best $e} }
  return $best }
# potential tap positions {mx ry ro}
set taps {}
foreach hs $hsegs {
  lassign $hs hy xlo xhi
  set inseg {}
  foreach vx $vxs { if {$vx >= $xlo && $vx <= $xhi} { lappend inseg $vx } }
  for {set i 1} {$i < [llength $inseg]} {incr i} {
    set mx [expr {([lindex $inseg [expr {$i-1}]]+[lindex $inseg $i])/2}]
    lassign [near_rowy $rowsy $hy] ry ro
    lappend taps [list $mx $ry $ro $hy]
  }
}
# FFs (CLK pins) + positions
set sinks {}
foreach inst [$block getInsts] {
  set c [$inst findITerm CLK]; if {$c eq "NULL"} continue
  set bb [$c getBBox]
  lappend sinks [list [expr {([$bb xMin]+[$bb xMax])/2}] [expr {([$bb yMin]+[$bb yMax])/2}] $c]
}
# assign each FF to its nearest tap with load < C (spill); record per-tap FFs
set Ccap 16
array set tapff {}; array set tapload {}
for {set t 0} {$t < [llength $taps]} {incr t} { set tapload($t) 0 }
foreach s $sinks {
  lassign $s sx sy sit
  set cand {}
  for {set t 0} {$t < [llength $taps]} {incr t} {
    lassign [lindex $taps $t] tx ty to
    lappend cand [list [expr {abs($sx-$tx)+abs($sy-$ty)}] $t]
  }
  foreach c [lsort -integer -index 0 $cand] {
    lassign $c d t
    if {$tapload($t) < $Ccap} { lappend tapff($t) $sit; incr tapload($t); break }
  }
}
# place ONLY taps that got >=1 FF; TSV gets the keepout/BPR-break, buffer goes
# OUTSIDE it as a movable cell (detailed_placement legalizes it on a powered row)
set sn 0
for {set t 0} {$t < [llength $taps]} {incr t} {
  if {![info exists tapff($t)]} continue
  lassign [lindex $taps $t] mx ry ro hy
  set ox [expr {$cxmin + (($mx-$ypoff-$cxmin+$sitew/2)/$sitew)*$sitew}]
  set ti [odb::dbInst_create $block $tsvm "sink_tsv_$sn"]
  if {$ti eq "NULL"} continue
  $ti setOrient $ro; $ti setLocation $ox $ry; $ti setPlacementStatus FIRM
  set bi [odb::dbInst_create $block $bufm "sink_buf_$sn"]
  $bi setOrient $ro; $bi setLocation [expr {$ox + $tw + 2*$halo + $sitew}] $ry; $bi setPlacementStatus PLACED
  # Mirror the drive side: TSV.Y on a SEPARATE backside net + a proxy BTerm whose
  # BPin sits on the mesh H-wire at the tap (mx,hy). The router routes the 2-pin
  # Y->BTerm net; the BPin overlaps the clk_mesh stripe = the physical tie to grid.
  set bnet [odb::dbNet_create $block "b_sink_$sn"]; $bnet setSigType CLOCK
  set yit [$ti findITerm Y]; if {$yit ne "NULL"} { $yit connect $bnet }
  set bt [odb::dbBTerm_create $bnet "b_sink_$sn"]; $bt setIoType INPUT; $bt setSigType CLOCK
  set bp [odb::dbBPin_create $bt]
  odb::dbBox_create $bp $bm2 [expr {$mx-$bhw}] [expr {$hy-$bhw}] [expr {$mx+$bhw}] [expr {$hy+$bhw}]
  $bp setPlacementStatus PLACED
  set tapn [odb::dbNet_create $block "sink_tap_$sn"]; $tapn setSigType CLOCK
  set ait [$ti findITerm A]; if {$ait ne "NULL"} { $ait connect $tapn }
  set bin [$bi findITerm A]; if {$bin ne "NULL"} { $bin connect $tapn }
  set drvn [odb::dbNet_create $block "sink_drv_$sn"]; $drvn setSigType CLOCK
  set bout [$bi findITerm Y]; if {$bout ne "NULL"} { $bout connect $drvn }
  foreach ff $tapff($t) { $ff disconnect; $ff connect $drvn }
  # keepout + BPR cut at the TSV footprint ONLY (buffer is outside it)
  set bb [$ti getBBox]
  lappend cuts [list [expr {[$bb xMin]-$halo}] [$bb yMin] [expr {[$bb xMax]+$halo}] [$bb yMax]]
  odb::dbBlockage_create $block [expr {[$bb xMin]-$halo}] [expr {[$bb yMin]-$rh}] [expr {[$bb xMax]+$halo}] [expr {[$bb yMax]+$rh}]
  incr sn
}
puts ">>> placed $sn used sink-taps (of [llength $taps] possible), [llength $sinks] FFs assigned"

proc trim_bpr {block bpr cuts} {
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
  return $trimmed
}
puts ">>> trimmed [trim_bpr $block $bpr $cuts] BPR rails"

# floating-stub cleanup
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
    odb::dbSBox_destroy $sb; incr nfloat } }
puts ">>> removed $nfloat floating BPR stubs"

# delete taps in blockages
set dead {}
foreach inst [$block getInsts] {
  if {[[$inst getMaster] getName] ne "gt2_6t_tapbspdn_w31_lvt"} continue
  set ib [$inst getBBox]
  foreach b [$block getBlockages] { set bb [$b getBBox]
    if {[rov [$ib xMin] [$ib xMax] [$bb xMin] [$bb xMax]] && [rov [$ib yMin] [$ib yMax] [$bb yMin] [$bb yMax]]} { lappend dead $inst; break } } }
foreach inst $dead { odb::dbInst_destroy $inst }
puts ">>> deleted [llength $dead] taps in blockages"

detailed_placement -max_displacement 1000

# (sink assignment is folded into the sink-tap placement above: only taps with
#  >=1 assigned FF are placed, and each FF is already on its sink_drv net)

# ---------------- route ----------------
# Keep the signal router OFF BPR (the power followpin layer). BPR sits inside
# the BM2-M5 clock range, so without this the router uses it and shorts to the
# power rails. The clock crosses front<->back INSIDE the TSV cell, so the router
# never needs BPR. A signal routing obstruction over the core blocks it (the
# PDN followpins are special wires, unaffected).
set core_r [$block getCoreArea]
set bpr_obs [odb::dbObstruction_create $block $bpr [$core_r xMin] [$core_r yMin] [$core_r xMax] [$core_r yMax]]
puts ">>> blocked signal routing on BPR over the core (temporary)"
set_routing_layers -signal M2-M5 -clock BM2-M5
global_route   -guide_file $rdir/integ.guide -congestion_iterations 50
detailed_route -output_drc $rdir/integ_drc.rpt -droute_end_iter 0

# The BPR block was only to steer the router; remove it so the saved odb has BPR
# back to normal (no sky-blue obstruction covering the core in the GUI).
odb::dbObstruction_destroy $bpr_obs
puts ">>> removed temporary BPR routing obstruction (BPR back to normal)"

write_db $rdir/gcd_integrated.odb
puts ">>> integrated flow done: gcd_integrated.odb"
exit 0
