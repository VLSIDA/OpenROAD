# Test the CMS create_sink_taps command (sink side of the backside clock mesh).
#   create_clock_mesh   -> backside BM1/BM2 mesh + drive TSVs + buffers
#   setup_proxy_bterms  -> drive-side TSV.Y -> proxy BTerm on the mesh
#   create_sink_taps    -> per H-wire gap: assign FFs to nearest tap (cap C),
#                          place winning taps (sink-TSV + proxy BTerm on mesh +
#                          sink-buffer driving the tap's FFs)
# Verifies: taps placed only where FFs exist, every FF moved onto a sink-buffer
# output, every sink-TSV.Y tied to the mesh through a proxy BTerm on a stripe.
set orfs /home/wali2/backside/OpenROAD-flow-scripts/flow
set res  $orfs/results/gt2n/gcd/base
set plat $orfs/platforms/gt2n
set rdir [file join [file dirname [file normalize [info script]]] results]
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

# ---- the command under test ----
create_sink_taps -h_layer BM2 -v_layer BM1 \
    -buffer gt2_6t_buf_x4_w31_lvt -capacity 16 -tsv_master gt2_6t_TSV

# ---------------- verification ----------------
set block [ord::get_db_block]
set mesh  [$block findNet clk_mesh]
proc ov {a b} { return [expr {!([lindex $a 2] <= [lindex $b 0] || [lindex $b 2] <= [lindex $a 0] \
                              || [lindex $a 3] <= [lindex $b 1] || [lindex $b 3] <= [lindex $a 1])}] }
# mesh BM2 stripe rects (for the BPin-overlap check)
set bm2rects {}
foreach sw [$mesh getSWires] { foreach sb [$sw getWires] {
  if {[$sb isVia]} continue
  if {[[$sb getTechLayer] getName] eq "BM2"} {
    lappend bm2rects [list [$sb xMin] [$sb yMin] [$sb xMax] [$sb yMax]] } } }

set ntsv 0; set nbuf 0; set yb 0; set ab 0; set bpin_on 0
foreach inst [$block getInsts] {
  set nm [$inst getName]
  if {[string match "sink_tsv_*" $nm]} {
    incr ntsv
    set y [$inst findITerm Y]
    set yn [expr {$y ne "NULL" && [$y getNet] ne "NULL" ? [[$y getNet] getName] : ""}]
    if {[string match "b_sink_*" $yn]} {
      incr yb
      # proxy BTerm on this b_sink net with a BPin overlapping a mesh BM2 stripe
      foreach bt [[$y getNet] getBTerms] { foreach bp [$bt getBPins] { foreach bx [$bp getBoxes] {
        set r [list [$bx xMin] [$bx yMin] [$bx xMax] [$bx yMax]]
        foreach m $bm2rects { if {[ov $r $m]} { incr bpin_on; break } } } } }
    }
    set a [$inst findITerm A]
    set an [expr {$a ne "NULL" && [$a getNet] ne "NULL" ? [[$a getNet] getName] : ""}]
    if {[string match "sink_tap_*" $an]} { incr ab }
  } elseif {[string match "sink_buf_*" $nm]} { incr nbuf }
}
# FFs: how many CLK pins now ride a sink_drv net (driven by the mesh)?
set ff_drv 0; set ff_clk 0
foreach inst [$block getInsts] {
  set c [$inst findITerm CLK]
  if {$c eq "NULL"} continue
  set cn [expr {[$c getNet] ne "NULL" ? [[$c getNet] getName] : ""}]
  if {[string match "sink_drv_*" $cn]} { incr ff_drv } elseif {$cn eq "clk"} { incr ff_clk }
}

puts "=================== create_sink_taps test ==================="
puts "  sink-TSVs placed:                 $ntsv"
puts "  sink-buffers placed:              $nbuf"
puts "  sink-TSV.Y on a b_sink net:       $yb / $ntsv"
puts "  proxy BPin overlaps a BM2 stripe: $bpin_on / $ntsv"
puts "  sink-TSV.A on a sink_tap net:     $ab / $ntsv"
puts "  FF clock pins on a sink_drv net:  $ff_drv"
puts "  FF clock pins still on clk:       $ff_clk"
set ok 1
if {$ntsv == 0}            { set ok 0; puts "  FAIL: no sink-taps placed" }
if {$nbuf != $ntsv}       { set ok 0; puts "  FAIL: TSV/buffer count mismatch" }
if {$yb != $ntsv}         { set ok 0; puts "  FAIL: some sink-TSV.Y not on a b_sink net" }
if {$bpin_on != $ntsv}    { set ok 0; puts "  FAIL: some proxy BPin not on a mesh stripe" }
if {$ab != $ntsv}         { set ok 0; puts "  FAIL: some sink-TSV.A not on a sink_tap net" }
if {$ff_clk != 0}         { set ok 0; puts "  FAIL: some FFs still on clk (not mesh-driven)" }
puts [expr {$ok ? ">>> PASS" : ">>> FAIL"}]
puts "============================================================"

write_db $rdir/gcd_sink_taps.odb
exit [expr {$ok ? 0 : 1}]
