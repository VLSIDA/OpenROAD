# NEW STRATEGY test: clock mesh on the DESIGNED co-planar BM1/BM2 PDN.
#   1. V clock wires SHIFT clear of the BM1 power straps (same layer)
#   2. H clock wires SHIFT clear of the BM2 power straps (same layer) -- no
#      notching (no via columns punch through the mesh layers: only BV0@BPR
#      and BV1 inside power intersections)
#   3. post-shift spacing check (same-metal distance, no interference)
# Expected: ONE connected mesh (no fragments), zero same-layer overlaps.
set orfs /home/wali2/backside/OpenROAD-flow-scripts/flow
set res  $orfs/results/gt2n/gcd/base
set plat $orfs/platforms/gt2n
set rdir [file join [file dirname [file normalize [info script]]] results]

read_db      $rdir/gcd_pdn_designed.odb
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

# ---------------- verification ----------------
set block [ord::get_db_block]
set mesh  [$block findNet clk_mesh]
proc rov {alo ahi blo bhi} { return [expr {$alo < $bhi && $blo < $ahi}] }

# power strap rects per mesh layer
array set pwr {BM1 {} BM2 {}}
foreach nn {vdd vss} {
  foreach sw [[$block findNet $nn] getSWires] { foreach s [$sw getWires] {
    if {[$s isVia]} continue
    set L [[$s getTechLayer] getName]
    if {$L eq "BM1" || $L eq "BM2"} {
      lappend pwr($L) [list [$s xMin] [$s yMin] [$s xMax] [$s yMax]] } } }
}
# clock wires: same-layer overlap + min same-layer gap to power
set nclk 0; set overlaps 0; set mingap 999999999
foreach sw [$mesh getSWires] { foreach s [$sw getWires] {
  if {[$s isVia]} continue
  set L [[$s getTechLayer] getName]
  if {$L ne "BM1" && $L ne "BM2"} continue
  incr nclk
  foreach p $pwr($L) { lassign $p px0 py0 px1 py1
    if {[rov [$s xMin] [$s xMax] $px0 $px1] && [rov [$s yMin] [$s yMax] $py0 $py1]} {
      incr overlaps
    } else {
      # same-direction gap (V: x-gap, H: y-gap) when spans overlap in the other axis
      if {$L eq "BM1" && [rov [$s yMin] [$s yMax] $py0 $py1]} {
        set g [expr {max($px0-[$s xMax], [$s xMin]-$px1)}]
        if {$g >= 0 && $g < $mingap} { set mingap $g }
      } elseif {$L eq "BM2" && [rov [$s xMin] [$s xMax] $px0 $px1]} {
        set g [expr {max($py0-[$s yMax], [$s yMin]-$py1)}]
        if {$g >= 0 && $g < $mingap} { set mingap $g }
      }
    }
  }
}}
set dbu [$block getDbUnitsPerMicron]
puts "============ mesh-on-designed-PDN check ============"
puts "  clock mesh wires (BM1/BM2):        $nclk"
puts "  same-layer clock/power OVERLAPS:   $overlaps   (must be 0)"
puts "  min same-layer clock-power gap:    [format %.3f [expr {$mingap/double($dbu)}]] um  (required >= 0.112 = 2x0.056)"
set ok [expr {$nclk > 0 && $overlaps == 0 && $mingap >= int(round(0.112*$dbu))}]
puts [expr {$ok ? ">>> PASS" : ">>> FAIL"}]
write_db $rdir/gcd_mesh_on_designed.odb
puts ">>> saved: $rdir/gcd_mesh_on_designed.odb"
exit [expr {$ok ? 0 : 1}]
