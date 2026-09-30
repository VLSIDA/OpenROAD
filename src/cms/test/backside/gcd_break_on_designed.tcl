# ORPHAN-BPR test on the co-planar designed PDN: TSV at every intersection,
# break the BPR, and measure the orphan cleanup (floating rail pieces = BPR
# segments left with no BV0 feed via after the breaks).
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
detailed_placement -max_displacement 1000
setup_proxy_bterms -clock $clk -proxy_layer BM2
create_sink_taps -h_layer BM2 -v_layer BM1 \
    -buffer gt2_6t_buf_x4_w31_lvt -capacity 16 -tsv_master gt2_6t_TSV

# BPR rail length BEFORE the break
set block [ord::get_db_block]
proc bpr_len {block} {
  set L 0
  foreach nn {vdd vss} {
    foreach sw [[$block findNet $nn] getSWires] { foreach s [$sw getWires] {
      if {[$s isVia]} continue
      if {[[$s getTechLayer] getName] ne "BPR"} continue
      incr L [expr {[$s xMax]-[$s xMin]}] } } }
  return $L
}
set dbu [$block getDbUnitsPerMicron]
set len0 [bpr_len $block]

break_bpr_at_tsvs -halo 0.112 -relocate_rows 1
detailed_placement -max_displacement 1000

set len1 [bpr_len $block]
# blockage area (the dead placement zones)
set barea 0
foreach b [$block getBlockages] { set bb [$b getBBox]
  set barea [expr {$barea + ([$bb xMax]-[$bb xMin])*1.0*([$bb yMax]-[$bb yMin])}] }
set core [$block getCoreArea]
set carea [expr {([$core xMax]-[$core xMin])*1.0*([$core yMax]-[$core yMin])}]

puts "============ orphan-BPR result ============"
puts "  BPR rail length before: [format %.1f [expr {$len0/double($dbu)}]] um"
puts "  BPR rail length after:  [format %.1f [expr {$len1/double($dbu)}]] um  ([format %.1f [expr {100.0*$len1/$len0}]]% kept)"
puts "  blockage (dead) area:   [format %.1f [expr {$barea/double($dbu)/double($dbu)}]] um2 = [format %.1f [expr {100.0*$barea/$carea}]]% of core"
write_db $rdir/gcd_break_on_designed.odb
puts ">>> saved: $rdir/gcd_break_on_designed.odb"
exit 0
