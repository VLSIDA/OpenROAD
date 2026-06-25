set orfs    /home/wali2/backside/OpenROAD-flow-scripts/flow
set results $orfs/results/gt2n/gcd/base
set plat    $orfs/platforms/gt2n
read_db      $results/3_place.odb
read_liberty $plat/lib/gt2_6t_w31_lvt_tt_0p7v25c.lib
read_sdc     $results/3_place.sdc
detailed_placement
source $plat/setRC.tcl
set clk ""
foreach c [sta::all_clocks] { set clk [get_name $c]; break }
create_clock_mesh -clock $clk -h_layer BM2 -v_layer BM1 -pitch 1.0 \
    -buffers {gt2_6t_buf_x4_w31_lvt} -cts_buffers {gt2_6t_buf_x4_w31_lvt}
detailed_placement -max_displacement 1000
setup_proxy_bterms    -clock $clk -proxy_layer BM2
connect_sinks_to_mesh -clock $clk -proxy_layer BM2

set b [ord::get_db_block]
puts "=== CLOCK nets: name | #iterms | #bterms | backside_bterm? | special? ==="
foreach n [$b getNets] {
  if {[$n getSigType] ne "CLOCK"} continue
  set nb 0
  foreach bt [$n getBTerms] {
    foreach bp [$bt getBPins] {
      foreach box [$bp getBoxes] {
        set ly [$box getTechLayer]
        if {$ly ne "NULL" && [$ly isBackside]} { set nb 1 }
      }
    }
  }
  puts [format "%-30s it=%-3d bt=%-3d backside=%d special=%d" \
        [$n getName] [llength [$n getITerms]] [llength [$n getBTerms]] $nb [$n isSpecial]]
}
exit 0
