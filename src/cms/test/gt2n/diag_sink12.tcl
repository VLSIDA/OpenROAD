# Diagnose why tap_sink_12 and b_sink_bterm_12 end up far apart.
set orfs    /home/wali2/backside/OpenROAD-flow-scripts/flow
set results $orfs/results/gt2n/gcd/base
set plat    $orfs/platforms/gt2n
read_db      $results/3_place.odb
read_liberty $plat/lib/gt2_6t_w31_lvt_tt_0p7v25c.lib
read_liberty $plat/lib/gt2_ntsv_tap.lib
read_sdc     $results/3_place.sdc
detailed_placement
source $plat/setRC.tcl
set clk ""; foreach c [sta::all_clocks] { set clk [get_name $c]; break }
create_clock_mesh -clock $clk -h_layer BM2 -v_layer BM1 -pitch 1.0 \
    -buffers {gt2_6t_buf_x4_w31_lvt} -cts_buffers {gt2_6t_buf_x4_w31_lvt}
detailed_placement -max_displacement 1000
setup_proxy_bterms    -clock $clk -proxy_layer M2
connect_sinks_to_mesh -clock $clk -proxy_layer M2
puts "=== BEFORE final legalize ==="
proc dump {tag} {
  set b [ord::get_db_block]
  set t [$b findInst "ntsv_tap_sink_12"]
  if {$t ne "NULL"} { set bb [$t getBBox]; puts "$tag tap_sink_12 @([$bb xMin],[$bb yMin])-([$bb xMax],[$bb yMax])" }
  set n [$b findNet "b_sink_12"]
  if {$n ne "NULL"} {
    foreach bt [$n getBTerms] { foreach bp [$bt getBPins] { foreach box [$bp getBoxes] {
      puts "$tag b_sink_12 BTerm @([$box xMin],[$box yMin])-([$box xMax],[$box yMax]) layer=[[$box getTechLayer] getName]" }}}
    foreach it [$n getITerms] { set i [$it getInst]; set ib [$i getBBox]
      puts "$tag b_sink_12 iterm [$i getName]/[[$it getMTerm] getName] @([$ib xMin],[$ib yMin])" }
  }
}
dump "BEFORE"
detailed_placement -max_displacement 1000
puts "=== AFTER final legalize ==="
dump "AFTER"
exit 0
