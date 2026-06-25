# Tap-cell CMS flow: buffers + buffer-taps placed together, sinks get taps,
# nets split front (clk_buf_*/sink_*) / back (b_clk_buf_*/b_sink_*), legalize.
set orfs    /home/wali2/backside/OpenROAD-flow-scripts/flow
set results $orfs/results/gt2n/gcd/base
set plat    $orfs/platforms/gt2n
set rdir    [file join [file dirname [file normalize [info script]]] results]
file mkdir $rdir

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
detailed_placement -max_displacement 1000
check_placement -verbose
write_db $rdir/gcd_tap.odb

# ---- verify ----
set b [ord::get_db_block]
set ntap 0; set nbuf 0; set nsink 0
foreach inst [$b getInsts] {
  set mn [[$inst getMaster] getName]
  if {$mn eq "nTSV_tap"} { incr ntap }
}
foreach net [$b getNets] {
  set nm [$net getName]
  if {[string match "*_buf_*" $nm] && ![string match "b_*" $nm]} { incr nbuf }
  if {[string match "sink_*" $nm]} { incr nsink }
}
puts ">>> taps placed: $ntap | clk_buf_* nets: $nbuf | sink_* nets: $nsink"

# show one buffer crossing + one sink crossing net membership
foreach probe {clk_buf b_clk_buf sink_ b_sink_} {
  foreach net [$b getNets] {
    set nm [$net getName]
    if {[string match "${probe}*" $nm]} {
      puts "    net $nm : iterms=[llength [$net getITerms]] bterms=[llength [$net getBTerms]]"
      break
    }
  }
}
puts ">>> tap flow done: gcd_tap.odb"
exit 0
