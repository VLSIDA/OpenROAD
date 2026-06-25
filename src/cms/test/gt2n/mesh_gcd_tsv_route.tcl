# Single-pass route of the backside clock mesh + TSV crossings (SINKS SKIPPED).
#
# Why single-pass (not front/back split): the proxy BTerms sit on BM2 (backside),
# so the crossing nets have backside pins. DRT track assignment crashes if such a
# net is in a frontside-only layer range, and set_dont_touch does NOT exclude a
# net from the router. So we use one full range BM2-M5 -- the BM pins then have
# tracks. That range spans the VSD device cut, so the nTSV via def is required
# (DRT-0233); we inject it after read_db since 3_place.odb's baked tech lacks it.
set orfs    /home/wali2/backside/OpenROAD-flow-scripts/flow
set results $orfs/results/gt2n/gcd/base
set plat    $orfs/platforms/gt2n
set rdir    [file join [file dirname [file normalize [info script]]] results]
file mkdir $rdir

read_db      $results/3_place.odb
read_liberty $plat/lib/gt2_6t_w31_lvt_tt_0p7v25c.lib
read_sdc     $results/3_place.sdc
read_lef     $plat/lef/gt2_6t_TSV.lef
read_liberty $plat/lib/gt2_6t_TSV.lib
read_lef     $plat/lef/gt2_ntsv_via.lef   ;# nTSV via -> satisfies DRT-0233 on VSD

detailed_placement
source $plat/setRC.tcl
set clk ""
foreach c [sta::all_clocks] { set clk [get_name $c]; break }
puts ">>> clock: $clk"

create_clock_mesh -clock $clk -h_layer BM2 -v_layer BM1 -pitch 1.0 \
    -buffers {gt2_6t_buf_x4_w31_lvt} -cts_buffers {gt2_6t_buf_x4_w31_lvt}
detailed_placement -max_displacement 1000
setup_proxy_bterms -clock $clk -proxy_layer BM2
# connect_sinks_to_mesh  -- SKIPPED on purpose

set_routing_layers -signal M2-M5 -clock BM2-M5
global_route   -guide_file $rdir/tsv_route.guide -congestion_iterations 50
detailed_route -output_drc $rdir/tsv_route_drc.rpt -droute_end_iter 0
write_db $rdir/gcd_tsv_routed.odb
puts ">>> routed (single-pass, sinks skipped): gcd_tsv_routed.odb"
exit 0
