# Diagnostic: frontside Pass 1 with NO TSVs (TSV LEF not loaded -> create_clock_mesh
# builds the PDN-aware deformed mesh but skips TSV insertion). Isolates whether
# the DRT track-assignment crash comes from the mesh geometry or the TSV cells.
set orfs    /home/wali2/backside/OpenROAD-flow-scripts/flow
set results $orfs/results/gt2n/gcd/base
set plat    $orfs/platforms/gt2n
set rdir    [file join [file dirname [file normalize [info script]]] results]
file mkdir $rdir

read_db      $results/3_place.odb
read_liberty $plat/lib/gt2_6t_w31_lvt_tt_0p7v25c.lib
read_sdc     $results/3_place.sdc
# NOTE: gt2_6t_TSV.lef intentionally NOT read -> no TSVs placed.

detailed_placement
source $plat/setRC.tcl
set clk ""
foreach c [sta::all_clocks] { set clk [get_name $c]; break }

create_clock_mesh -clock $clk -h_layer BM2 -v_layer BM1 -pitch 1.0 \
    -buffers {gt2_6t_buf_x4_w31_lvt} -cts_buffers {gt2_6t_buf_x4_w31_lvt}
detailed_placement -max_displacement 1000
setup_proxy_bterms -clock $clk -proxy_layer BM2

catch { set_dont_touch [get_nets -quiet $clk] }

set_routing_layers -signal M2-M5 -clock M3-M5
global_route   -guide_file $rdir/diag.guide -congestion_iterations 50
detailed_route -output_drc $rdir/diag_drc.rpt -droute_end_iter 0
write_db $rdir/gcd_diag_notsv.odb
puts ">>> diag (no TSV) frontside route done"
exit 0
