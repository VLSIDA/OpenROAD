# Single-pass backside clock: tree on TOP, mesh on backside BM1/BM2, crossing
# in one detailed_route. No masking / no two passes needed:
#  - clock range BM2..M5 (set once) -> DRT floor = BM2 (VSD has the nTSV via, so
#    no DRT-0233; VG drops out as the redundant BPR<->M0 cut).
#  - getNetLayerRange clamps min to each net's lowest pin: frontside tree nets
#    (pins on M0) get range [M0,M5] -> stay on top; crossing stubs (a backside
#    BTerm on BM2) get [BM2,M5] -> span M0 -nTSV- BPR -BV0- BM1 -BV1- BM2.
#  - the DRT maze confines signals + frontside tree to the frontside and leaves
#    only the backside-crossing clock nets unrestricted (FlexDR_maze).
set orfs    /home/wali2/backside/OpenROAD-flow-scripts/flow
set results $orfs/results/gt2n/gcd/base
set plat    $orfs/platforms/gt2n
set rdir    [file join [file dirname [file normalize [info script]]] results]
file mkdir $rdir

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

set_routing_layers -signal M2-M5 -clock BM2-M5
global_route   -guide_file $rdir/single.guide -congestion_iterations 50
detailed_route -output_drc $rdir/single_drc.rpt -droute_end_iter 0
write_db $rdir/gcd_single.odb
puts ">>> single-pass done: gcd_single.odb"
exit 0
