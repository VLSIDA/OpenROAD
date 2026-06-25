# Pass 1 (frontside): build the clock mesh + buffers, then route the clock TREE
# and all signals on the frontside (M2..M5). The backside "crossing" stubs
# (mesh-buffer outputs + sinks) are masked (special) so GRT/DRT skip them here.
# Saves gcd_4c_frontside.odb for pass 2.
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

proc is_stub {nm} { return [expr {[string match "sink_*" $nm] || [string match "*_buf_*" $nm]}] }
set b [ord::get_db_block]
foreach n [$b getNets] { if {[is_stub [$n getName]]} { $n setSpecial } }

set_routing_layers -signal M2-M5 -clock M3-M5
global_route   -guide_file $rdir/4c_p1.guide -congestion_iterations 50
detailed_route -output_drc $rdir/4c_p1_drc.rpt -droute_end_iter 0
write_db $rdir/gcd_4c_frontside.odb
puts ">>> PASS1 done: gcd_4c_frontside.odb"
exit 0
