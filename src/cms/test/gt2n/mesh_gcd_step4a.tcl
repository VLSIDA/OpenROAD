# Step 4a: build the clock mesh on BM1 (V) / BM2 (H). No routing yet.
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
puts ">>> clock = $clk ; mesh on BM2(H)/BM1(V)"

create_clock_mesh \
    -clock        $clk \
    -h_layer      BM2 \
    -v_layer      BM1 \
    -pitch        1.0 \
    -buffers      {gt2_6t_buf_x4_w31_lvt} \
    -cts_buffers  {gt2_6t_buf_x4_w31_lvt}

write_db  $rdir/gcd_step4a_mesh.odb
puts ">>> Step 4a done. Mesh built on BM1/BM2."
exit 0
