# Stage 1: place mesh buffers + gt2_6t_TSV crossing cells and wire them up.
#   buffer output -> TSV.A  (frontside net clk_buf_x_y)
#   TSV.Y -> proxy BTerm     (backside net b_clk_buf_x_y, on the mesh)
# Stops after setup_proxy_bterms and saves the ODB. NO sink->mesh, NO routing,
# NO PDN break yet (those come later).
set orfs    /home/wali2/backside/OpenROAD-flow-scripts/flow
set results $orfs/results/gt2n/gcd/base
set plat    $orfs/platforms/gt2n
set rdir    [file join [file dirname [file normalize [info script]]] results]
file mkdir $rdir

read_db      $results/3_place.odb
read_liberty $plat/lib/gt2_6t_w31_lvt_tt_0p7v25c.lib
read_sdc     $results/3_place.sdc

# gt2_6t_TSV collateral (the placed odb predates this cell, so add it now).
read_lef     $plat/lef/gt2_6t_TSV.lef
read_liberty $plat/lib/gt2_6t_TSV.lib

detailed_placement
source $plat/setRC.tcl

set clk ""
foreach c [sta::all_clocks] { set clk [get_name $c]; break }
puts ">>> clock: $clk"

create_clock_mesh -clock $clk -h_layer BM2 -v_layer BM1 -pitch 1.0 \
    -buffers {gt2_6t_buf_x4_w31_lvt} -cts_buffers {gt2_6t_buf_x4_w31_lvt}
detailed_placement -max_displacement 1000
setup_proxy_bterms -clock $clk -proxy_layer BM2

# quick audit
set ntsv 0
foreach inst [[ord::get_db_block] getInsts] {
  if {[[$inst getMaster] getName] eq "gt2_6t_TSV"} { incr ntsv }
}
puts ">>> placed $ntsv gt2_6t_TSV cells"

write_db $rdir/gcd_tsv_stage1.odb
puts ">>> Stage 1 done: gcd_tsv_stage1.odb"
exit 0
