# Complete backside clock-mesh flow for gcd, built entirely from CMS commands
# plus the routing wrapper. After routing, saves the odb.
#   create_clock_mesh  -> backside BM1/BM2 mesh + drive TSVs + buffers
#   setup_proxy_bterms -> drive TSV.Y -> proxy BTerm on mesh
#   create_sink_taps   -> assign FFs to nearest tap, place sink-TSVs + buffers
#   break_bpr_at_tsvs  -> break BPR at every TSV (+relocate, clean, drop taps)
#   route + write_db
set orfs /home/wali2/backside/OpenROAD-flow-scripts/flow
set design [expr {[info exists ::env(DESIGN)] ? $::env(DESIGN) : "gcd"}]
# scale knobs (defaults = gcd tuning)
set pitch    [expr {[info exists ::env(MESH_PITCH)] ? $::env(MESH_PITCH) : 1.0}]
set capacity [expr {[info exists ::env(CAPACITY)]   ? $::env(CAPACITY)   : 16}]
set maxl     [expr {[info exists ::env(MAXL)]       ? $::env(MAXL)       : "M5"}]
set droute   [expr {[info exists ::env(DROUTE)]     ? $::env(DROUTE)     : 1}]
set res  $orfs/results/gt2n/$design/base
set plat $orfs/platforms/gt2n
set rdir [file join [file dirname [file normalize [info script]]] results]
file mkdir $rdir

# input odb override (e.g. a rebuilt/balanced-PDN base); default = flow 3_place
set odb_in [expr {[info exists ::env(ODB_IN)] ? $::env(ODB_IN) : "$res/3_place.odb"}]
read_db      $odb_in
# read every W/Vt liberty (the flow synthesizes with all families)
foreach lib [lsort [glob $plat/lib/gt2_6t_w*_tt_0p7v25c.lib]] {
  read_liberty $lib
}
read_sdc     $res/3_place.sdc
read_lef     $plat/lef/gt2_6t_TSV.lef
read_liberty $plat/lib/gt2_6t_TSV.lib
read_lef     $plat/lef/gt2_ntsv_via.lef

detailed_placement
source $plat/setRC.tcl
set clk ""
foreach c [sta::all_clocks] { set clk [get_name $c]; break }

create_clock_mesh -clock $clk -h_layer BM2 -v_layer BM1 -pitch $pitch \
    -buffers {gt2_6t_buf_x4_w31_lvt} -cts_buffers {gt2_6t_buf_x4_w31_lvt}
detailed_placement -max_displacement 1000
setup_proxy_bterms -clock $clk -proxy_layer BM2
create_sink_taps -h_layer BM2 -v_layer BM1 \
    -buffer gt2_6t_buf_x4_w31_lvt -capacity $capacity -tsv_master gt2_6t_TSV
break_bpr_at_tsvs -halo 0.224 -relocate_rows 1
detailed_placement -max_displacement 1000

# ---------------- route ----------------
# Keep the signal router off BPR (the power followpin layer) over the core; the
# clock crosses front<->back inside the TSV cells, so BPR is never needed.
set block  [ord::get_db_block]
set bpr    [[ord::get_db_tech] findLayer BPR]
set core_r [$block getCoreArea]
set bpr_obs [odb::dbObstruction_create $block $bpr \
    [$core_r xMin] [$core_r yMin] [$core_r xMax] [$core_r yMax]]
# b_* backside nets are SPECIAL (TSV->grid stubs drawn by CMS), so no signal
# net needs the backside layers -- the router stays entirely on the frontside.
set_routing_layers -signal M2-$maxl -clock M2-$maxl
global_route   -guide_file $rdir/${design}_bk.guide -congestion_iterations 50
if {$droute} {
  detailed_route -output_drc $rdir/${design}_bk_drc.rpt -droute_end_iter 0
} else {
  puts ">>> DROUTE=0: skipping detailed route (structure/power pass)"
}
odb::dbObstruction_destroy $bpr_obs

write_db $rdir/${design}_backside.odb
puts ">>> backside flow done: $rdir/${design}_backside.odb"
exit 0
