# Complete backside clock-mesh flow for JPEG -> routed jpeg_backside.odb.
# Uses the ANALYTIC RC model (setRC.tcl); OpenRCX is deferred.
# Same CMS command sequence as gcd, but jpeg-scaled:
#   - routes signal+clock on M2-M9 (jpeg's MAX_ROUTING_LAYER, vs gcd M5)
#   - larger core => more sinks/TSVs, but capacity/pitch heuristics unchanged
#   create_clock_mesh -> setup_proxy_bterms -> create_sink_taps
#   -> break_bpr_at_tsvs -> route -> write_db
set orfs /home/wali2/backside/OpenROAD-flow-scripts/flow
set design jpeg
set res  $orfs/results/gt2n/$design/base
set plat $orfs/platforms/gt2n
set rdir [file join [file dirname [file normalize [info script]]] results]
file mkdir $rdir

# Start from the BALANCED-PDN base (4 wide straps/net @1.5um), NOT 3_place --
# 3_place carries the dense ORFS PDN (~53 thin 0.2um straps/layer). Build it
# first with:  DESIGN=jpeg PDN_W=1.5 PDN_N=4 openroad -exit pdn_balance.tcl
set base_odb $rdir/jpeg_pdn_balanced.odb
if {![file exists $base_odb]} {
  error "missing $base_odb -- run pdn_balance.tcl first (DESIGN=jpeg PDN_W=1.5 PDN_N=4)"
}
read_db      $base_odb
# jpeg synthesizes across all W/Vt families -> read every liberty
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
puts ">>> jpeg backside flow: clock=$clk"

# ---- backside clock mesh (BM1/BM2) + drive buffers/TSVs ----
# Mesh pitch 10um matches jpeg_b's density (9x9=~81 intersections over the
# 80.6x80.5um core). This sparse mesh (~147 TSVs vs ~2767 at pitch 1.0) is what
# lets the post-break legalize converge -- dense TSV blockages were what stalled
# detailed_placement. See gt2n_legalize_maxdisp / the jpeg_b comparison.
create_clock_mesh -clock $clk -h_layer BM2 -v_layer BM1 -pitch 3.0 \
    -buffers {gt2_6t_buf_x4_w31_lvt} -cts_buffers {gt2_6t_buf_x4_w31_lvt}
detailed_placement -max_displacement {80 6}
setup_proxy_bterms -clock $clk -proxy_layer BM2
# capacity 128: only ~81 candidate tap sites at pitch 10, so each must hold many
# FFs (4384 FFs / ~66 used taps ~= 66 each). 16 would overflow (81*16 < 4384).
create_sink_taps -h_layer BM2 -v_layer BM1 \
    -buffer gt2_6t_buf_x4_w31_lvt -capacity 16 -tsv_master gt2_6t_TSV
break_bpr_at_tsvs -halo 0.224 -relocate_rows 1
detailed_placement -max_displacement {80 6}

# ---------------- route (frontside only, M2-M9) ----------------
# BPR (power followpin) is blocked over the core; the clock crosses front<->back
# inside the TSV cells so no signal net needs BPR or the backside layers.
set block  [ord::get_db_block]
set bpr    [[ord::get_db_tech] findLayer BPR]
set core_r [$block getCoreArea]
set bpr_obs [odb::dbObstruction_create $block $bpr \
    [$core_r xMin] [$core_r yMin] [$core_r xMax] [$core_r yMax]]
set_routing_layers -signal M2-M9 -clock M2-M9
global_route   -guide_file $rdir/${design}_bk.guide -congestion_iterations 50
detailed_route -output_drc $rdir/${design}_bk_drc.rpt -droute_end_iter 0
odb::dbObstruction_destroy $bpr_obs

write_db $rdir/${design}_backside.odb
puts ">>> backside flow done: $rdir/${design}_backside.odb"
exit 0
