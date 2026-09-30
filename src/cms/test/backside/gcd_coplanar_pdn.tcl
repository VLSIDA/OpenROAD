# EXPERIMENT: co-planar fine-layer PDN. Drop the coarse BM3/BM4 power straps and
# put a DENSE vertical PDN on BM1 (+ BPR followpins) instead. The clock mesh
# lives on the same fine layers: V-wires on BM1 auto-dodge the BM1 PDN straps
# (collectPdnVStraps is layer-agnostic), H-wires on BM2 stay free. Then route and
# check IR to see whether thin-metal power can still meet the IR budget on gcd.
set orfs /home/wali2/backside/OpenROAD-flow-scripts/flow
set res  $orfs/results/gt2n/gcd/base
set plat $orfs/platforms/gt2n
set rdir [file join [file dirname [file normalize [info script]]] results]
file mkdir $rdir

read_db      $res/3_place.odb
read_liberty $plat/lib/gt2_6t_w31_lvt_tt_0p7v25c.lib
read_sdc     $res/3_place.sdc
read_lef     $plat/lef/gt2_6t_TSV.lef
read_liberty $plat/lib/gt2_6t_TSV.lib
read_lef     $plat/lef/gt2_ntsv_via.lef

set block [ord::get_db_block]

# ---------- strip the existing coarse PDN (BM3/BM4/BM1/BPR special wires) ----------
foreach nn {vdd vss} {
  set net [$block findNet $nn]
  if {$net eq "NULL"} continue
  set sws {}
  foreach sw [$net getSWires] { lappend sws $sw }
  foreach sw $sws { odb::dbSWire_destroy $sw }
}
puts ">>> stripped old PDN special wires"

# ---------- rebuild a DENSE fine-layer PDN: BPR followpins + BM1 vertical ----------
add_global_connection -net {vdd} -inst_pattern {.*} -pin_pattern {^vdd$} -power
add_global_connection -net {vss} -inst_pattern {.*} -pin_pattern {^vss$} -ground
global_connect
set_voltage_domain -name {CORE} -power {vdd} -ground {vss}
define_pdn_grid -name {fgrid} -voltage_domains {CORE} -pins {BM1}
# cell power rail (BPR followpins), unchanged
add_pdn_stripe -grid {fgrid} -layer {BPR} -width {0.032} -pitch {0.144} -offset {0} -followpins
# Vertical straps on the FINE layer BM1. 1.0um-pair pitch collapsed the clock
# mesh (create_sink_taps -> 0 candidates); back off to the original 2.16um strap
# pitch (clock mesh forms) but keep them WIDE (0.2um) for low IR.
add_pdn_stripe -grid {fgrid} -layer {BM1} -width {0.2} -spacing {0.1} -pitch {2.16} -offset {1.08}
# tie BPR up to the BM1 straps (BV0 via)
add_pdn_connect -grid {fgrid} -layers {BPR BM1}
pdngen
puts ">>> rebuilt PDN on BM1 (dense) + BPR"

# ---------- clock-mesh flow (unchanged CMS commands) ----------
detailed_placement
source $plat/setRC.tcl
set clk ""
foreach c [sta::all_clocks] { set clk [get_name $c]; break }

create_clock_mesh -clock $clk -h_layer BM2 -v_layer BM1 -pitch 1.0 \
    -buffers {gt2_6t_buf_x4_w31_lvt} -cts_buffers {gt2_6t_buf_x4_w31_lvt}
detailed_placement -max_displacement 1000
setup_proxy_bterms -clock $clk -proxy_layer BM2
create_sink_taps -h_layer BM2 -v_layer BM1 \
    -buffer gt2_6t_buf_x4_w31_lvt -capacity 16 -tsv_master gt2_6t_TSV
break_bpr_at_tsvs -halo 0.112 -relocate_rows 1
detailed_placement -max_displacement 1000

# ---------- route ----------
set bpr    [[ord::get_db_tech] findLayer BPR]
set core_r [$block getCoreArea]
set bpr_obs [odb::dbObstruction_create $block $bpr \
    [$core_r xMin] [$core_r yMin] [$core_r xMax] [$core_r yMax]]
set_routing_layers -signal M2-M5 -clock BM2-M5
global_route   -guide_file $rdir/cop.guide -congestion_iterations 50
detailed_route -output_drc $rdir/cop_drc.rpt -droute_end_iter 0
odb::dbObstruction_destroy $bpr_obs

write_db $rdir/gcd_coplanar.odb
puts ">>> co-planar flow done: $rdir/gcd_coplanar.odb"

# ---------- IR / connectivity ----------
puts ">>> === connectivity check (vdd/vss) ==="
foreach nn {vdd vss} {
  if {[catch {check_power_grid -net $nn -dont_require_terminals -error_file $rdir/${nn}_open.rpt} msg]} {
    puts ">>> $nn: $msg"
  } else {
    puts ">>> $nn: connectivity OK"
  }
}
exit 0
