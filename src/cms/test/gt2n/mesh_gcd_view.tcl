# Produce a VIEWABLE routed result: single-pass backside clock, with the 4 known
# offset-crossing sinks (19/27/32/34) masked so detailed_route completes. Writes
# the odb + renders PNGs (full, frontside-only, backside-only) for inspection.
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

# Mask the 4 offset-crossing sinks that fail DRT-0218 so the route completes.
set b [ord::get_db_block]
foreach nm {sink_19 sink_27 sink_32 sink_34} {
  set n [$b findNet $nm]
  if {$n ne "NULL"} { $n setSpecial }
}

set_routing_layers -signal M2-M5 -clock BM2-M5
global_route   -guide_file $rdir/view.guide -congestion_iterations 50
detailed_route -output_drc $rdir/view_drc.rpt -droute_end_iter 0
write_db $rdir/gcd_view.odb
puts ">>> routed odb written: $rdir/gcd_view.odb"

# ---------- render images ----------
gui::show "" false
gui::fit

# 1) everything
gui::save_image $rdir/view_all.png

# helper: set visibility of a list of routing layers
proc only_layers {b show hide} {
  foreach L $hide { catch { gui::set_display_control "Layers/$L" visible false } }
  foreach L $show { catch { gui::set_display_control "Layers/$L" visible true  } }
}

# 2) frontside metals only (the clock tree on top)
only_layers $b {M0 M1 M2 M3 M4 M5} {BM1 BM2 BM3 BM4 BPR}
gui::save_image $rdir/view_frontside.png

# 3) backside metals only (the mesh + crossings)
only_layers $b {BM1 BM2 BM3 BM4 BPR} {M0 M1 M2 M3 M4 M5 M6 M7}
gui::save_image $rdir/view_backside.png

puts ">>> images: view_all.png view_frontside.png view_backside.png in $rdir"
exit 0
