# Backside clock mesh on GT2N (BM3/BM4), tree on frontside.
# Mirrors test/ASAP7/mesh_ibex_uniform_x4_M5M6.tcl but:
#   - mesh layers BM3 (V) / BM4 (H)  [backside]
#   - mesh-driver buffers = backside clones (Y on BM2), tree buffers = stock
#   - flop CLK pins already on BM2 (lib/LEF edited in the gt2n platform)
# Stops after the mesh is built + routed + converted (write_db). RCX/SPICE are
# intentionally omitted: OpenRCX still crashes on the extended backside stack.

set orfs    /home/wali2/backside/OpenROAD-flow-scripts/flow
set design  gcd
set results $orfs/results/gt2n/$design/base
set plat    $orfs/platforms/gt2n

set rdir [file join [file dirname [file normalize [info script]]] results]
file mkdir $rdir

read_db      $results/3_place.odb
read_liberty $plat/lib/gt2_6t_w31_lvt_tt_0p7v25c.lib
read_sdc     $results/3_place.sdc

detailed_placement
source $plat/setRC.tcl

set clk_name ""
foreach c [sta::all_clocks] { set clk_name [get_name $c]; break }
puts "Using clock: $clk_name  (mesh on BM3/BM4, backside)"

# Save the odb after every stage so each step is inspectable.
proc snap {dir tag} { write_db $dir/gcd_bs_$tag.odb; puts ">> saved stage: $tag" }

# Build the backside mesh. Tree buffers stay stock/frontside (A,Y on M1);
# mesh-driver buffers are the backside clones (A on M1, Y on BM2).
create_clock_mesh \
    -clock        $clk_name \
    -h_layer      BM4 \
    -v_layer      BM3 \
    -pitch        1.0 \
    -buffers      {mesh_gt2_6t_buf_x4_w31_lvt} \
    -cts_buffers  {gt2_6t_buf_x4_w31_lvt}
snap $rdir 1_mesh

detailed_placement -max_displacement 1000
snap $rdir 2_legalized

# Special-wire stub approach: connect backside pins (mesh-buffer outputs + flop
# clocks) directly to the SPECIAL mesh net and stitch the geometry ourselves.
# The router then never does pin access on backside layers (no DRT-0073), and
# the mesh net (special) is skipped by GRT.
connect_backside_pins_to_mesh -clock $clk_name
snap $rdir 3_stitched

# Router handles ONLY the frontside clock tree + signals (M2-M5). Span excludes
# BPR/M0, so no GRT-0126; the special mesh net is skipped, so no backside pins.
# Clock stays M2-M5 (matches DRT's floor); M1 buffer-input pins are reached via
# via-access (V1), exactly like normal M1-pin cells under MIN_ROUTING_LAYER=M2.
set_routing_layers -signal M2-M5 -clock M2-M5

global_route  -guide_file $rdir/${design}_bs_mesh.guide \
    -congestion_iterations 50
snap $rdir 4_grt
detailed_route -output_drc $rdir/${design}_bs_mesh_drc.rpt \
    -droute_end_iter 0
snap $rdir 5_drt

write_db  $rdir/${design}_bs_mesh.odb
write_def $rdir/${design}_bs_mesh.def

puts "Done: gcd backside mesh (BM3/BM4), special-wire stubs. RCX/SPICE skipped."
exit 0
