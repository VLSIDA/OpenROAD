# SPDX-License-Identifier: BSD-3-Clause

# High-level TCL commands for Clock Mesh generation

# Create a clock mesh grid
# Usage: create_clock_mesh -clock <clock_name> \
#                          -h_layer <horizontal_layer> \
#                          -v_layer <vertical_layer> \
#                          -pitch <pitch> \
#                          [-buffers {buf1 ...}] \
#                          [-cts_buffers {buf1 ...}] \
#                          [-macro_halo <microns>]
# -buffers:     cell(s) placed at mesh grid intersections
# -cts_buffers: cell(s) used by TritonCTS to build the tree from root to
#               mesh intersection buffers. If omitted, TritonCTS auto-infers
#               from the loaded liberty files.
# Note: Wire width is automatically taken from tech file (layer default width)
proc create_clock_mesh { args } {
    sta::parse_key_args "create_clock_mesh" args \
        keys {-clock -h_layer -v_layer -pitch -buffers -cts_buffers -macro_halo -mesh_strategy} \
        flags {-remove_colliding_wires -checkerboard_buffers}

    if { [info exists keys(-clock)] } {
        set clock_name $keys(-clock)
    } else {
        utl::error CMS 300 "Missing required argument: -clock"
    }

    if { [info exists keys(-h_layer)] } {
        set h_layer $keys(-h_layer)
    } else {
        utl::error CMS 301 "Missing required argument: -h_layer"
    }

    if { [info exists keys(-v_layer)] } {
        set v_layer $keys(-v_layer)
    } else {
        utl::error CMS 302 "Missing required argument: -v_layer"
    }

    if { [info exists keys(-pitch)] } {
        set pitch $keys(-pitch)
    } else {
        utl::error CMS 304 "Missing required argument: -pitch"
    }

    # Mesh intersection buffers (required for buffer-inserted mesh)
    if { [info exists keys(-buffers)] } {
        set buffer_list $keys(-buffers)
    } else {
        set buffer_list {}
    }

    # CTS tree buffers (optional — empty means TritonCTS auto-infers)
    if { [info exists keys(-cts_buffers)] } {
        set cts_buffer_list $keys(-cts_buffers)
    } else {
        set cts_buffer_list {}
    }

    # Optional macro halo (microns)
    if { [info exists keys(-macro_halo)] } {
        set macro_halo_dbu [ord::microns_to_dbu $keys(-macro_halo)]
    } else {
        set macro_halo_dbu 0
    }

    # Convert pitch to DBU
    set pitch_dbu [ord::microns_to_dbu $pitch]

    # Buffer-placement strategy: uniform | adaptive | set_cover
    # Default "adaptive" matches the previous behavior for buffer_list size > 1.
    if { [info exists keys(-mesh_strategy)] } {
        set mesh_strategy $keys(-mesh_strategy)
        switch -- $mesh_strategy {
            uniform - adaptive - set_cover {}
            default {
                utl::error CMS 305 \
                    "Invalid -mesh_strategy '$mesh_strategy'. Use uniform | adaptive | set_cover."
            }
        }
    } else {
        # Backward-compatible default: single buffer → uniform, multi → adaptive.
        if { [llength $buffer_list] > 1 } {
            set mesh_strategy "adaptive"
        } else {
            set mesh_strategy "uniform"
        }
    }

    set remove_colliding [info exists flags(-remove_colliding_wires)]
    # -checkerboard_buffers: mesh drivers at every OTHER intersection
    # ((row+col) even), so no driver has a driven up/down/left/right neighbor.
    # Halves the driver/TSV count; the mesh wire grid is unchanged.
    set checkerboard [info exists flags(-checkerboard_buffers)]
    cms::create_mesh_grid_cmd $clock_name $h_layer $v_layer $pitch_dbu $buffer_list $macro_halo_dbu $cts_buffer_list $mesh_strategy $remove_colliding $checkerboard
}

# Verification: compute the deterministic frozen grid and log a fragment report
# (CMS-144 summary + CMS-146 fragment sizes). Does not modify the design.
# Usage: compute_frozen_grid -h_layer BM2 -v_layer BM1 -pitch 1.0 \
#            -strap_pitch 2.16 -strap_offset 1.08 -strap_width 0.36
proc compute_frozen_grid { args } {
    sta::parse_key_args "compute_frozen_grid" args \
        keys {-h_layer -v_layer -pitch -strap_pitch -strap_offset -strap_width} \
        flags {}
    set pitch [ord::microns_to_dbu $keys(-pitch)]
    set sp    [ord::microns_to_dbu $keys(-strap_pitch)]
    set so    [ord::microns_to_dbu $keys(-strap_offset)]
    set sw    [ord::microns_to_dbu $keys(-strap_width)]
    cms::compute_frozen_grid_cmd $keys(-h_layer) $keys(-v_layer) $pitch $sp $so $sw
}

# Phase 1 RESERVE: place TSV cells + layer-selective keepouts at the frozen
# grid's intersections. Run at floorplan, BEFORE tapcell/PDN, so the PDN breaks
# the BPR rails around the keepouts automatically.
# Usage: reserve_clock_mesh -h_layer BM2 -v_layer BM1 -pitch 1.0 \
#            -strap_pitch 2.16 -strap_offset 1.08 -strap_width 0.36 \
#            -tsv_master gt2_6t_TSV [-keepout_w 0.31] [-keepout_h 0.176] [-spacing 2.0]
proc reserve_clock_mesh { args } {
    sta::parse_key_args "reserve_clock_mesh" args \
        keys {-h_layer -v_layer -pitch -strap_pitch -strap_offset -strap_width \
              -tsv_master -keepout_w -keepout_h -spacing} \
        flags {}
    set pitch [ord::microns_to_dbu $keys(-pitch)]
    set sp    [ord::microns_to_dbu $keys(-strap_pitch)]
    set so    [ord::microns_to_dbu $keys(-strap_offset)]
    set sw    [ord::microns_to_dbu $keys(-strap_width)]
    set kw [expr {[info exists keys(-keepout_w)] ? \
        [ord::microns_to_dbu $keys(-keepout_w)] : [ord::microns_to_dbu 0.31]}]
    set kh [expr {[info exists keys(-keepout_h)] ? \
        [ord::microns_to_dbu $keys(-keepout_h)] : [ord::microns_to_dbu 0.176]}]
    set spc [expr {[info exists keys(-spacing)] ? \
        [ord::microns_to_dbu $keys(-spacing)] : [ord::microns_to_dbu 2.0]}]
    cms::reserve_clock_mesh_cmd $keys(-h_layer) $keys(-v_layer) $pitch \
        $sp $so $sw $keys(-tsv_master) $kw $kh $spc
}

# Create sink-taps that bring the backside mesh up to local sink-buffers.
# Run AFTER create_clock_mesh + setup_proxy_bterms (the mesh + drive side must
# exist); BEFORE break_bpr_at_tsvs + detailed_placement.
# Usage: create_sink_taps -h_layer BM2 -v_layer BM1 -buffer <master> \
#            [-capacity 16] [-tsv_master gt2_6t_TSV] [-halo 0.112]
# -capacity: max FF clock pins one sink-buffer drives (nearest assignment,
#            spill to next-nearest when full).
proc create_sink_taps { args } {
    sta::parse_key_args "create_sink_taps" args \
        keys {-h_layer -v_layer -buffer -capacity -tsv_master -halo} \
        flags {-no_tsv}
    foreach req {-h_layer -v_layer -buffer} {
        if { ![info exists keys($req)] } {
            utl::error CMS 311 "Missing required argument: $req"
        }
    }
    # -no_tsv (frontside LCB tier): no TSV cell -- the LCB input is routed to
    # a proxy BTerm pinned on the nearest mesh wire instead.
    if { [info exists flags(-no_tsv)] } {
        set tsv_master ""
    } else {
        set tsv_master [expr {[info exists keys(-tsv_master)] ? \
            $keys(-tsv_master) : "gt2_6t_TSV"}]
    }
    set capacity [expr {[info exists keys(-capacity)] ? $keys(-capacity) : 16}]
    set halo [expr {[info exists keys(-halo)] ? \
        [ord::microns_to_dbu $keys(-halo)] : [ord::microns_to_dbu 0.112]}]
    cms::create_sink_taps_cmd $keys(-h_layer) $keys(-v_layer) \
        $tsv_master $keys(-buffer) $capacity $halo
}

# Break BPR power rails at every front<->back TSV (drive + sink). Run AFTER all
# TSVs are placed (create_clock_mesh + create_sink_taps); run detailed_placement
# afterward to relocate the cells stranded by the cut blockages.
# Usage: break_bpr_at_tsvs [-bpr_layer BPR] [-tsv_master gt2_6t_TSV] \
#            [-tap_master gt2_6t_tapbspdn_w31_lvt] [-halo 0.224] [-relocate_rows 1]
proc break_bpr_at_tsvs { args } {
    sta::parse_key_args "break_bpr_at_tsvs" args \
        keys {-bpr_layer -tsv_master -tap_master -halo -relocate_rows} \
        flags {}
    set bpr_layer [expr {[info exists keys(-bpr_layer)] ? \
        $keys(-bpr_layer) : "BPR"}]
    # -no_tsv (frontside LCB tier): no TSV cell -- the LCB input is routed to
    # a proxy BTerm pinned on the nearest mesh wire instead.
    if { [info exists flags(-no_tsv)] } {
        set tsv_master ""
    } else {
        set tsv_master [expr {[info exists keys(-tsv_master)] ? \
            $keys(-tsv_master) : "gt2_6t_TSV"}]
    }
    set tap_master [expr {[info exists keys(-tap_master)] ? \
        $keys(-tap_master) : "gt2_6t_tapbspdn_w31_lvt"}]
    set halo [expr {[info exists keys(-halo)] ? \
        [ord::microns_to_dbu $keys(-halo)] : [ord::microns_to_dbu 0.224]}]
    set rrows [expr {[info exists keys(-relocate_rows)] ? \
        $keys(-relocate_rows) : 1}]
    cms::break_bpr_at_tsvs_cmd $bpr_layer $tsv_master $tap_master $halo $rrows
}

# Connect sinks via router - places BTerms at grid intersections for router-based connections
# Call AFTER detailed_placement to legalize buffers, and AFTER setup_proxy_bterms for buffer BTerms
#
# This command:
#   1. Finds the nearest grid intersection for each sink
#   2. If that intersection has a buffer BTerm, the sink uses the same net
#   3. If not, creates a new sink BTerm at that intersection with net name sink_N
#   4. The router will then route sinks to their BTerms
#
# Usage: connect_sinks_to_mesh -clock <clock_name> -proxy_layer <layer_name>
proc connect_sinks_to_mesh { args } {
    sta::parse_key_args "connect_sinks_to_mesh" args \
        keys {-clock -proxy_layer} \
        flags {}

    if { [info exists keys(-clock)] } {
        set clock_name $keys(-clock)
    } else {
        utl::error CMS 305 "Missing required argument: -clock"
    }

    if { [info exists keys(-proxy_layer)] } {
        set proxy_layer $keys(-proxy_layer)
    } else {
        utl::error CMS 509 "Missing required argument: -proxy_layer"
    }

    cms::connect_sinks_cmd $clock_name $proxy_layer
}

# Setup proxy BTERMs at mesh intersections for router-based buffer connections
# This creates BTERMs on the proxy_layer (above the mesh) for buffer outputs.
# The router will then connect buffer outputs to these BTERMs.
#
# IMPORTANT: Call this BEFORE connect_sinks_to_mesh so that sinks can share buffer BTerms
#
# Full Flow:
#   1. create_clock_mesh (creates mesh grid + places buffers)
#   2. detailed_placement (legalizes buffer placement)
#   3. setup_proxy_bterms (creates buffer BTERMs at intersections)
#   4. connect_sinks_to_mesh (creates sink BTERMs or shares buffer BTERMs)
#   5. global_route / detailed_route (router connects buffer outputs and sinks to BTERMs)
#   6. connect_proxy_bterms_to_mesh (creates via stacks connecting BTERMs to mesh grid)
#
# Usage: setup_proxy_bterms -clock <clock_name> -proxy_layer <layer_name>
proc setup_proxy_bterms { args } {
    sta::parse_key_args "setup_proxy_bterms" args \
        keys {-clock -proxy_layer} \
        flags {}

    if { [info exists keys(-clock)] } {
        set clock_name $keys(-clock)
    } else {
        utl::error CMS 620 "Missing required argument: -clock"
    }

    if { [info exists keys(-proxy_layer)] } {
        set proxy_layer $keys(-proxy_layer)
    } else {
        utl::error CMS 621 "Missing required argument: -proxy_layer"
    }

    cms::setup_proxy_bterms_cmd $clock_name $proxy_layer
}

# Connect proxy BTERMs to mesh after routing
# This creates via stacks from the routed BTERM locations down to the mesh grid,
# effectively shorting the buffer nets to the mesh.
#
# Usage: connect_proxy_bterms_to_mesh -clock <clock_name>
proc connect_proxy_bterms_to_mesh { args } {
    sta::parse_key_args "connect_proxy_bterms_to_mesh" args \
        keys {-clock} \
        flags {}

    if { [info exists keys(-clock)] } {
        set clock_name $keys(-clock)
    } else {
        utl::error CMS 630 "Missing required argument: -clock"
    }

    cms::connect_proxy_bterms_to_mesh_cmd $clock_name
}

# Author sink_tap special wires (buffer.A -> TSV.A) at the sink buffers' FINAL
# positions. Call AFTER the post-break detailed_placement so the wires reach the
# legalized buffer locations. With -use_router the nets are instead left as
# ordinary routed nets for GRT/DRT (no special wire, no SPICE-prep re-author).
# Usage: connect_sink_taps [-use_router]
proc connect_sink_taps { args } {
    sta::parse_key_args "connect_sink_taps" args keys {} flags {-use_router}
    cms::connect_sink_taps_cmd [info exists flags(-use_router)]
}

# Re-map FFs to their nearest sink buffer using FINAL (post-placement) positions,
# capacity-capped. Repairs stale FF->buffer assignments left by the break_bpr
# displacement. Call AFTER the post-break detailed_placement, BEFORE
# connect_sink_taps. Usage: reassign_sink_ffs -capacity <F_max>
proc reassign_sink_ffs { args } {
    sta::parse_key_args "reassign_sink_ffs" args keys {-capacity} flags {}
    set cap [expr {[info exists keys(-capacity)] ? $keys(-capacity) : 0}]
    cms::reassign_sink_ffs_cmd $cap
}

# Capture CTS leaf arrival times from STA for SPICE skew analysis
# Must be called BEFORE merge_mesh_nets while STA timing graph is valid.
# The captured arrivals are used by write_mesh_spice to generate per-leaf-net
# clock sources with realistic CTS delay offsets.
#
# Usage: capture_mesh_arrivals -clock <clock_name>
proc capture_mesh_arrivals { args } {
    sta::parse_key_args "capture_mesh_arrivals" args \
        keys {-clock} \
        flags {}

    if { [info exists keys(-clock)] } {
        set clock_name $keys(-clock)
    } else {
        utl::error CMS 871 "Missing required argument: -clock"
    }

    cms::capture_leaf_arrivals_cmd $clock_name
}

# Merge buffer and sink nets into clk_mesh for parasitic extraction
# After routing, buffer output nets (clk_buf_*) and sink nets (sink_*)
# have their routing moved to clk_mesh and their BTERMs removed.
# This makes clk_mesh one complete net for OpenRCX extraction.
#
# Usage: merge_mesh_nets -clock <clock_name>
proc merge_mesh_nets { args } {
    sta::parse_key_args "merge_mesh_nets" args \
        keys {-clock} \
        flags {}

    if { [info exists keys(-clock)] } {
        set clock_name $keys(-clock)
    } else {
        utl::error CMS 810 "Missing required argument: -clock"
    }

    cms::merge_nets_to_mesh_cmd $clock_name
}

# Convert mesh grid SWires to regular dbWire for OpenRCX parasitic extraction
# After merge_mesh_nets, the grid is still stored as SWire (special wire).
# OpenRCX only extracts regular wires (dbWire), so this conversion is needed.
#
# Usage: convert_mesh_swire -clock <clock_name>
proc convert_mesh_swire { args } {
    sta::parse_key_args "convert_mesh_swire" args \
        keys {-clock} \
        flags {}

    if { [info exists keys(-clock)] } {
        set clock_name $keys(-clock)
    } else {
        utl::error CMS 853 "Missing required argument: -clock"
    }

    cms::convert_swire_to_wire_cmd $clock_name
}

# Write SPICE netlist from extracted parasitics for ngspice simulation
# Reads R, C, and coupling capacitance from OpenRCX extraction results
# and writes a SPICE-compatible netlist.
#
# VDD voltage and clock period are auto-detected from the Liberty library
# and SDC constraints. Use optional arguments to override:
#   -vdd <voltage>       Override supply voltage (e.g., 0.7 for ASAP7)
#   -rise_time <ns>      Override clock rise time in nanoseconds
#   -fall_time <ns>      Override clock fall time in nanoseconds
#
# Usage: write_mesh_spice -clock <clock_name> -output <spice_file> [-vdd <V>]
#                         [-rise_time <ns>] [-fall_time <ns>]
proc write_mesh_spice { args } {
    sta::parse_key_args "write_mesh_spice" args \
        keys {-clock -output -vdd -rise_time -fall_time -spice_models -tsv_res} \
        flags {-zero_delay -full_tree -finfet}

    if { [info exists keys(-clock)] } {
        set clock_name $keys(-clock)
    } else {
        utl::error CMS 863 "Missing required argument: -clock"
    }

    if { [info exists keys(-output)] } {
        set spice_file $keys(-output)
    } else {
        utl::error CMS 864 "Missing required argument: -output"
    }

    # Optional overrides (0.0 = auto-detect in C++)
    if { [info exists keys(-vdd)] } {
        set vdd $keys(-vdd)
    } else {
        set vdd 0.0
    }

    if { [info exists keys(-rise_time)] } {
        set rise_time $keys(-rise_time)
    } else {
        set rise_time 0.0
    }

    if { [info exists keys(-fall_time)] } {
        set fall_time $keys(-fall_time)
    } else {
        set fall_time 0.0
    }

    # Optional SPICE model/CDL files for subcircuit definitions
    if { [info exists keys(-spice_models)] } {
        set spice_models $keys(-spice_models)
    } else {
        set spice_models {}
    }

    set zero_delay [info exists flags(-zero_delay)]
    set full_tree  [info exists flags(-full_tree)]
    set finfet     [info exists flags(-finfet)]

    # TSV crossing resistance (ohm): passive front<->back via stack, from the
    # real GT2N ITF: V0 54.99 + VSD 36.86 + VBPR 32.0 + BV0 25.10 = 149
    # (chain M1 -> M0 -> SDCON -> BPR -> BM1)
    if { [info exists keys(-tsv_res)] } {
        set tsv_res $keys(-tsv_res)
    } else {
        set tsv_res 149.0
    }

    cms::write_mesh_spice_cmd $clock_name $spice_file $vdd $rise_time $fall_time $spice_models $zero_delay $full_tree $finfet $tsv_res
}

# Write mesh-merged Verilog netlist with correct connectivity
# Reads a Verilog file (generated by write_verilog) and modifies it:
#   - Removes internal proxy_* and sink_bterm_* BTERMs from port list
#   - Replaces buf_net_* and sink_* net names with the clock name
#   - Shows correct clock mesh connectivity for simulation/LVS
#
# Usage: write_mesh_verilog -clock <clock_name> -input <input.v> -output <output.v>
proc write_mesh_verilog { args } {
    sta::parse_key_args "write_mesh_verilog" args \
        keys {-clock -input -output} \
        flags {}

    if { [info exists keys(-clock)] } {
        set clock_name $keys(-clock)
    } else {
        utl::error CMS 760 "Missing required argument: -clock"
    }

    if { [info exists keys(-input)] } {
        set input_file $keys(-input)
    } else {
        utl::error CMS 761 "Missing required argument: -input"
    }

    if { [info exists keys(-output)] } {
        set output_file $keys(-output)
    } else {
        utl::error CMS 762 "Missing required argument: -output"
    }

    cms::write_mesh_verilog_cmd $clock_name $input_file $output_file
}

