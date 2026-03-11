# SPDX-License-Identifier: BSD-3-Clause

# High-level TCL commands for Clock Mesh generation

# Create a clock mesh grid
# Usage: create_clock_mesh -clock <clock_name> \
#                          -h_layer <horizontal_layer> \
#                          -v_layer <vertical_layer> \
#                          -pitch <pitch> \
#                          [-buffers {buf1 buf2 buf3...}]
# Note: Wire width is automatically taken from tech file (layer default width)
proc create_clock_mesh { args } {
    sta::parse_key_args "create_clock_mesh" args \
        keys {-clock -h_layer -v_layer -pitch -buffers} \
        flags {}

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

    # Optional buffers parameter (list of buffer cell names)
    # If not specified, no buffers will be inserted
    if { [info exists keys(-buffers)] } {
        set buffer_list $keys(-buffers)
    } else {
        set buffer_list {}
    }

    # Convert pitch to DBU
    set pitch_dbu [ord::microns_to_dbu $pitch]

    # Call the C++ backend command (wire width is auto-computed from tech file)
    cms::create_mesh_grid_cmd $clock_name $h_layer $v_layer $pitch_dbu $buffer_list
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
        keys {-clock -output -vdd -rise_time -fall_time -spice_models} \
        flags {}

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

    cms::write_mesh_spice_cmd $clock_name $spice_file $vdd $rise_time $fall_time $spice_models
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

