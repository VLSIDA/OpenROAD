# SPDX-License-Identifier: BSD-3-Clause

# High-level TCL commands for Clock Mesh generation

# Create a clock mesh grid
# Usage: create_clock_mesh -clock <clock_name> \
#                          -h_layer <horizontal_layer> \
#                          -v_layer <vertical_layer> \
#                          -pitch <pitch> \
#                          [-buffers {buf1 buf2 buf3...}] \
#                          [-tree_layer <tree_layer>]
# Note: Wire width is automatically taken from tech file (layer default width)
proc create_clock_mesh { args } {
    sta::parse_key_args "create_clock_mesh" args \
        keys {-clock -h_layer -v_layer -pitch -buffers -tree_layer} \
        flags {}

    if { [info exists keys(-clock)] } {
        set clock_name $keys(-clock)
    } else {
        utl::error MESH 300 "Missing required argument: -clock"
    }

    if { [info exists keys(-h_layer)] } {
        set h_layer $keys(-h_layer)
    } else {
        utl::error MESH 301 "Missing required argument: -h_layer"
    }

    if { [info exists keys(-v_layer)] } {
        set v_layer $keys(-v_layer)
    } else {
        utl::error MESH 302 "Missing required argument: -v_layer"
    }

    if { [info exists keys(-pitch)] } {
        set pitch $keys(-pitch)
    } else {
        utl::error MESH 304 "Missing required argument: -pitch"
    }

    # Optional buffers parameter (list of buffer cell names)
    # If not specified, no buffers will be inserted
    if { [info exists keys(-buffers)] } {
        set buffer_list $keys(-buffers)
    } else {
        set buffer_list {}
    }

    # Optional tree_layer parameter
    # If not specified, will default to h_layer + 1 in C++ code
    if { [info exists keys(-tree_layer)] } {
        set tree_layer $keys(-tree_layer)
    } else {
        set tree_layer ""
    }

    # Convert pitch to DBU
    set pitch_dbu [ord::microns_to_dbu $pitch]

    # Call the C++ backend command (wire width is auto-computed from tech file)
    mesh::create_mesh_grid_cmd $clock_name $h_layer $v_layer $pitch_dbu $buffer_list $tree_layer
}

# Run mesh analysis (find clock sinks)
proc run_mesh { args } {
    sta::parse_key_args "run_mesh" args \
        keys {-clock} \
        flags {}

    if { [info exists keys(-clock)] } {
        set clock_name $keys(-clock)
    } else {
        set clock_name "default"
    }

    mesh::run_mesh_cmd $clock_name
}

# Connect sinks to mesh - call AFTER detailed_placement to legalize buffers
# Usage: connect_sinks_to_mesh -clock <clock_name>
proc connect_sinks_to_mesh { args } {
    sta::parse_key_args "connect_sinks_to_mesh" args \
        keys {-clock} \
        flags {}

    if { [info exists keys(-clock)] } {
        set clock_name $keys(-clock)
    } else {
        utl::error MESH 305 "Missing required argument: -clock"
    }

    mesh::connect_sinks_cmd $clock_name
}

# OBSOLETE: Using router-based proxy BTERM approach instead
# Connect buffers to their associated intersections - call AFTER connect_sinks_to_mesh
# Each buffer connects back to the intersection where it was originally placed
# Usage: connect_buffers_to_mesh -clock <clock_name>
# proc connect_buffers_to_mesh { args } {
#     sta::parse_key_args "connect_buffers_to_mesh" args \
#         keys {-clock} \
#         flags {}
#
#     if { [info exists keys(-clock)] } {
#         set clock_name $keys(-clock)
#     } else {
#         utl::error MESH 306 "Missing required argument: -clock"
#     }
#
#     mesh::connect_buffers_cmd $clock_name
# }

# Setup proxy BTERMs at mesh intersections for router-based buffer connections
# This creates BTERMs on the proxy_layer (above the mesh) with via stacks down to the mesh.
# The router will then connect buffer outputs to these BTERMs.
#
# Flow:
#   1. create_clock_mesh (creates mesh grid + places buffers)
#   2. detailed_placement (legalizes buffer placement)
#   3. connect_sinks_to_mesh (connects sinks to mesh)
#   4. setup_proxy_bterms (creates BTERMs + via stacks, connects buffer outputs to mesh_net)
#   5. global_route / detailed_route (router connects buffer outputs to proxy BTERMs)
#
# Usage: setup_proxy_bterms -clock <clock_name> -proxy_layer <layer_name>
proc setup_proxy_bterms { args } {
    sta::parse_key_args "setup_proxy_bterms" args \
        keys {-clock -proxy_layer} \
        flags {}

    if { [info exists keys(-clock)] } {
        set clock_name $keys(-clock)
    } else {
        utl::error MESH 620 "Missing required argument: -clock"
    }

    if { [info exists keys(-proxy_layer)] } {
        set proxy_layer $keys(-proxy_layer)
    } else {
        utl::error MESH 621 "Missing required argument: -proxy_layer"
    }

    mesh::setup_proxy_bterms_cmd $clock_name $proxy_layer
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
        utl::error MESH 630 "Missing required argument: -clock"
    }

    mesh::connect_proxy_bterms_to_mesh_cmd $clock_name
}
