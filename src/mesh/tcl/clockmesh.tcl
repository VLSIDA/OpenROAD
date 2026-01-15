# SPDX-License-Identifier: BSD-3-Clause

# High-level TCL commands for Clock Mesh generation

# Create a clock mesh grid
# Usage: create_clock_mesh -clock <clock_name> \
#                          -h_layer <horizontal_layer> \
#                          -v_layer <vertical_layer> \
#                          -wire_width <width> \
#                          -pitch <pitch> \
#                          [-buffers {buf1 buf2 buf3...}] \
#                          [-tree_layer <tree_layer>]
proc create_clock_mesh { args } {
    sta::parse_key_args "create_clock_mesh" args \
        keys {-clock -h_layer -v_layer -wire_width -pitch -buffers -tree_layer} \
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

    if { [info exists keys(-wire_width)] } {
        set wire_width $keys(-wire_width)
    } else {
        utl::error MESH 303 "Missing required argument: -wire_width"
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

    # Convert wire_width and pitch to DBU if needed
    set wire_width_dbu [ord::microns_to_dbu $wire_width]
    set pitch_dbu [ord::microns_to_dbu $pitch]

    # Call the C++ backend command with buffer list and tree layer
    mesh::create_mesh_grid_cmd $clock_name $h_layer $v_layer $wire_width_dbu $pitch_dbu $buffer_list $tree_layer
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

# Connect buffers to their associated intersections - call AFTER connect_sinks_to_mesh
# Each buffer connects back to the intersection where it was originally placed
# Usage: connect_buffers_to_mesh -clock <clock_name>
proc connect_buffers_to_mesh { args } {
    sta::parse_key_args "connect_buffers_to_mesh" args \
        keys {-clock} \
        flags {}

    if { [info exists keys(-clock)] } {
        set clock_name $keys(-clock)
    } else {
        utl::error MESH 306 "Missing required argument: -clock"
    }

    mesh::connect_buffers_cmd $clock_name
}
