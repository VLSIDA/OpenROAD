# SPDX-License-Identifier: BSD-3-Clause

# High-level TCL commands for Clock Mesh generation

# Create a clock mesh grid
# Usage: create_clock_mesh -clock <clock_name> \
#                          -h_layer <horizontal_layer> \
#                          -v_layer <vertical_layer> \
#                          -wire_width <width> \
#                          -pitch <pitch> \
#                          [-buffers {buf1 buf2 buf3...}]
proc create_clock_mesh { args } {
    sta::parse_key_args "create_clock_mesh" args \
        keys {-clock -h_layer -v_layer -wire_width -pitch -buffers} \
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

    # Convert wire_width and pitch to DBU if needed
    set wire_width_dbu [ord::microns_to_dbu $wire_width]
    set pitch_dbu [ord::microns_to_dbu $pitch]

    # Call the C++ backend command with buffer list
    mesh::create_mesh_grid_cmd $clock_name $h_layer $v_layer $wire_width_dbu $pitch_dbu $buffer_list
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
