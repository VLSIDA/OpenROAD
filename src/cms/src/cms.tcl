# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2025, The OpenROAD Authors

sta::define_cmd_args "create_clock_mesh" {-clock clock_name \
                                          -h_layer layer \
                                          -v_layer layer \
                                          -pitch pitch \
                                          [-buffers buffers] \
                                          [-cts_buffers buffers] \
                                          [-macro_halo halo] \
                                          [-mesh_strategy strategy] \
                                          [-buffer_gain gain]}

proc create_clock_mesh { args } {
  sta::parse_key_args "create_clock_mesh" args \
    keys {-clock -h_layer -v_layer -pitch -buffers -cts_buffers -macro_halo \
          -mesh_strategy -buffer_gain} flags {}
  sta::check_argc_eq0 "create_clock_mesh" $args

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

  # Buffers placed at the mesh grid intersections.
  set buffer_list {}
  if { [info exists keys(-buffers)] } {
    set buffer_list $keys(-buffers)
  }

  # Buffers for the TritonCTS tree from the root to the mesh buffers.
  # An empty list lets TritonCTS infer them from the Liberty libraries.
  set cts_buffer_list {}
  if { [info exists keys(-cts_buffers)] } {
    set cts_buffer_list $keys(-cts_buffers)
  }

  set macro_halo_dbu 0
  if { [info exists keys(-macro_halo)] } {
    set macro_halo_dbu [ord::microns_to_dbu $keys(-macro_halo)]
  }

  set pitch_dbu [ord::microns_to_dbu $pitch]

  # Buffer placement strategy: uniform | adaptive | set_cover.
  # Default: a single buffer master is placed uniformly, several masters are
  # sized adaptively from the sink load.
  if { [info exists keys(-mesh_strategy)] } {
    set mesh_strategy $keys(-mesh_strategy)
    switch -- $mesh_strategy {
      uniform - adaptive - set_cover { }
      default {
        utl::error CMS 303 "Invalid -mesh_strategy '$mesh_strategy'.\
          Use uniform | adaptive | set_cover."
      }
    }
  } elseif { [llength $buffer_list] > 1 } {
    set mesh_strategy "adaptive"
  } else {
    set mesh_strategy "uniform"
  }

  # A buffer may drive buffer_gain times its own input capacitance.
  set buffer_gain 3.0
  if { [info exists keys(-buffer_gain)] } {
    set buffer_gain $keys(-buffer_gain)
    if { $buffer_gain <= 0.0 } {
      utl::error CMS 306 "-buffer_gain must be positive."
    }
  }

  cms::create_mesh_grid_cmd $clock_name $h_layer $v_layer $pitch_dbu \
    $buffer_list $macro_halo_dbu $cts_buffer_list $mesh_strategy $buffer_gain
}

sta::define_cmd_args "setup_proxy_bterms" {-clock clock_name \
                                           -proxy_layer layer}

# Creates a BTerm on the proxy layer above every mesh buffer output so the
# router can connect the buffer to it. Call before connect_sinks_to_mesh.
proc setup_proxy_bterms { args } {
  sta::parse_key_args "setup_proxy_bterms" args \
    keys {-clock -proxy_layer} flags {}
  sta::check_argc_eq0 "setup_proxy_bterms" $args

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

sta::define_cmd_args "connect_sinks_to_mesh" {-clock clock_name \
                                              -proxy_layer layer}

# Moves every sink to a BTerm at its nearest grid intersection. A sink whose
# intersection already has a buffer BTerm shares that BTerm's net; otherwise
# a new sink BTerm and net are created. The router then connects them.
proc connect_sinks_to_mesh { args } {
  sta::parse_key_args "connect_sinks_to_mesh" args \
    keys {-clock -proxy_layer} flags {}
  sta::check_argc_eq0 "connect_sinks_to_mesh" $args

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

sta::define_cmd_args "connect_proxy_bterms_to_mesh" {-clock clock_name}

# After routing, drops via stacks from every proxy and sink BTerm to the
# mesh stripe beneath it.
proc connect_proxy_bterms_to_mesh { args } {
  sta::parse_key_args "connect_proxy_bterms_to_mesh" args \
    keys {-clock} flags {}
  sta::check_argc_eq0 "connect_proxy_bterms_to_mesh" $args

  if { [info exists keys(-clock)] } {
    set clock_name $keys(-clock)
  } else {
    utl::error CMS 630 "Missing required argument: -clock"
  }

  cms::connect_proxy_bterms_to_mesh_cmd $clock_name
}

sta::define_cmd_args "capture_mesh_arrivals" {-clock clock_name}

# Records the STA arrival time and slew at every mesh buffer input for
# write_mesh_spice. Call before merge_mesh_nets while the graph is acyclic.
proc capture_mesh_arrivals { args } {
  sta::parse_key_args "capture_mesh_arrivals" args \
    keys {-clock} flags {}
  sta::check_argc_eq0 "capture_mesh_arrivals" $args

  if { [info exists keys(-clock)] } {
    set clock_name $keys(-clock)
  } else {
    utl::error CMS 871 "Missing required argument: -clock"
  }

  cms::capture_leaf_arrivals_cmd $clock_name
}

sta::define_cmd_args "merge_mesh_nets" {-clock clock_name}

# Moves the routing of the buffer and sink nets onto the mesh net and removes
# their BTerms, leaving one net for parasitic extraction.
proc merge_mesh_nets { args } {
  sta::parse_key_args "merge_mesh_nets" args \
    keys {-clock} flags {}
  sta::check_argc_eq0 "merge_mesh_nets" $args

  if { [info exists keys(-clock)] } {
    set clock_name $keys(-clock)
  } else {
    utl::error CMS 810 "Missing required argument: -clock"
  }

  cms::merge_nets_to_mesh_cmd $clock_name
}

sta::define_cmd_args "convert_mesh_swire" {-clock clock_name}

# Converts the mesh stripes from special wires to regular wires so that
# OpenRCX can extract them.
proc convert_mesh_swire { args } {
  sta::parse_key_args "convert_mesh_swire" args \
    keys {-clock} flags {}
  sta::check_argc_eq0 "convert_mesh_swire" $args

  if { [info exists keys(-clock)] } {
    set clock_name $keys(-clock)
  } else {
    utl::error CMS 853 "Missing required argument: -clock"
  }

  cms::convert_swire_to_wire_cmd $clock_name
}

sta::define_cmd_args "write_mesh_spice" {-clock clock_name \
                                         -output file \
                                         [-vdd voltage] \
                                         [-rise_time time] \
                                         [-fall_time time] \
                                         [-spice_models files] \
                                         [-zero_delay] \
                                         [-full_tree]}

# Writes a SPICE netlist of the mesh from the extracted parasitics. Supply
# voltage and clock period default to the Liberty and SDC values.
proc write_mesh_spice { args } {
  sta::parse_key_args "write_mesh_spice" args \
    keys {-clock -output -vdd -rise_time -fall_time -spice_models} \
    flags {-zero_delay -full_tree}
  sta::check_argc_eq0 "write_mesh_spice" $args

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

  # 0.0 means auto-detect in C++.
  set vdd 0.0
  if { [info exists keys(-vdd)] } {
    set vdd $keys(-vdd)
  }

  set rise_time 0.0
  if { [info exists keys(-rise_time)] } {
    set rise_time $keys(-rise_time)
  }

  set fall_time 0.0
  if { [info exists keys(-fall_time)] } {
    set fall_time $keys(-fall_time)
  }

  set spice_models {}
  if { [info exists keys(-spice_models)] } {
    set spice_models $keys(-spice_models)
  }

  set zero_delay [info exists flags(-zero_delay)]
  set full_tree [info exists flags(-full_tree)]

  cms::write_mesh_spice_cmd $clock_name $spice_file $vdd $rise_time \
    $fall_time $spice_models $zero_delay $full_tree
}
