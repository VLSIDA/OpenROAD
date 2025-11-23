# SPDX-License-Identifier: BSD-3-Clause

sta::define_cmd_args "mesh" { \
  [-name name] \
}

proc mesh { args } {
  sta::parse_key_args "mesh" args \
    keys {-name} flags {}

  set has_name [info exists keys(-name)]
  set name "default"
  if { $has_name } {
    set name $keys(-name)
  }

  sta::check_argc_eq0 "mesh" $args

  utl::info "MESH" 100 "mesh start running"
  mesh::run_cmd $name
}
