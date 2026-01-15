#!/usr/bin/env tclsh
# Highlight mesh buffers, CTS tree buffers, and DFF cells in OpenROAD GUI
#
# Usage: In GUI console, run: source highlight_cells.tcl
#
# Highlight groups (colors):
#   0: Red, 1: Orange, 2: Yellow, 3: Green, 4: Blue, 5: Purple

# Clear any existing selections/highlights
gui::clear_selections
gui::clear_highlights

# Count cells
set block [ord::get_db_block]
set mesh_buf_count 0
set tree_buf_count 0
set dff_count 0

foreach inst [$block getInsts] {
  set name [$inst getName]
  if { [string match "mesh_buf_*" $name] } {
    incr mesh_buf_count
  } elseif { [string match "clkbuf_*_tree" $name] || [string match "*_clock_tree*" $name] } {
    # CTS tree buffers: clkbuf_0_core_clock_tree, clkbuf_1_core_clock_tree, etc.
    incr tree_buf_count
  } elseif { [string match "*dff*" $name] || [string match "*DFF*" $name] } {
    incr dff_count
  }
}

# Highlight mesh buffers in green (group 3)
select -type Inst -name "mesh_buf_*" -highlight 3

# Highlight CTS tree buffers in yellow (group 2)
# TritonCTS naming: clkbuf_<level>_<net_name>
select -type Inst -name "clkbuf_*_tree" -highlight 2
select -type Inst -name "*_clock_tree*" -highlight 2

# Highlight DFFs in blue (group 4)
select -type Inst -name "*dff*" -highlight 4
select -type Inst -name "*DFF*" -highlight 4

puts "========================================"
puts "Highlighted cells:"
puts "  Mesh buffers (GREEN):  $mesh_buf_count instances (mesh_buf_*)"
puts "  Tree buffers (YELLOW): $tree_buf_count instances (clkbuf_*_tree)"
puts "  DFF cells (BLUE):      $dff_count instances (*dff*)"
puts "========================================"
puts "Total clock network cells: [expr {$mesh_buf_count + $tree_buf_count}]"
puts "Total clock sinks (DFFs):  $dff_count"
puts "========================================"
