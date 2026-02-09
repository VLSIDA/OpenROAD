# Unit test for clock mesh grid creation - Sky130HD
source "helpers.tcl"

puts "Loading database"
read_db "3_place.odb"

puts "Loading liberty"
read_liberty "sky130hd/sky130_fd_sc_hd__tt_025C_1v80.lib"

puts "Loading SDC"
read_sdc "3_place.sdc"

set clk_name "core_clock"

# Create clock mesh grid
puts "\nCreating clock mesh grid"
create_clock_mesh \
    -clock $clk_name \
    -h_layer met4 \
    -v_layer met3 \
    -pitch 2.0


# 1. Write ODB (database snapshot)
set odb_file [make_result_file "mesh_grid_sky130hd.odb"]
puts "Writing ODB to: $odb_file"
write_db $odb_file

set def_file [make_result_file "mesh_grid_sky130hd.def"]
puts "Writing DEF to: $def_file"
write_def $def_file

set guide_file [make_result_file "mesh_grid_sky130hd.guide"]
puts "Writing guides to: $guide_file"
write_guides $guide_file

set block [ord::get_db_block]
set mesh_net [$block findNet "clk"]

if { $mesh_net == "NULL" } {
  utl::error CMS 400 "Mesh net 'clk' not found!"
}

# set root [$mesh_net get1stBTerm]
# if { $root == "NULL" } {
#   utl::error CMS 401 "Clock root not connected to mesh!"
# }
# puts "Root connected: [$root getName]"

# Check sink count
set sinks [$mesh_net getITerms]
#gcd have 35 sinks 
set expected_sinks 35  
if { [llength $sinks] != $expected_sinks } {
  utl::error CMS 402 "Expected $expected_sinks sinks, got [llength $sinks]"
}
puts "Sinks connected: [llength $sinks]/$expected_sinks"

if { [file exists "mesh_grid_sky130hd.defok"] } {
  puts "\nComparing DEF to golden reference..."
  diff_files "mesh_grid_sky130hd.defok" $def_file
} 
  
if { [file exists "mesh_grid_sky130hd.guideok"] } {
  puts "Comparing guides to golden reference..."
  diff_files "mesh_grid_sky130hd.guideok" $guide_file
} 

puts "\npass"
exit 0