source "helpers.tcl"
read_db "results/mesh_cts_integration_sky130hd-tcl.odb"
read_liberty "sky130hd/lib/sky130_fd_sc_hd__tt_025C_1v80.lib"

# Sky130 RCX rules
set SKY130_BASE "$::env(HOME)/.ciel/ciel/sky130/versions/54435919abffb937387ec956209f9cf5fd2dfbee/sky130B"
set SKY130_RCX_RULES "$SKY130_BASE/libs.tech/openlane/rules.openrcx.sky130B.nom.spef_extractor"

# Re-run parasitic extraction (parasitics are not saved in ODB)
puts "\n=== Extracting parasitics ==="
define_process_corner -ext_model_index 0 X
extract_parasitics -ext_model_file $SKY130_RCX_RULES

puts "\n=== Checking parasitics ==="

# puts "\n=== Parasitics for buf_net_1380_2300 ==="
# report_parasitics -net buf_net_1380_2300

# puts "\n=== Parasitics for sink_1 ==="
# report_parasitics -net sink_1

puts "\n=== Parasitics for core_clock_tree ==="
report_parasitics -net core_clock_tree

puts "\n=== Parasitics for core_clock (mesh net) ==="
report_parasitics -net core_clock
