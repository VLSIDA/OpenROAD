# Minimal placement test for the nTSV_tap cell:
#  - LEFs load (the M0<->BM1 via inside the pin is accepted)
#  - the cell instantiates and the legalizer places it on a row, no overlap.
set lef /home/wali2/backside/OpenROAD-flow-scripts/flow/platforms/gt2n/lef
read_lef $lef/gt2_tech.lef
read_lef $lef/gt2_6t_w31_lvt.lef
read_lef $lef/gt2_ntsv_tap.lef

set db [ord::get_db]
set m [$db findMaster nTSV_tap]
puts ">>> master nTSV_tap loaded: [expr {$m eq "NULL" ? "NO" : "YES"}]  size=[[$m getWidth]]x[[$m getHeight]]"
puts ">>> CLK pin layers:"
foreach mt [$m getMTerms] {
  foreach mp [$mt getMPins] { foreach g [$mp getGeometry] {
    set ly [$g getTechLayer]; if {$ly ne "NULL"} { puts "    [$mt getName] -> [$ly getName]" }
  }}
}

# small floorplan with rows on the gt2_6t site
initialize_floorplan -die_area "0 0 5 5" -core_area "0.3 0.3 4.7 4.7" -site gt2_6t
set b [ord::get_db_block]
puts ">>> rows created: [llength [$b getRows]]"

# instantiate: 3 buffers + 1 tap, all dumped at the same spot to force the
# legalizer to spread them (tests no-overlap).
set buf [$db findMaster gt2_6t_buf_x4_w31_lvt]
for {set i 0} {$i < 3} {incr i} {
  set inst [odb::dbInst_create $b $buf "buf_$i"]
  $inst setLocation 2000 2000
  $inst setPlacementStatus PLACED
}
set tap [odb::dbInst_create $b $m "tap1"]
$tap setLocation 2000 2000
$tap setPlacementStatus PLACED

# legalize
detailed_placement
check_placement -verbose

set bb [$tap getBBox]
puts ">>> tap1 final: ([$bb xMin],[$bb yMin])-([$bb xMax],[$bb yMax]) status=[$tap getPlacementStatus]"
foreach i {0 1 2} {
  set inst [$b findInst "buf_$i"]; set ib [$inst getBBox]
  puts ">>> buf_$i final: ([$ib xMin],[$ib yMin])-([$ib xMax],[$ib yMax])"
}
exit 0
