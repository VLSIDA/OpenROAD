# Placement test for nTSV_tap in the real gcd design: drop several taps onto
# occupied locations, then legalize -> they must land on legal sites in
# whitespace with no overlap.
set orfs    /home/wali2/backside/OpenROAD-flow-scripts/flow
set results $orfs/results/gt2n/gcd/base
set plat    $orfs/platforms/gt2n
read_db      $results/3_place.odb
read_liberty $plat/lib/gt2_6t_w31_lvt_tt_0p7v25c.lib
read_sdc     $results/3_place.sdc

set db [ord::get_db]
set b  [ord::get_db_block]
set m  [$db findMaster nTSV_tap]
puts ">>> nTSV_tap master in odb: [expr {$m eq {NULL} ? {NO} : {YES}}]"

# Drop 6 taps at interior points (many on top of existing cells).
set pts {5000 5000 8000 8000 11000 6000 6000 11000 9000 12000 12000 9000}
set k 0
foreach {x y} $pts {
  incr k
  set inst [odb::dbInst_create $b $m "ntsv_tap_$k"]
  $inst setLocation $x $y
  $inst setPlacementStatus PLACED
  set bb [$inst getBBox]
  puts ">>> tap $k dropped at ([$bb xMin],[$bb yMin])"
}

detailed_placement
check_placement -verbose

puts ">>> after legalize:"
for {set i 1} {$i <= $k} {incr i} {
  set inst [$b findInst "ntsv_tap_$i"]
  set bb [$inst getBBox]
  puts "    tap $i -> ([$bb xMin],[$bb yMin])-([$bb xMax],[$bb yMax]) status=[$inst getPlacementStatus]"
}
exit 0
