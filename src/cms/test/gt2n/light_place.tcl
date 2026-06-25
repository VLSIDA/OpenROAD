# Fast placement: global + detailed only (NO timing-driven resizer/repair, which
# loops pathologically with the tap liberty). Produces a legal 3_place.odb good
# enough to exercise the backside-clock flow.
set orfs    /home/wali2/backside/OpenROAD-flow-scripts/flow
set results $orfs/results/gt2n/gcd/base
set plat    $orfs/platforms/gt2n
read_db      $results/2_floorplan.odb
read_liberty $plat/lib/gt2_6t_w31_lvt_tt_0p7v25c.lib
read_liberty $plat/lib/gt2_ntsv_tap.lib
read_sdc     $results/2_floorplan.sdc
source $plat/setRC.tcl
global_placement -density 0.65
detailed_placement
write_db $results/3_place.odb
puts ">>> light place done: 3_place.odb"
exit 0
