set orfs    /home/wali2/backside/OpenROAD-flow-scripts/flow
set results $orfs/results/gt2n/gcd/base
set plat    $orfs/platforms/gt2n
read_db      $results/3_place.odb
read_liberty $plat/lib/gt2_6t_w31_lvt_tt_0p7v25c.lib
read_sdc     $results/3_place.sdc
source $plat/setRC.tcl
set b [ord::get_db_block]
set tech [ord::get_db_tech]
foreach L {BM4 BM3 BM2 BM1 BPR M0 M1 M2 M5} {
  set ly [$tech findLayer $L]
  if {$ly ne "NULL"} { puts "layer $L routingLevel=[$ly getRoutingLevel] backside=[$ly isBackside]" }
}
puts "BEFORE: minClk=[$b getMinLayerForClock] maxClk=[$b getMaxLayerForClock] minRt=[$b getMinRoutingLayer] maxRt=[$b getMaxRoutingLayer]"
set_routing_layers -signal M2-M5 -clock BM2-M5
puts "AFTER -clock BM2-M5: minClk=[$b getMinLayerForClock] maxClk=[$b getMaxLayerForClock] minRt=[$b getMinRoutingLayer] maxRt=[$b getMaxRoutingLayer]"
exit 0
