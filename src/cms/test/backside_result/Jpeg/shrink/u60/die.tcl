read_db /home/wali2/backside/OpenROAD/src/cms/test/backside_result/Jpeg/shrink/u60/results/gt2n/jpeg/base/3_place.odb
set b [ord::get_db_block]
set dbu [$b getDbUnitsPerMicron]
set d [$b getDieArea]
puts "DIE [expr {[$d dx]/double($dbu)}] [expr {[$d dy]/double($dbu)}]"
exit 0
