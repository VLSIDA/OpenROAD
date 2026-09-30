# Static IR-drop ONLY, on the clean baseline gcd. The well-tap cell declares
# vdd/vss on M1 (and M0) that are NOT via-connected to its BPR pin in the
# abstract -> PSM sees them as floating frontside islands and aborts with
# PSM-0069. Taps carry negligible current, so we delete those frontside power
# shapes from the tap master IN THIS SESSION (the edit doesn't survive write_db,
# so strip + analyze must happen together) and then run IR on the real BPR/BM
# power network.
set plat /home/wali2/backside/OpenROAD-flow-scripts/flow/platforms/gt2n
set res  /home/wali2/backside/OpenROAD-flow-scripts/flow/results/gt2n/gcd/base

read_db      $res/6_final.odb
read_liberty $plat/lib/gt2_6t_w31_lvt_tt_0p7v25c.lib
read_sdc     $res/6_final.sdc

# --- strip floating frontside (M1/M0) power shapes from the tap master ---
set db  [ord::get_db]
set tap [$db findMaster gt2_6t_tapbspdn_w31_lvt]
set killed 0
foreach mt [$tap getMTerms] {
  set nm [$mt getName]
  if {$nm ne "vdd" && $nm ne "vss"} { continue }
  set dead {}
  foreach mp [$mt getMPins] {
    foreach b [$mp getGeometry] {
      set L [$b getTechLayer]
      if {$L eq "NULL"} { continue }
      set ln [$L getName]
      if {$ln eq "M1" || $ln eq "M0"} { lappend dead $b }
    }
  }
  foreach b $dead { odb::dbBox_destroy $b; incr killed }
}
puts ">>> stripped $killed frontside tap power boxes"

set_pdnsim_net_voltage -net vdd -voltage 0.7
set_pdnsim_net_voltage -net vss -voltage 0.0

# Supply enters on the BACKSIDE (BM4 straps). PSM's auto source (STRAPS/FULL)
# keys off the highest-numbered = frontside layer, so it never lands on BM4 and
# finds no source. Feed source points placed ON the BM4 straps via -vsrc.
puts "================== VDD =================="
analyze_power_grid -net vdd -vsrc /tmp/vdd.vsrc
puts "================== VSS =================="
analyze_power_grid -net vss -vsrc /tmp/vss.vsrc
exit 0
