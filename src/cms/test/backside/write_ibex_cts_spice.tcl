# Write a CTS clock-tree SPICE deck for Ibex from the routed CTS odb.
# Same format as backside_result/Jpeg/CTS/jpeg_cts.sp: buffers = transistor
# subckts, wire + FF-pin caps lumped per net (no wire R - tree nets are short),
# root pulse, arrivals measured per leaf net on the SECOND rising edge.
# Measures named t_sink_*/slew_sink_* so mc/{make_mc,mc_skew}.py work unchanged.
set plat /home/wali2/backside/OpenROAD-flow-scripts/flow/platforms/gt2n
set here /home/wali2/backside/OpenROAD/src/cms/test/backside
set gt2n /home/wali2/backside/GT2N
set idir /home/wali2/backside/OpenROAD/src/cms/test/backside_result/Ibex/CTS
set out  $idir/ibex_cts.sp

read_db $idir/ibex_cts_routed.odb
foreach lib [lsort [glob $plat/lib/gt2_6t_w*_tt_0p7v25c.lib]] { read_liberty $lib }

set block [ord::get_db_block]
set dbu   [$block getDbUnitsPerMicron]

# ---- per-layer wire cap (F/um) from setRC.tcl (values are pF/um) ----
array set capum {}
set fh [open $plat/setRC.tcl r]
while {[gets $fh line] >= 0} {
  if {[regexp {set_layer_rc\s+-layer\s+(\S+)\s+-resistance\s+\S+\s+-capacitance\s+(\S+)} $line -> ln c]} {
    set capum($ln) [expr {$c * 1e-12}]
  }
}
close $fh

# ---- FF CLK pin caps from liberty (liberty unit = pF) ----
proc clk_pin_cap {master pin} {
  global pincap
  set key "$master/$pin"
  if {![info exists pincap($key)]} {
    set lp [get_lib_pins */$master/$pin]
    set pincap($key) [expr {[get_property [lindex $lp 0] capacitance] * 1e-12}]
  }
  return $pincap($key)
}

# ---- collect CTS buffers and tree nets ----
set root_net [[$block findBTerm clk_i] getNet]
set bufs {}   ;# {inst inNet outNet master}
set nets [list [$root_net getName]]
foreach inst [$block getInsts] {
  set iname [$inst getName]
  if {![string match "clkbuf_*" $iname]} { continue }
  set bin ""; set bout ""
  foreach it [$inst getITerms] {
    set mt [$it getMTerm]
    set n  [$it getNet]
    if {$n eq "NULL"} { continue }
    if {[$mt getIoType] eq "INPUT"}  { set bin  [$n getName] }
    if {[$mt getIoType] eq "OUTPUT"} { set bout [$n getName] }
  }
  if {$bin eq "" || $bout eq ""} { continue }
  lappend bufs [list $iname $bin $bout [[$inst getMaster] getName]]
  lappend nets $bout
}
puts ">>> [llength $bufs] clock buffers, [llength $nets] tree nets"

# ---- per-net: routed wire length per layer -> wire cap; FF pin caps; leafness ----
set path   [odb::new_dbWirePath]
set pshape [odb::new_dbWirePathShape]
set nff_total 0
foreach nname $nets {
  set net [$block findNet $nname]
  # wire cap from routed shapes
  set wcap 0.0
  set wire [$net getWire]
  if {$wire ne "NULL"} {
    set itr [odb::new_dbWirePathItr]
    odb::dbWirePathItr_begin $itr $wire
    while {[odb::dbWirePathItr_getNextPath $itr $path]} {
      set pp [odb::dbWirePath_point_get $path]
      set px [$pp getX]; set py [$pp getY]
      while {[odb::dbWirePathItr_getNextShape $itr $pshape]} {
        set sp [odb::dbWirePathShape_point_get $pshape]
        set sx [$sp getX]; set sy [$sp getY]
        set ly [odb::dbWirePathShape_layer_get $pshape]
        set len [expr {abs($sx-$px) + abs($sy-$py)}]
        if {$len > 0 && $ly ne "NULL"} {
          set lname [$ly getName]
          if {[info exists capum($lname)]} {
            set wcap [expr {$wcap + double($len)/$dbu * $capum($lname)}]
          }
        }
        set px $sx; set py $sy
      }
    }
    odb::delete_dbWirePathItr $itr
  }
  # FF (non-clkbuf) input pin caps on this net
  set fcap 0.0; set nff 0
  foreach it [$net getITerms] {
    set inst [$it getInst]
    if {[string match "clkbuf_*" [$inst getName]]} { continue }
    if {[[$it getMTerm] getIoType] ne "INPUT"} { continue }
    set fcap [expr {$fcap + [clk_pin_cap [[$inst getMaster] getName] [[$it getMTerm] getName]]}]
    incr nff
  }
  incr nff_total $nff
  set netwcap($nname) $wcap
  set netfcap($nname) $fcap
  set netff($nname)   $nff
}
puts ">>> FF sinks covered: $nff_total"

# ---- emit deck ----
set f [open $out w]
puts $f "* CTS clock-tree SPICE deck (ibex / gt2n) -- apples-to-apples with mesh deck"
puts $f "* buffers=transistor subckts, wire+FF caps lumped per net, root-driven"
puts $f ".option rshunt=1e12"
puts $f ".option method=gear"
puts $f ".option abstol=1e-10 reltol=0.003 vntol=1e-4"
puts $f ".option delmax=10p"
puts $f ".option autostop"
puts $f ".option measdgt=7"
puts $f ".param mc_mm_switch=0"
puts $f ".param mc_pr_switch=0"
puts $f ".include $here/gt2_w31_lvt_tt_renamed.sp"
puts $f ".include $gt2n/cdl/gt2_6t_w31_lvt.cdl"
puts $f ""
puts $f "Vvdd VDD 0 0.7"
puts $f "* clock source at tree root net '[$root_net getName]'"
puts $f "Vclk [$root_net getName] 0 PULSE(0 0.7 0.1n 0.01n 0.01n 0.5n 1.0n)"
puts $f ""
puts $f "* --- clock tree buffers ---"
foreach b $bufs {
  lassign $b iname bin bout master
  puts $f "X$iname $bin $bout VDD 0 $master"
}
puts $f ""
puts $f "* --- per-net lumped caps: C_<net> = routed wire cap, Csink<k> = FF CLK pin caps ---"
set sk 0
foreach nname $nets {
  puts $f "C_$nname $nname 0 [format %.6g $netwcap($nname)]"
  if {$netfcap($nname) > 0} {
    puts $f "Csink$sk $nname 0 [format %.6g $netfcap($nname)]"
    incr sk
  }
}
puts $f ""
puts $f ".tran 0.005n 2.1n"
puts $f "* --- FF clock-pin arrivals (per distinct leaf net; covers all FFs) ---"
set k 0
foreach nname $nets {
  if {$netff($nname) == 0} { continue }
  puts $f ".measure tran t_sink_$k WHEN v($nname)=0.35 RISE=2"
  puts $f ".measure tran slew_sink_$k TRIG v($nname)=0.07 RISE=2 TARG v($nname)=0.63 RISE=2"
  incr k
}
puts $f ".measure tran avg_power AVG power FROM=1.1n TO=2.1n"
puts $f ".end"
close $f
puts ">>> wrote $out  ($k leaf-net measures)"
exit 0
