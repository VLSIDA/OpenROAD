# Distributed-RC CTS clock-tree SPICE deck for Ibex (v2 of write_ibex_cts_spice).
# Full clock elements: per-segment wire R (per-layer R/um from setRC.tcl),
# per-segment wire C split to both segment ends (pi model), per-FF sink nodes
# with individual Csink + measures (all 1938 FFs), inverter loads as caps.
# Vias are node merges (0 ohm) - coordinate-keyed nodes across layers.
set plat /home/wali2/backside/OpenROAD-flow-scripts/flow/platforms/gt2n
set here /home/wali2/backside/OpenROAD/src/cms/test/backside
set gt2n /home/wali2/backside/GT2N
set idir /home/wali2/backside/OpenROAD/src/cms/test/backside_result/Ibex/CTS
set out  $idir/ibex_cts_rc.sp

read_db $idir/ibex_cts_routed.odb
foreach lib [lsort [glob $plat/lib/gt2_6t_w*_tt_0p7v25c.lib]] { read_liberty $lib }

set block [ord::get_db_block]
set dbu   [$block getDbUnitsPerMicron]

# ---- per-layer wire R (ohm/um) and C (F/um) from setRC.tcl ----
array set rum {}; array set capum {}
set fh [open $plat/setRC.tcl r]
while {[gets $fh line] >= 0} {
  if {[regexp {set_layer_rc\s+-layer\s+(\S+)\s+-resistance\s+(\S+)\s+-capacitance\s+(\S+)} $line -> ln r c]} {
    set rum($ln) $r
    set capum($ln) [expr {$c * 1e-12}]
  }
}
close $fh

proc pin_cap {master pin} {
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
set bufs {}
set nets [list [$root_net getName]]
foreach inst [$block getInsts] {
  if {![string match "clkbuf_*" [$inst getName]]} { continue }
  set in ""; set out_n ""
  foreach it [$inst getITerms] {
    set n [$it getNet]; if {$n eq "NULL"} continue
    if {[[$it getMTerm] getIoType] eq "INPUT"}  { set in  [$n getName] }
    if {[[$it getMTerm] getIoType] eq "OUTPUT"} { set out_n [$n getName] }
  }
  if {$in eq "" || $out_n eq ""} { continue }
  lappend bufs [list [$inst getName] $in $out_n [[$inst getMaster] getName]]
  lappend nets $out_n
}
puts ">>> [llength $bufs] clock buffers, [llength $nets] tree nets"

# ---- walk every net's routed wire: R segments, node caps, pin bindings ----
set path   [odb::new_dbWirePath]
set pshape [odb::new_dbWirePathShape]
array set pin2node {}   ;# "<inst>/<pin>" or "BTERM/<name>" -> node
array set nodecap {}    ;# node -> F
set rlines {}
set nseg 0
set ni 0
foreach nname $nets {
  set net [$block findNet $nname]
  set wire [$net getWire]
  if {$wire eq "NULL"} { puts "ERROR: net $nname has no wire"; exit 1 }
  array unset xy2node
  set itr [odb::new_dbWirePathItr]
  odb::dbWirePathItr_begin $itr $wire

  proc nodeof {key ni} {
    upvar 1 xy2node xy2node netnodes netnodes nname nname
    if {![info exists xy2node($key)]} {
      set xy2node($key) "n${ni}_$key"
      lassign [split $key _] kx ky
      lappend netnodes($nname) [list "n${ni}_$key" $kx $ky]
    }
    return $xy2node($key)
  }
  if {![info exists netnodes($nname)]} { set netnodes($nname) {} }

  while {[odb::dbWirePathItr_getNextPath $itr $path]} {
    set pp [odb::dbWirePath_point_get $path]
    set px [$pp getX]; set py [$pp getY]
    set pnode [nodeof "${px}_${py}" $ni]
    set pit [odb::dbWirePath_iterm_get $path]
    if {$pit ne "NULL"} { set pin2node([$pit getName]) $pnode }
    set pbt [odb::dbWirePath_bterm_get $path]
    if {$pbt ne "NULL"} { set pin2node(BTERM/[$pbt getName]) $pnode }
    while {[odb::dbWirePathItr_getNextShape $itr $pshape]} {
      set sp [odb::dbWirePathShape_point_get $pshape]
      set sx [$sp getX]; set sy [$sp getY]
      set snode [nodeof "${sx}_${sy}" $ni]
      set ly [odb::dbWirePathShape_layer_get $pshape]
      set len [expr {abs($sx-$px) + abs($sy-$py)}]
      if {$len > 0 && $ly ne "NULL"} {
        set lname [$ly getName]
        if {[info exists rum($lname)]} {
          set lum [expr {double($len)/$dbu}]
          set rv [expr {$lum * $rum($lname)}]
          lappend rlines "Rw$nseg $pnode $snode [format %.5g $rv]"
          set cv [expr {$lum * $capum($lname) / 2.0}]
          foreach nd [list $pnode $snode] {
            if {![info exists nodecap($nd)]} { set nodecap($nd) 0.0 }
            set nodecap($nd) [expr {$nodecap($nd) + $cv}]
          }
          incr nseg
        }
      }
      set sit [odb::dbWirePathShape_iterm_get $pshape]
      if {$sit ne "NULL"} { set pin2node([$sit getName]) $snode }
      set sbt [odb::dbWirePathShape_bterm_get $pshape]
      if {$sbt ne "NULL"} { set pin2node(BTERM/[$sbt getName]) $snode }
      set px $sx; set py $sy; set pnode $snode
    }
  }
  odb::delete_dbWirePathItr $itr
  incr ni
}
puts ">>> $nseg wire segments, [array size nodecap] RC nodes, [array size pin2node] pins bound by decoder"

# ---- geometric fallback: bind every clock pin to the nearest same-net node ----
proc nearest_node {nname x y} {
  global netnodes
  set best ""; set bd 1e18
  foreach e $netnodes($nname) {
    lassign $e node kx ky
    set d [expr {abs($kx-$x) + abs($ky-$y)}]
    if {$d < $bd} { set bd $d; set best $node }
  }
  return $best
}
foreach nname $nets {
  set net [$block findNet $nname]
  foreach it [$net getITerms] {
    set pn [$it getName]
    if {[info exists pin2node($pn)]} { continue }
    lassign [$it getAvgXY] ok ax ay
    if {!$ok} { puts "ERROR: no location for $pn"; exit 1 }
    set pin2node($pn) [nearest_node $nname $ax $ay]
  }
}
if {![info exists pin2node(BTERM/clk_i)]} {
  set bt [$block findBTerm clk_i]
  set bx0 ""; foreach bp [$bt getBPins] { foreach bx [$bp getBoxes] {
    set cx [expr {([$bx xMin]+[$bx xMax])/2}]; set cy [expr {([$bx yMin]+[$bx yMax])/2}]
    set pin2node(BTERM/clk_i) [nearest_node [$root_net getName] $cx $cy]
  }}
}
puts ">>> pins bound total: [array size pin2node]"

# ---- classify net loads: FF sinks (measured) and other loads (cap only) ----
set sinks {}   ;# {node ffname cap}
set loads {}   ;# {node cap}
set miss 0
foreach nname $nets {
  set net [$block findNet $nname]
  foreach it [$net getITerms] {
    set inst [$it getInst]
    if {[string match "clkbuf_*" [$inst getName]]} { continue }
    if {[[$it getMTerm] getIoType] ne "INPUT"} { continue }
    set pn [$it getName]
    if {![info exists pin2node($pn)]} { incr miss; continue }
    set cap [pin_cap [[$inst getMaster] getName] [[$it getMTerm] getName]]
    if {[string match "*dffasync*" [[$inst getMaster] getName]]} {
      lappend sinks [list $pin2node($pn) [$inst getName] $cap]
    } else {
      lappend loads [list $pin2node($pn) $cap]
    }
  }
}
puts ">>> FF sinks: [llength $sinks], other loads: [llength $loads], unbound pins: $miss"
if {$miss > 0} { puts "ERROR: unbound load pins"; exit 1 }

# ---- emit deck ----
set f [open $out w]
puts $f "* CTS clock-tree SPICE deck v2 (ibex / gt2n) -- DISTRIBUTED wire RC"
puts $f "* per-segment R + pi-model C from routed layout; per-FF sink nodes;"
puts $f "* vias are ideal (node merge). Apples-to-apples with the mesh decks."
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
if {![info exists pin2node(BTERM/clk_i)]} { puts "ERROR: root bterm not bound"; exit 1 }
puts $f "* clock source at root bterm node"
puts $f "Vclk $pin2node(BTERM/clk_i) 0 PULSE(0 0.7 0.1n 0.01n 0.01n 0.5n 1.0n)"
puts $f ""
puts $f "* --- clock tree buffers (pins bound to routed-wire nodes) ---"
foreach b $bufs {
  lassign $b iname bin bout master
  set an "$iname/A"; set yn "$iname/Y"
  if {![info exists pin2node($an)] || ![info exists pin2node($yn)]} {
    puts "ERROR: buffer $iname pins unbound"; exit 1
  }
  puts $f "X$iname $pin2node($an) $pin2node($yn) VDD 0 $master"
}
puts $f ""
puts $f "* --- wire resistance segments ---"
foreach l $rlines { puts $f $l }
puts $f ""
puts $f "* --- wire capacitance (pi-model halves summed per node) ---"
set k 0
foreach nd [lsort [array names nodecap]] {
  puts $f "Cw$k $nd 0 [format %.6g $nodecap($nd)]"
  incr k
}
puts $f ""
puts $f "* --- non-FF clock loads (inverter gate caps) ---"
set k 0
foreach l $loads {
  lassign $l nd cap
  puts $f "Cload$k $nd 0 [format %.6g $cap]"
  incr k
}
puts $f ""
puts $f "* --- FF clock-pin caps (per sink, IID under MC) ---"
set k 0
foreach s $sinks {
  lassign $s nd ffname cap
  puts $f "Csink$k $nd 0 [format %.6g $cap] \$ $ffname"
  incr k
}
puts $f ""
puts $f ".tran 0.005n 2.1n"
puts $f "* --- FF clock-pin arrivals (ALL FFs individually) ---"
set k 0
foreach s $sinks {
  lassign $s nd ffname cap
  puts $f ".measure tran t_sink_$k WHEN v($nd)=0.35 RISE=2"
  puts $f ".measure tran slew_sink_$k TRIG v($nd)=0.07 RISE=2 TARG v($nd)=0.63 RISE=2"
  incr k
}
puts $f ".measure tran avg_power AVG power FROM=1.1n TO=2.1n"
puts $f ".end"
close $f
puts ">>> wrote $out  ([llength $sinks] FF measures)"
exit 0
