# Export everything the distributed-RC tree deck builder needs:
#   cts_pins.csv  - every pin on every tree net (location, liberty cap, kind)
#   cts_bufs.csv  - buffer instance, in-net, out-net, master
#   cts_root.txt  - root net name + bterm location
#   ibex_cts.def  - complete routing (ground truth for wire segments)
set plat /home/wali2/backside/OpenROAD-flow-scripts/flow/platforms/gt2n
set idir /home/wali2/backside/OpenROAD/src/cms/test/backside_result/Ibex/CTS

read_db $idir/ibex_cts_routed.odb
foreach lib [lsort [glob $plat/lib/gt2_6t_w*_tt_0p7v25c.lib]] { read_liberty $lib }
set block [ord::get_db_block]

proc pin_cap {master pin} {
  global pincap
  set key "$master/$pin"
  if {![info exists pincap($key)]} {
    set lp [get_lib_pins */$master/$pin]
    set pincap($key) [expr {[get_property [lindex $lp 0] capacitance] * 1e-12}]
  }
  return $pincap($key)
}

set root_net [[$block findBTerm clk_i] getNet]
set nets [list [$root_net getName]]
set fb [open $idir/cts_bufs.csv w]
foreach inst [$block getInsts] {
  if {![string match "clkbuf_*" [$inst getName]]} { continue }
  set in ""; set out_n ""
  foreach it [$inst getITerms] {
    set n [$it getNet]; if {$n eq "NULL"} continue
    if {[[$it getMTerm] getIoType] eq "INPUT"}  { set in  [$n getName] }
    if {[[$it getMTerm] getIoType] eq "OUTPUT"} { set out_n [$n getName] }
  }
  if {$in eq "" || $out_n eq ""} { continue }
  puts $fb "[$inst getName]|$in|$out_n|[[$inst getMaster] getName]"
  lappend nets $out_n
}
close $fb

set fp [open $idir/cts_pins.csv w]
foreach nname $nets {
  set net [$block findNet $nname]
  foreach it [$net getITerms] {
    set inst [$it getInst]
    set master [[$inst getMaster] getName]
    set mt [[$it getMTerm] getName]
    lassign [$it getAvgXY] ok ax ay
    if {!$ok} { puts "ERROR: no xy for [$it getName]"; exit 1 }
    if {[string match "clkbuf_*" [$inst getName]]} {
      set kind [expr {[[$it getMTerm] getIoType] eq "OUTPUT" ? "bufY" : "bufA"}]
      set cap 0
    } elseif {[[$it getMTerm] getIoType] ne "INPUT"} {
      continue
    } elseif {[string match "*dffasync*" $master]} {
      set kind ff; set cap [pin_cap $master $mt]
    } else {
      set kind load; set cap [pin_cap $master $mt]
    }
    puts $fp "[$inst getName]|$nname|$ax|$ay|$cap|$kind"
  }
}
close $fp

set bt [$block findBTerm clk_i]
set fr [open $idir/cts_root.txt w]
foreach bp [$bt getBPins] { foreach bx [$bp getBoxes] {
  puts $fr "[$root_net getName]|[expr {([$bx xMin]+[$bx xMax])/2}]|[expr {([$bx yMin]+[$bx yMax])/2}]"
}}
close $fr

write_def $idir/ibex_cts.def
puts ">>> exported [llength $nets] nets"
exit 0
