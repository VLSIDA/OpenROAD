# mesh_dynamic_power.tcl
# Computes complete dynamic wire + pin-cap power:
#
# P1 = CTS wire caps      (clknet_* nets)   +  mesh buf input pin caps
# P2 = Mesh output wire   (clk_mesh net)    +  FF clock pin caps
# P_dynamic_total = P1 + P2
#
# Run: openroad -exit mesh_dynamic_power.tcl

source "../helpers.tcl"

set orfs  /home/wali2/OpenROAD-flow-scripts/flow
set plat  $orfs/platforms/asap7
set res   $orfs/results/asap7/aes-block/base

set vdd   0.7
set freq  2.222e9

# ── Load ODB ──────────────────────────────────────────────────────────────
set odb_file [make_result_file "aes_block_mesh_v3.odb"]
puts "Loading ODB: $odb_file"
read_db $odb_file

# ── Load Liberty ──────────────────────────────────────────────────────────
foreach blk {aes-block_aes_rcon aes-block_aes_sbox} {
    set base $orfs/results/asap7/$blk/base
    set nick [string range $blk 10 end]
    catch { read_liberty $base/${nick}_typ.lib }
}
read_liberty $plat/lib/NLDM/asap7sc7p5t_AO_RVT_TT_nldm_211120.lib.gz
read_liberty $plat/lib/NLDM/asap7sc7p5t_INVBUF_RVT_TT_nldm_220122.lib.gz
read_liberty $plat/lib/NLDM/asap7sc7p5t_OA_RVT_TT_nldm_211120.lib.gz
read_liberty $plat/lib/NLDM/asap7sc7p5t_SIMPLE_RVT_TT_nldm_211120.lib.gz
read_liberty $plat/lib/NLDM/asap7sc7p5t_SEQ_RVT_TT_nldm_220123.lib

read_sdc $res/3_place.sdc
source $plat/setRC.tcl

# ── OpenRCX extraction ────────────────────────────────────────────────────
puts "Running OpenRCX extraction..."
extract_parasitics -ext_model_file $plat/rcx_patterns.rules \
    -cc_model 10 -coupling_threshold 0.1 -max_res 0 -skip_over_cell

# ── Write SPEF (wire caps) ────────────────────────────────────────────────
set spef_file [make_result_file "mesh_power.spef"]
puts "Writing SPEF: $spef_file"
write_spef $spef_file

set block [ord::get_db_block]

# ── Pin cap helper ────────────────────────────────────────────────────────
# Returns Liberty capacitance for a cell pin (in fF for ASAP7)
proc get_pin_cap_fF { master_name pin_name } {
    set lib_pins [get_lib_pins "${master_name}/${pin_name}"]
    if { [llength $lib_pins] == 0 } { return 0.0 }
    set cap [get_property [lindex $lib_pins 0] capacitance]
    if { [string is double -strict $cap] } { return $cap }
    return 0.0
}

# ── P1 pin caps: mesh buffer input pins (on clknet_* nets) ────────────────
puts "Computing mesh buffer input pin caps..."
set C_buf_pin_fF 0.0
set n_buf_pins   0

foreach net [$block getNets] {
    if { ![string match "clknet_*" [$net getName]] } { continue }
    foreach iterm [$net getITerms] {
        set inst [$iterm getInst]
        if { ![string match "mesh_buf_*" [$inst getName]] } { continue }
        set cap [get_pin_cap_fF \
            [[$inst getMaster] getName] \
            [[$iterm getMTerm] getName]]
        set C_buf_pin_fF [expr { $C_buf_pin_fF + $cap }]
        incr n_buf_pins
    }
}

# ── P2 pin caps: FF clock input pins (on clk_mesh net) ───────────────────
puts "Computing FF sink clock pin caps..."
set C_sink_pin_fF 0.0
set n_sinks       0

set mesh_net [$block findNet "clk_mesh"]
foreach iterm [$mesh_net getITerms] {
    set inst [$iterm getInst]
    # Skip mesh buffer OUTPUT pins — they drive clk_mesh, not receive from it
    if { [string match "mesh_buf_*" [$inst getName]] } { continue }
    set cap [get_pin_cap_fF \
        [[$inst getMaster] getName] \
        [[$iterm getMTerm] getName]]
    set C_sink_pin_fF [expr { $C_sink_pin_fF + $cap }]
    incr n_sinks
}

# ── Write pin cap summary ─────────────────────────────────────────────────
set pin_file [make_result_file "mesh_pin_caps.txt"]
set f [open $pin_file w]
puts $f "# Liberty pin caps (femtofarads, ASAP7 capacitive_load_unit = 1 fF)"
puts $f "BUF_PIN_CAP_fF  $C_buf_pin_fF"
puts $f "BUF_PIN_COUNT   $n_buf_pins"
puts $f "SINK_PIN_CAP_fF $C_sink_pin_fF"
puts $f "SINK_COUNT      $n_sinks"
puts $f "VDD             $vdd"
puts $f "FREQ_HZ         $freq"
close $f

puts "Pin caps written: $pin_file"
puts [format "  Mesh buf input pins : %d  total = %.2f fF" $n_buf_pins $C_buf_pin_fF]
puts [format "  FF sink clock pins  : %d  total = %.2f fF" $n_sinks    $C_sink_pin_fF]
puts ""
puts "Now run:"
puts "  python3 parse_spef_power.py $spef_file $pin_file"

exit 0
