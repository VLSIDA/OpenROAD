# Approach B v2 (macro + re-PDN): place TSVs as CLASS BLOCK macros at the regular
# mesh grid (no strap-shift), then DELETE the existing PDN and re-run pdngen so it
# breaks the BPR followpins at the macros and routes the BM3/BM4 straps AROUND
# them -- no manual surgery, no analytic-strap matching. Then relocate std cells.
set orfs /home/wali2/backside/OpenROAD-flow-scripts/flow
set res  $orfs/results/gt2n/gcd/base
set plat $orfs/platforms/gt2n

read_db  $res/3_place.odb
read_lef gt2_6t_TSV_block.lef     ;# CLASS BLOCK variant -> pdngen macro

# Place 64 TSVs at the REGULAR grid (strap_width 0 => no deform/shift => 1 mesh).
reserve_clock_mesh -h_layer BM2 -v_layer BM1 -pitch 1.0 \
    -strap_pitch 2.16 -strap_offset 1.08 -strap_width 0 \
    -tsv_master gt2_6t_TSV -spacing 0 -keepout_w 0.1 -keepout_h 0.144

set block [ord::get_db_block]

# The BLOCK macro drives both placement-avoidance and pdngen, so strip reserve's
# manual keepouts (the BPR obstruction in particular would make pdngen drop rails).
set nb 0; foreach b [$block getBlockages]   { odb::dbBlockage_destroy $b;   incr nb }
set no 0; foreach o [$block getObstructions] { odb::dbObstruction_destroy $o; incr no }
puts ">>> stripped $nb placement blockages, $no obstructions"

# Clear the existing PDN special wires so pdngen rebuilds around the macros.
foreach nn {vdd vss} {
  set net [$block findNet $nn]
  set sws {}
  foreach sw [$net getSWires] { lappend sws $sw }
  foreach sw $sws { odb::dbSWire_destroy $sw }
}
puts ">>> cleared old PDN"

# Re-run PDN: pdngen cuts BPR at the TSV macros + routes straps around them.
source $plat/pdn.tcl
pdngen

# Relocate std cells that overlap the TSV macros.
detailed_placement

write_db results/gcd_postplace_B2.odb
puts ">>> approach B (macro + re-PDN) done: results/gcd_postplace_B2.odb"
exit 0
