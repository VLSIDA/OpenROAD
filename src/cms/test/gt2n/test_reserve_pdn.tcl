# Verify the auto-BPR-break premise: run tapcell + PDN on the reserved odb and
# check (1) no tap lands in a keepout, (2) BPR followpins break at the keepouts.
set orfs /home/wali2/backside/OpenROAD-flow-scripts/flow
set plat $orfs/platforms/gt2n

read_db results/gcd_reserve.odb

# Tapcells (should skip the placement blockages we created).
tapcell -distance 5 -tapcell_master gt2_6t_tapbspdn_w31_lvt

# PDN: platform grid definition + generate.
source $plat/pdn.tcl
pdngen

write_db results/gcd_reserve_pdn.odb
puts ">>> tapcell + pdn done: results/gcd_reserve_pdn.odb"
exit 0
