source "helpers.tcl"
read_lef Nangate45/Nangate45_tech.lef
read_lef Nangate45_data/Nangate45_stdcell.lef
read_def disallow_one_site_gaps.def

set def_file [make_result_file disallow_one_site_gaps2.def]

place_endcaps -endcap_vertical "TAPCELL_X2"
place_tapcells -distance 20 \
  -master "TAPCELL_X2"

write_def $def_file

diff_file disallow_one_site_gaps.defok $def_file
