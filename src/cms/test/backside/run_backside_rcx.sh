#!/usr/bin/env bash
set -e
cd /home/wali2/backside/OpenROAD/src/cms/test/backside
echo "=== [1/2] regenerate backside deck with field-solved mesh caps ==="
/home/wali2/backside/OpenROAD/build/bin/openroad -threads 72 -exit jpeg_spice_skew.tcl > jpeg_spice.log 2>&1
echo "openroad exit: $?"
grep -E "mesh-cap override|CMS-0895|CMS-0898|SPICE deck|DRT-0206" jpeg_spice.log | tail -8
echo "=== [2/2] hspice ==="
cd results
hspice jpeg_mesh_skew.sp -mt 16 -o jpeg_mesh_skew > jpeg_hspice.log 2>&1
echo "hspice exit: $?"
grep -iE "concluded|error" jpeg_hspice.log | tail -3
echo "=== DONE ==="
