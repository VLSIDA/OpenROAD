#!/usr/bin/env bash
set -e
cd /home/wali2/backside/OpenROAD/src/cms/test/backside
/home/wali2/backside/OpenROAD/build/bin/openroad -threads 72 -exit jpeg_backside_26x26.tcl > jpeg_bk26_f16.log 2>&1
echo "flow done: $(grep -E 'CMS-0703' jpeg_bk26_f16.log | tail -1)"
echo "DRT-0206=$(grep -c DRT-0206 jpeg_bk26_f16.log) DPL-0036=$(grep -c DPL-0036 jpeg_bk26_f16.log)"
cd results
hspice jpeg_backside_26_skew.sp -mt 16 -o jpeg_bk_f16 > jpeg_bk_f16_hs.log 2>&1
echo "F16 DONE"
