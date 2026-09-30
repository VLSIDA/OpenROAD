#!/bin/bash
cd /home/wali2/backside/OpenROAD/src/cms/test/backside
ORD=/home/wali2/backside/OpenROAD/build/bin/openroad
$ORD -threads 72 -exit jpeg_backside_26x26.tcl > jpeg_bk26_f8.log 2>&1
grep -E "CMS-0703|DRT-0206|DPL-0036" jpeg_bk26_f8.log | tail -5
cd results
cp jpeg_backside_26_skew.sp jpeg_bk_f8.sp
hspice jpeg_bk_f8.sp -o jpeg_bk_f8 > hspice_f8.log 2>&1
echo "hspice exit: $?"
echo "F8 DONE"
