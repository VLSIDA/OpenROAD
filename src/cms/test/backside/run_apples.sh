#!/usr/bin/env bash
set -e
cd /home/wali2/backside/OpenROAD/src/cms/test/backside
echo "[1] regenerate x4 backside deck"
/home/wali2/backside/OpenROAD/build/bin/openroad -threads 72 -exit jpeg_spice_skew.tcl > ap_flow.log 2>&1
echo "flow: $(grep -c DRT-0206 ap_flow.log) DRT-0206"
cd results
echo "[2] baseline (x4 backside)"
hspice jpeg_mesh_skew.sp -mt 16 -o ap_base > ap_base.log 2>&1
echo "[3] no-sink-tier (short sink buffers)"
python3 -c "
import re
o=[];n=0
for l in open('jpeg_mesh_skew.sp'):
    m=re.match(r'(Xsink_buf_\S+)\s+(\S+)\s+(\S+)\s+VDD\s+0\s+\S+',l.rstrip())
    if m: o.append(f'Rsb_{n} {m.group(2)} {m.group(3)} 1e-3'); n+=1
    else: o.append(l.rstrip())
open('ap_nosink.sp','w').write('\n'.join(o)+'\n'); print('shorted',n)
"
hspice ap_nosink.sp -mt 16 -o ap_nosink > ap_nosink.log 2>&1
echo "APPLES DONE"
