#!/usr/bin/env python3
# Die-shrink / utilization sweep on JPEG: at each CORE_UTILIZATION step,
# re-floorplan + place, rebuild the balanced PDN, then build and FULLY route
# (DRT iterations, not first-pass) the BS+LCB and FS+LCB meshes at the
# Table III knobs (P3.2/x1/F6/x4, M5-M9, checkerboard). Metric: final DRC
# violation count; headline: smallest core where each architecture closes.
# Resumable: stages whose outputs exist are skipped. No HSpice needed.
#   usage: jpeg_shrink_sweep.py [--utils 40,45,50,55,60,65,70]
import argparse, os, re, subprocess
from subprocess import TimeoutExpired

BASE  = "/home/wali2/backside/OpenROAD/src/cms/test/backside_result"
BSD   = "/home/wali2/backside/OpenROAD/src/cms/test/backside"
ORFS  = "/home/wali2/backside/OpenROAD-flow-scripts/flow"
ORD   = "/home/wali2/backside/OpenROAD/build/bin/openroad"
SWEEP = f"{BASE}/Jpeg/shrink"
KNOBS = dict(PITCH="3.2", MBUF="gt2_6t_buf_x1_w31_lvt", FMAX="6",
             SBUF="gt2_6t_buf_x4_w31_lvt", CLKLAYERS="M5-M9", CHECKER="1",
             FEEDER="full", DESIGN="jpeg", DRTITER="25")
SYNTH_FILES = ["1_synth.odb", "1_synth.sdc", "1_2_yosys.v", "1_2_yosys.sdc",
               "1_synth.vars", "mem.json", "1_1_yosys_canonicalize.rtlil"]

def sh(cmd, log, cwd=None, env=None, timeout=14400):
    with open(log, "a") as lf:
        return subprocess.run(cmd, shell=isinstance(cmd, str), stdout=lf,
                              stderr=subprocess.STDOUT, cwd=cwd, env=env,
                              timeout=timeout).returncode

def last_drc(log):
    n = None
    for l in open(log, errors="ignore"):
        m = re.search(r"Number of violations = (\d+)", l)
        if m:
            n = int(m.group(1))
    return n

def die_side(work):
    tcl = f"{work}/die.tcl"
    open(tcl, "w").write(
        f"read_db {work}/results/gt2n/jpeg/base/3_place.odb\n"
        "set b [ord::get_db_block]\n"
        "set dbu [$b getDbUnitsPerMicron]\n"
        "set d [$b getDieArea]\n"
        "puts \"DIE [expr {[$d dx]/double($dbu)}] [expr {[$d dy]/double($dbu)}]\"\n"
        "exit 0\n")
    out = subprocess.run([ORD, "-exit", tcl], capture_output=True, text=True)
    m = re.search(r"DIE ([\d.]+) ([\d.]+)", out.stdout)
    return (float(m.group(1)), float(m.group(2))) if m else (0, 0)

def run_util(u):
    work = f"{SWEEP}/u{u}"
    res = f"{work}/results/gt2n/jpeg/base"
    os.makedirs(res, exist_ok=True)
    log = f"{work}/sweep.log"
    # 1. stage the golden synth outputs (schema-matched: built with our binary)
    for f in SYNTH_FILES:
        src = f"{ORFS}/results/gt2n/jpeg/base/{f}"
        dst = f"{res}/{f}"
        if os.path.exists(src) and not os.path.exists(dst):
            sh(f"cp {src} {dst}", log)
    # 2. floorplan + place at this utilization
    if not os.path.exists(f"{res}/3_place.odb"):
        r = sh(["make", "-C", ORFS,
                "DESIGN_CONFIG=./designs/gt2n/jpeg/config.mk",
                f"WORK_HOME={work}", f"CORE_UTILIZATION={u}",
                f"OPENROAD_EXE={ORD}",
                "YOSYS_EXE=/home/wali2/OpenROAD-flow-scripts/tools/install/yosys/bin/yosys",
                "floorplan", "place"], log)
        if r != 0 or not os.path.exists(f"{res}/3_place.odb"):
            return dict(util=u, status="place_failed")
    dx, dy = die_side(work)
    # 3. balanced PDN on this floorplan
    pdn = f"{work}/jpeg_pdn_balanced.odb"
    if not os.path.exists(pdn):
        env = dict(os.environ, DESIGN="jpeg",
                   PDN_W="1.5", PDN_N="4",   # match the July baseline grid
                   PLACE_ODB=f"{res}/3_place.odb", PLACE_SDC=f"{res}/3_place.sdc",
                   OUT_ODB=pdn)
        r = sh([ORD, "-threads", "32", "-exit", "pdn_balance.tcl"],
               log, cwd=BSD, env=env)
        if r != 0 or not os.path.exists(pdn):
            return dict(util=u, status="pdn_failed", die=f"{dx:.1f}x{dy:.1f}")
    # 4. both mesh architectures, full detailed route
    row = dict(util=u, status="ok", die=f"{dx:.1f}x{dy:.1f}")
    for variant, flow, extra in [
            ("bs", "multi_backside_baltree.tcl", dict(TAPROUTE="router")),
            ("fslcb", "multi_frontside_baltree.tcl",
             dict(SINKTIER="lcb", MESHLAYERS="M5M6",
                  MERGEROUTE="1", SKIPDECK="1"))]:
        out = f"{work}/{variant}"
        os.makedirs(out, exist_ok=True)
        olog = f"{out}/openroad.log"
        tmark = f"{out}/TIMEOUT"
        done = os.path.exists(olog) and (
            last_drc(olog) is not None
            or "GRT-0116" in open(olog, errors="ignore").read()
            or os.path.exists(tmark))
        if not done:
            env = dict(os.environ, **KNOBS, **extra,
                       PDNODB=pdn, PLACE_SDC=f"{res}/3_place.sdc", OUTDIR=out)
            try:
                sh([ORD, "-threads", "32", "-exit", f"{BASE}/{flow}"],
                   olog, cwd=f"{BASE}/Jpeg", env=env)
            except TimeoutExpired:
                open(tmark, "w").write("flow exceeded timeout\n")
        if os.path.exists(tmark):
            row[variant + "_drc"] = "timeout"
        elif os.path.exists(olog) and "GRT-0116" in open(olog, errors="ignore").read():
            row[variant + "_drc"] = "grt_fail"
        else:
            row[variant + "_drc"] = last_drc(olog) if os.path.exists(olog) else None
    return row

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--utils", default="40,45,50,55,60,65,70")
    a = ap.parse_args()
    os.makedirs(SWEEP, exist_ok=True)
    csv = f"{SWEEP}/shrink.csv"
    with open(csv, "w") as f:
        f.write("util,die,status,bs_drc,fslcb_drc\n")
        for u in [int(x) for x in a.utils.split(",")]:
            r = run_util(u)
            line = (f"{r['util']},{r.get('die','')},{r['status']},"
                    f"{r.get('bs_drc','')},{r.get('fslcb_drc','')}")
            print(line, flush=True)
            f.write(line + "\n")
    print(f"wrote {csv}")

if __name__ == "__main__":
    main()
