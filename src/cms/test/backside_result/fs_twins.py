#!/usr/bin/env python3
# For every Pareto point of a design's CHECKERBOARD backside study, run the
# two frontside twins at the same knobs (CHECKER=1, mesh on M5/M6, FULL deck):
#   fslcb    - frontside mesh + LCB tier (SINKTIER=lcb, same FMAX/SBUF)
#   fsdirect - frontside mesh, FFs attached directly (no tier)
# Results are cached by output dir (re-running only executes missing twins),
# so this can be re-run as the backside frontier grows.
#   usage: fs_twins.py <ibex|jpeg|minimax|floonoc> [--threads N] [--hspice_mt N]
import argparse, os, re, statistics, subprocess, sys, time

import optuna

BASE = "/home/wali2/backside/OpenROAD/src/cms/test/backside_result"
ORD  = "/home/wali2/backside/OpenROAD/build/bin/openroad"
FLOW = f"{BASE}/multi_frontside_baltree.tcl"

CFG = {
    "ibex":    dict(dir="Ibex",    study="ibex_checker_mesh_dse",
                    db="dse_checker/ibex_checker_dse.db",    sinks=1938),
    "jpeg":    dict(dir="Jpeg",    study="jpeg_checker_mesh_dse",
                    db="dse_checker/jpeg_checker_dse.db",    sinks=4384),
    "minimax": dict(dir="Minimax", study="minimax_checker_mesh_dse",
                    db="dse_checker/minimax_checker_dse.db", sinks=2251),
    "floonoc": dict(dir="Floonoc", study="floonoc_checker_mesh_dse",
                    db="dse_checker/floonoc_checker_dse.db", sinks=15311),
}

def pareto(trials):
    done = [t for t in trials if t.state.name == "COMPLETE" and t.values]
    seen, uniq = set(), []
    for t in done:
        k = tuple(sorted(t.params.items()))
        if k not in seen:
            seen.add(k)
            uniq.append(t)
    front = [t for t in uniq if not any(
        o.values[0] <= t.values[0] and o.values[1] <= t.values[1]
        and o.values != t.values for o in uniq)]
    front.sort(key=lambda t: t.values[0])
    return front

def parse_lis(lis):
    unit = {"p": 1e-12, "n": 1e-9, "f": 1e-15, "u": 1e-6, "m": 1e-3, "": 1}
    ts, ss = [], []
    pw = None
    for l in open(lis, errors="ignore"):
        m = re.match(r"\s*(t_sink|slew_sink)_\d+=\s*([-0-9.eE+]+)([a-z]?)", l)
        if m:
            v = float(m.group(2)) * unit.get(m.group(3), 1)
            if 0 < v < 1e-6:
                (ts if m.group(1) == "t_sink" else ss).append(v * 1e12)
        m = re.match(r"\s*avg_power=\s*([-0-9.eE+]+)([a-z]?)", l)
        if m:
            pw = float(m.group(1)) * unit.get(m.group(2), 1) * 1e3
    if not ts:
        return None
    ts.sort()
    n = len(ts)
    return dict(n=n, skew=ts[-1] - ts[0],
                p99=ts[int(0.99 * n)] - ts[int(0.01 * n)],
                ins=statistics.mean(ts), pw=pw,
                slew_max=max(ss) if ss else 0,
                slew_mean=statistics.mean(ss) if ss else 0)

def run_twin(design, cfg, p, variant, threads, hsp_mt):
    ddir = f"{BASE}/{cfg['dir']}"
    tag = (f"p{str(p['PITCH']).replace('.', 'p')}_{p['MBUF']}"
           + (f"_f{p['FMAX']}_{p['SBUF']}" if variant == "fslcb" else "")
           + f"_m5m6_{p['CLKLAYERS'].replace('-', '').lower()}_ck1")
    out = f"{ddir}/fs_checker/{variant}_{tag}"
    os.makedirs(out, exist_ok=True)
    lis = f"{out}/{design}_frontside_full.lis"
    if not os.path.exists(lis):
        env = dict(os.environ, DESIGN=design, FEEDER="full", CHECKER="1",
                   PITCH=str(p["PITCH"]),
                   MBUF=f"gt2_6t_buf_{p['MBUF']}_w31_lvt",
                   MESHLAYERS="M5M6", CLKLAYERS=p["CLKLAYERS"], OUTDIR=out)
        if variant == "fslcb":
            env.update(SINKTIER="lcb", FMAX=str(p["FMAX"]),
                       SBUF=f"gt2_6t_buf_{p['SBUF']}_w31_lvt")
        log = f"{out}/openroad.log"
        if not os.path.exists(f"{out}/{design}_frontside_full.sp"):
            with open(log, "w") as lf:
                r = subprocess.run([ORD, "-threads", str(threads), "-exit", FLOW],
                                   stdout=lf, stderr=subprocess.STDOUT,
                                   env=env, cwd=ddir, timeout=14400)
            if r.returncode != 0:
                return dict(status="flow_failed", outdir=out)
        for _att in range(6):
            subprocess.run(
                f"hspice {design}_frontside_full.sp -mt {hsp_mt}"
                f" -o {design}_frontside_full > hspice.log 2>&1",
                shell=True, cwd=out, timeout=14400)
            if os.path.exists(lis):
                if "Licensed number of users" not in open(lis, errors="ignore").read():
                    break
                os.remove(lis)
            time.sleep(180)
    if not os.path.exists(lis):
        return dict(status="hspice_failed", outdir=out)
    m = parse_lis(lis)
    if not m or m["n"] != cfg["sinks"]:
        return dict(status=f"bad_sinks({m['n'] if m else 0})", outdir=out)
    m.update(status="ok", outdir=out)
    return m

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("design", choices=CFG.keys())
    ap.add_argument("--threads", type=int, default=32)
    ap.add_argument("--hspice_mt", type=int, default=8)
    a = ap.parse_args()
    cfg = CFG[a.design]
    st = optuna.load_study(study_name=cfg["study"],
                           storage=f"sqlite:///{BASE}/{cfg['dir']}/{cfg['db']}")
    front = pareto(st.trials)
    print(f"{a.design}: {len(front)} Pareto points -> up to {2*len(front)} twins")
    csv = f"{BASE}/{cfg['dir']}/fs_checker/twins.csv"
    os.makedirs(os.path.dirname(csv), exist_ok=True)
    seen_direct = set()
    with open(csv, "w") as f:
        f.write("variant,pitch,mbuf,fmax,sbuf,clklayers,"
                "bs_skew,bs_pw,status,skew,p99,ins,pw,slew_max,slew_mean\n")
        for t in front:
            p = t.params
            for variant in ("fslcb", "fsdirect"):
                if variant == "fsdirect":
                    key = (p["PITCH"], p["MBUF"], p["CLKLAYERS"])
                    if key in seen_direct:
                        continue
                    seen_direct.add(key)
                r = run_twin(a.design, cfg, p, variant, a.threads, a.hspice_mt)
                row = (f"{variant},{p['PITCH']},{p['MBUF']},{p['FMAX']},"
                       f"{p['SBUF']},{p['CLKLAYERS']},"
                       f"{t.values[0]:.2f},{t.values[1]:.4f},{r['status']},"
                       + (f"{r['skew']:.2f},{r['p99']:.2f},{r['ins']:.1f},"
                          f"{r['pw']:.4f},{r['slew_max']:.1f},{r['slew_mean']:.1f}"
                          if r["status"] == "ok" else ",,,,,"))
                print(row, flush=True)
                f.write(row + "\n")
    print(f"wrote {csv}")

if __name__ == "__main__":
    main()
