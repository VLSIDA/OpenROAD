#!/usr/bin/env python3
# Multi-objective Bayesian DSE for the JPEG backside clock mesh
# CHECKERBOARD ALWAYS ON (CHECKER=1, half drivers/TSVs); FULL-TREE deck basis
# (root pulse, feeder simulated, power includes the feeder; analytic BM caps).
# Same harness as Ibex/ibex_bo_dse.py, retargeted:
#   objectives : minimize (worst-case skew [ps], clock power [mW])  -- from HSpice
#   knobs      : PITCH, MBUF (mesh driver), FMAX (LCB fanout), SBUF (LCB size),
#                CLKLAYERS (clock routing window)
#   basis      : TAPROUTE=router (sink taps routed by GRT/DRT), full LVT feeder
#   infeasible : route failure / FFs dropped from deck / sinks not toggling /
#                worst sink slew over budget  -> optuna.TrialPruned
#
# NOTE: starts UNSEEDED -- all prior JPEG runs are on the old (authored-tap,
# M2-M9) basis and would poison the model. TPE's first ~10 trials are random
# startup anyway.
#
# Run with the dse venv python:
#   /home/wali2/backside/dse_venv/bin/python jpeg_bo_dse.py --trials 300 --jobs 2
# Pareto set:  --report prints study.best_trials and writes dse/pareto.csv.
import argparse, os, re, subprocess, sys, glob, time

import optuna
from optuna.distributions import CategoricalDistribution  # noqa: F401 (parity)

BASE   = "/home/wali2/backside/OpenROAD/src/cms/test/backside_result"
JPEG   = f"{BASE}/Jpeg"
FLOW   = f"{BASE}/multi_backside_baltree.tcl"
ORD    = "/home/wali2/backside/OpenROAD/build/bin/openroad"
DSEDIR = f"{JPEG}/dse_checker"
DB     = f"sqlite:///{DSEDIR}/jpeg_checker_dse.db"
sys.path.insert(0, f"{BASE}/Jpeg")
from lis_metrics import metrics  # noqa: E402

EXPECT_SINKS = 4384   # every JPEG FF must be in the deck AND toggle
SLEW_FRAC    = 0.10   # worst sink slew budget = 10% of the clock period
TIMEOUT_S    = 14400  # jpeg dense pitches (52x52 mesh) run much longer than ibex

def deck_period_ps(sp):
    """Clock period from the deck's PULSE source (last arg), in ps."""
    for l in open(sp, errors="ignore"):
        m = re.search(r"PULSE\([^)]*?([0-9.]+)n\s*\)", l)
        if m:
            return float(m.group(1)) * 1000.0
    return 1000.0  # fallback: 1ns

# ---- knob space (jpeg core ~83um; window per old study 1.6-12, sparse edge ~14)
SPACE = {
    "PITCH":     [1.6, 2.4, 3.2, 4.0, 4.8, 6.0, 8.0, 10.0, 12.0, 14.0],
    "MBUF":      ["x1", "x2", "x3", "x4", "x6", "x8", "x10", "x12"],
    "FMAX":      [6, 8, 16, 24, 40, 80],
    "SBUF":      ["x2", "x3", "x4", "x6", "x8"],
    "CLKLAYERS": ["M2-M9", "M4-M8", "M5-M9"],
}

def tag_of(p):
    return (f"p{str(p['PITCH']).replace('.', 'p')}_{p['MBUF']}"
            f"_f{p['FMAX']}_{p['SBUF']}_{p['CLKLAYERS'].replace('-', '').lower()}_ck1")

def sink_counts(lis):
    tot, tog = 0, 0
    for l in open(lis, errors="ignore"):
        m = re.match(r"\s*t_sink_\d+=\s*([-0-9.eE+]+)([a-z]?)", l)
        if not m:
            continue
        tot += 1
        u = {"p": 1e-12, "n": 1e-9, "": 1}.get(m.group(2), 1)
        v = float(m.group(1)) * u
        if 0 < v < 1e-6:
            tog += 1
    return tot, tog

def evaluate(p, threads, hsp_mt):
    out = f"{DSEDIR}/{tag_of(p)}"
    os.makedirs(out, exist_ok=True)
    lis = f"{out}/jpeg_backside_full.lis"
    if not os.path.exists(lis):                      # cache: skip finished configs
        env = dict(os.environ,
                   DESIGN="jpeg", FEEDER="full", TAPROUTE="router",
                   PITCH=str(p["PITCH"]),
                   MBUF=f"gt2_6t_buf_{p['MBUF']}_w31_lvt",
                   CHECKER="1",
                   FMAX=str(p["FMAX"]),
                   SBUF=f"gt2_6t_buf_{p['SBUF']}_w31_lvt",
                   CLKLAYERS=p["CLKLAYERS"], OUTDIR=out)
        log = f"{out}/openroad.log"
        with open(log, "w") as lf:
            r = subprocess.run([ORD, "-threads", str(threads), "-exit", FLOW],
                               stdout=lf, stderr=subprocess.STDOUT, env=env,
                               cwd=JPEG, timeout=TIMEOUT_S)
        ltxt = open(log, errors="ignore").read()
        if (r.returncode != 0 or "DRT-0206" in ltxt or "GRT-0116" in ltxt
                or "checkConnectivity break" in ltxt or "[ERROR" in ltxt):
            raise optuna.TrialPruned("openroad/route failure")
        # license-retry: shared HSpice licenses run out when several studies
        # overlap; a license-failed .lis is a tiny stub with no measures.
        for _att in range(6):
            subprocess.run(f"hspice jpeg_backside_full.sp -mt {hsp_mt} -o jpeg_backside_full"
                           " > hspice.log 2>&1", shell=True, cwd=out,
                           timeout=TIMEOUT_S)
            if os.path.exists(lis):
                _t = open(lis, errors="ignore").read()
                if "Licensed number of users" not in _t:
                    break
                os.remove(lis)
            time.sleep(180)
    if not os.path.exists(lis):
        raise optuna.TrialPruned("no .lis (hspice failed)")
    tot, tog = sink_counts(lis)
    if tot != EXPECT_SINKS:
        raise optuna.TrialPruned(f"{EXPECT_SINKS - tot} FFs missing from deck")
    if tog != tot:
        raise optuna.TrialPruned(f"{tot - tog} sinks not toggling")
    m = metrics(out)
    if not m or not m["n"]:
        raise optuna.TrialPruned("metrics parse failed")
    budget = SLEW_FRAC * deck_period_ps(f"{out}/jpeg_backside_full.sp")
    if m.get("slew_max", 0) > budget:
        raise optuna.TrialPruned(
            f"slew_max {m['slew_max']:.1f}ps > {budget:.0f}ps budget")
    return m["skew"], m["pw"], m

def objective(trial, threads, hsp_mt):
    p = {k: trial.suggest_categorical(k, v) for k, v in SPACE.items()}
    skew, pw, m = evaluate(p, threads, hsp_mt)
    trial.set_user_attr("insertion_ps", round(m["ins"], 1))
    trial.set_user_attr("slew_max_ps", round(m.get("slew_max", 0), 1))
    trial.set_user_attr("outdir", tag_of(p))
    return skew, pw

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trials", type=int, default=40)
    ap.add_argument("--jobs", type=int, default=1, help="concurrent flow runs")
    ap.add_argument("--sampler", choices=["tpe", "nsga2"], default="tpe")
    ap.add_argument("--or_threads", type=int, default=0,
                    help="threads per OpenROAD run (default: 128//jobs)")
    ap.add_argument("--hspice_mt", type=int, default=0,
                    help="-mt per HSpice run (default: 48//jobs)")
    ap.add_argument("--report", action="store_true", help="print Pareto set only")
    a = ap.parse_args()
    os.makedirs(DSEDIR, exist_ok=True)
    sampler = (optuna.samplers.TPESampler(multivariate=True, seed=7)
               if a.sampler == "tpe" else optuna.samplers.NSGAIISampler(seed=7))
    study = optuna.create_study(study_name="jpeg_checker_mesh_dse", storage=DB,
                                directions=["minimize", "minimize"],
                                sampler=sampler, load_if_exists=True)
    if a.report:
        print(f"{'skew(ps)':>9} {'pw(mW)':>7}  params")
        rows = []
        for t in sorted(study.best_trials, key=lambda t: t.values[0]):
            print(f"{t.values[0]:>9.1f} {t.values[1]:>7.2f}  {t.params}")
            rows.append((t.values[0], t.values[1], t.params))
        with open(f"{DSEDIR}/pareto.csv", "w") as f:
            f.write("skew_ps,power_mW,params\n")
            for s, p, prm in rows:
                f.write(f'{s},{p},"{prm}"\n')
        print(f"({len(rows)} Pareto points; {len(study.trials)} total trials)"
              f" -> {DSEDIR}/pareto.csv")
        return
    threads = a.or_threads if a.or_threads else max(16, 128 // a.jobs)
    hsp_mt = a.hspice_mt if a.hspice_mt else max(8, 48 // a.jobs)
    print(f"per-trial resources: openroad -threads {threads}, hspice -mt {hsp_mt}, {a.jobs} parallel")
    study.optimize(lambda t: objective(t, threads, hsp_mt),
                   n_trials=a.trials, n_jobs=a.jobs,
                   catch=(subprocess.TimeoutExpired,))
    print("done; run with --report to see the Pareto set")

if __name__ == "__main__":
    main()
