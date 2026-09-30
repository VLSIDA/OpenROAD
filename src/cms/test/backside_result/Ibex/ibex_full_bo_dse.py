#!/usr/bin/env python3
# Multi-objective Bayesian DSE for the Ibex backside clock mesh
# FULL-TREE basis: objectives measured on ${design}_backside_full.sp (root pulse,
# feeder tree simulated; power includes the feeder; analytic BM caps).
#
#   objectives : minimize (worst-case skew [ps], clock power [mW])  -- from HSpice
#   knobs      : PITCH, MBUF (mesh driver), FMAX (LCB fanout), SBUF (LCB size),
#                CLKLAYERS (clock routing window)
#   basis      : TAPROUTE=router (sink taps routed by GRT/DRT), full LVT feeder
#   infeasible : route failure / FFs dropped from deck / sinks not toggling /
#                worst sink slew over budget  -> optuna.TrialPruned
#
# Run with the dse venv python:
#   /home/wali2/backside/dse_venv/bin/python ibex_bo_dse.py --seed --trials 60 --jobs 2
#
# Restartable: state lives in dse/ibex_dse.db (sqlite). Re-running resumes.
# Pareto set:  --report prints study.best_trials and writes dse/pareto.csv.
import argparse, os, re, subprocess, sys, glob, time

import optuna
from optuna.distributions import CategoricalDistribution
from optuna.trial import TrialState, create_trial

BASE   = "/home/wali2/backside/OpenROAD/src/cms/test/backside_result"
IBEX   = f"{BASE}/Ibex"
FLOW   = f"{BASE}/multi_backside_baltree.tcl"
ORD    = "/home/wali2/backside/OpenROAD/build/bin/openroad"
DSEDIR = f"{IBEX}/dse_full"
DB     = f"sqlite:///{DSEDIR}/ibex_full_dse.db"
sys.path.insert(0, f"{BASE}/Jpeg")
from lis_metrics import metrics  # noqa: E402

EXPECT_SINKS = 1938   # every Ibex FF must be in the deck AND toggle
SLEW_MAX_PS  = 50.0   # worst sink slew budget (10-90%); prune above this

# ---- knob space (P16 excluded: known flow bug drops 8 FFs -> always infeasible)
SPACE = {
    "PITCH":     [1.6, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0, 12.0, 14.0],
    "MBUF":      ["x1", "x2", "x3", "x4", "x6", "x8", "x10", "x12"],
    "FMAX":      [6, 8, 16, 24, 40, 80],
    "SBUF":      ["x2", "x3", "x4", "x6", "x8"],
    "CLKLAYERS": ["M2-M9", "M4-M8", "M5-M9"],
}
DISTS = {k: CategoricalDistribution(v) for k, v in SPACE.items()}

def tag_of(p):
    return (f"p{str(p['PITCH']).replace('.', 'p')}_{p['MBUF']}"
            f"_f{p['FMAX']}_{p['SBUF']}_{p['CLKLAYERS'].replace('-', '').lower()}")

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
    """Run flow + HSpice for knob dict p; return (skew_ps, power_mW, extras)."""
    out = f"{DSEDIR}/{tag_of(p)}"
    os.makedirs(out, exist_ok=True)
    lis = f"{out}/ibex_backside_full.lis"
    if not os.path.exists(lis):                      # cache: skip finished configs
        env = dict(os.environ,
                   DESIGN="ibex", FEEDER="full", TAPROUTE="router",
                   PITCH=str(p["PITCH"]),
                   MBUF=f"gt2_6t_buf_{p['MBUF']}_w31_lvt",
                   FMAX=str(p["FMAX"]),
                   SBUF=f"gt2_6t_buf_{p['SBUF']}_w31_lvt",
                   CLKLAYERS=p["CLKLAYERS"], OUTDIR=out)
        log = f"{out}/openroad.log"
        with open(log, "w") as lf:
            r = subprocess.run([ORD, "-threads", str(threads), "-exit", FLOW],
                               stdout=lf, stderr=subprocess.STDOUT, env=env,
                               cwd=IBEX, timeout=7200)
        ltxt = open(log, errors="ignore").read()
        if (r.returncode != 0 or "DRT-0206" in ltxt or "GRT-0116" in ltxt
                or "checkConnectivity break" in ltxt or "[ERROR" in ltxt):
            raise optuna.TrialPruned("openroad/route failure")
        # license-retry: shared HSpice licenses run out when several studies
        # overlap; a license-failed .lis is a tiny stub with no measures.
        for _att in range(6):
            subprocess.run(f"hspice ibex_backside_full.sp -mt {hsp_mt} -o ibex_backside_full"
                           " > hspice.log 2>&1", shell=True, cwd=out, timeout=7200)
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
    if m.get("slew_max", 0) > SLEW_MAX_PS:
        raise optuna.TrialPruned(f"slew_max {m['slew_max']:.1f}ps > budget")
    return m["skew"], m["pw"], m

def objective(trial, threads, hsp_mt):
    p = {k: trial.suggest_categorical(k, v) for k, v in SPACE.items()}
    skew, pw, m = evaluate(p, threads, hsp_mt)
    trial.set_user_attr("insertion_ps", round(m["ins"], 1))
    trial.set_user_attr("slew_max_ps", round(m.get("slew_max", 0), 1))
    trial.set_user_attr("outdir", tag_of(p))
    return skew, pw

# ---- seed trials: completed runs on the SAME basis (TAPROUTE=router)
SEEDS = [
    (dict(PITCH=6.0, MBUF="x1", FMAX=8, SBUF="x4", CLKLAYERS="M4-M8"),
     f"{IBEX}/fulltree/bs"),
]

def seed_study(study):
    done = {tuple(sorted(t.params.items())) for t in study.trials}
    n = 0
    for params, d in SEEDS:
        if tuple(sorted(params.items())) in done:
            continue
        lis = glob.glob(d + "/*_full.lis")
        if not lis:
            continue
        tot, tog = sink_counts(lis[0])
        if tot != EXPECT_SINKS or tog != tot:
            continue
        m = metrics(d)
        study.add_trial(create_trial(
            params=params, distributions=DISTS,
            values=[m["skew"], m["pw"]], state=TrialState.COMPLETE))
        n += 1
    print(f"seeded {n} completed runs into the study")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trials", type=int, default=40)
    ap.add_argument("--jobs", type=int, default=1, help="concurrent flow runs")
    ap.add_argument("--sampler", choices=["tpe", "nsga2"], default="tpe")
    ap.add_argument("--seed", action="store_true", help="inject completed runs")
    ap.add_argument("--report", action="store_true", help="print Pareto set only")
    a = ap.parse_args()
    os.makedirs(DSEDIR, exist_ok=True)
    sampler = (optuna.samplers.TPESampler(multivariate=True, seed=7)
               if a.sampler == "tpe" else optuna.samplers.NSGAIISampler(seed=7))
    study = optuna.create_study(study_name="ibex_full_mesh_dse", storage=DB,
                                directions=["minimize", "minimize"],
                                sampler=sampler, load_if_exists=True)
    if a.seed:
        seed_study(study)
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
    threads = max(16, 128 // a.jobs)
    hsp_mt = max(8, 48 // a.jobs)
    study.optimize(lambda t: objective(t, threads, hsp_mt),
                   n_trials=a.trials, n_jobs=a.jobs,
                   catch=(subprocess.TimeoutExpired,))
    print("done; run with --report to see the Pareto set")

if __name__ == "__main__":
    main()
