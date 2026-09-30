#!/usr/bin/env python3
# Multi-objective Bayesian DSE for the JPEG FRONTSIDE clock mesh.
# Smaller knob space than backside (no sink tier -- FFs connect directly):
#   knobs      : PITCH, MBUF (mesh driver), MESHLAYERS (M5M6|M7M8),
#                CLKLAYERS (clock routing window)
#   objectives : minimize (worst-case skew [ps], clock power [mW])  -- HSpice
#   infeasible : route failure / FFs missing / not toggling / slew > 10% period
#
# Seeds: previous FS runs (same flow basis: direct-to-mesh, full feeder).
#   /home/wali2/backside/dse_venv/bin/python ibex_fs_bo_dse.py --seed --trials 300 --jobs 3
import argparse, os, re, subprocess, sys, glob, time

import optuna
from optuna.distributions import CategoricalDistribution
from optuna.trial import TrialState, create_trial

BASE   = "/home/wali2/backside/OpenROAD/src/cms/test/backside_result"
JPEG   = f"{BASE}/Jpeg"
FLOW   = f"{BASE}/multi_frontside_baltree.tcl"
ORD    = "/home/wali2/backside/OpenROAD/build/bin/openroad"
DSEDIR = f"{JPEG}/dse_fs"
DB     = f"sqlite:///{DSEDIR}/jpeg_fs_dse.db"
sys.path.insert(0, f"{BASE}/Jpeg")
from lis_metrics import metrics  # noqa: E402

EXPECT_SINKS = 4384
SLEW_FRAC    = 0.10
TIMEOUT_S    = 14400  # jpeg dense FS meshes run long

def deck_period_ps(sp):
    for l in open(sp, errors="ignore"):
        m = re.search(r"PULSE\([^)]*?([0-9.]+)n\s*\)", l)
        if m:
            return float(m.group(1)) * 1000.0
    return 1000.0

SPACE = {
    "PITCH":      [1.6, 2.4, 3.2, 4.0, 4.8, 6.0, 8.0, 10.0, 12.0, 14.0],
    "MBUF":       ["x1", "x2", "x3", "x4", "x6", "x8", "x10", "x12"],
    "MESHLAYERS": ["M5M6", "M7M8"],
    "CLKLAYERS":  ["M2-M9", "M4-M8", "M5-M9"],
}
DISTS = {k: CategoricalDistribution(v) for k, v in SPACE.items()}

def tag_of(p):
    return (f"p{str(p['PITCH']).replace('.', 'p')}_{p['MBUF']}"
            f"_{p['MESHLAYERS'].lower()}_{p['CLKLAYERS'].replace('-', '').lower()}")

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
    lis = f"{out}/jpeg_frontside.lis"
    if not os.path.exists(lis):
        env = dict(os.environ,
                   DESIGN="jpeg", FEEDER="full",
                   PITCH=str(p["PITCH"]),
                   MBUF=f"gt2_6t_buf_{p['MBUF']}_w31_lvt",
                   MESHLAYERS=p["MESHLAYERS"],
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
            subprocess.run(f"hspice jpeg_frontside.sp -mt {hsp_mt} -o jpeg_frontside"
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
    budget = SLEW_FRAC * deck_period_ps(f"{out}/jpeg_frontside.sp")
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

# ---- seeds: previous FS runs (same basis; all M5M6 mesh, clock M2-M9)
def seed_dirs():
    out = []
    for p, t in [(1.6, "1p6"), (2.4, "2p4"), (8.0, "8"), (12.0, "12"), (14.0, "14")]:
        out.append((dict(PITCH=p, MBUF="x4", MESHLAYERS="M5M6",
                         CLKLAYERS="M2-M9"), f"{JPEG}/density/fs_pitch{t}"))
    out.append((dict(PITCH=3.2, MBUF="x4", MESHLAYERS="M5M6",
                     CLKLAYERS="M2-M9"), f"{JPEG}/baltree/fs_full"))
    return out

def seed_study(study):
    done = {tuple(sorted(t.params.items())) for t in study.trials}
    n = 0
    for params, d in seed_dirs():
        if tuple(sorted(params.items())) in done:
            continue
        lis = glob.glob(d + "/*.lis")
        if not lis:
            continue
        tot, tog = sink_counts(lis[0])
        if tot != EXPECT_SINKS or tog != tot:
            continue
        m = metrics(d)
        if not m or m.get("slew_max", 0) > 100.0:
            continue
        study.add_trial(create_trial(
            params=params, distributions=DISTS,
            values=[m["skew"], m["pw"]], state=TrialState.COMPLETE))
        n += 1
    print(f"seeded {n} completed FS runs into the study")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trials", type=int, default=40)
    ap.add_argument("--jobs", type=int, default=1)
    ap.add_argument("--sampler", choices=["tpe", "nsga2"], default="tpe")
    ap.add_argument("--or_threads", type=int, default=0)
    ap.add_argument("--hspice_mt", type=int, default=0)
    ap.add_argument("--seed", action="store_true")
    ap.add_argument("--report", action="store_true")
    a = ap.parse_args()
    os.makedirs(DSEDIR, exist_ok=True)
    sampler = (optuna.samplers.TPESampler(multivariate=True, seed=7)
               if a.sampler == "tpe" else optuna.samplers.NSGAIISampler(seed=7))
    study = optuna.create_study(study_name="jpeg_fs_mesh_dse", storage=DB,
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
    threads = a.or_threads if a.or_threads else max(16, 128 // a.jobs)
    hsp_mt = a.hspice_mt if a.hspice_mt else max(8, 48 // a.jobs)
    print(f"per-trial resources: openroad -threads {threads},"
          f" hspice -mt {hsp_mt}, {a.jobs} parallel")
    study.optimize(lambda t: objective(t, threads, hsp_mt),
                   n_trials=a.trials, n_jobs=a.jobs,
                   catch=(subprocess.TimeoutExpired,))
    print("done; run with --report to see the Pareto set")

if __name__ == "__main__":
    main()
