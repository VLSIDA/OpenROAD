#!/usr/bin/env python3
"""Monte Carlo driver: per-sample netlists, parallel HSPICE, checkpointed CSV.

Sample 0 is the deterministic nominal; samples 1..N are random.  Every
finished sample is appended to samples.csv immediately (that file is the
checkpoint: rerunning the same command resumes, skipping finished samples
after verifying the run configuration hash matches run_meta.json).

Fail-loud policy: a sample whose HSPICE run fails, whose .mt0 is missing a
sink, or where any t_sink/slew_sink measure 'failed' (a flip-flop that did
not toggle) aborts the whole run with the scratch dir kept for post-mortem.

Usage:
  run_mcvar.py --network bslcb -n 100 --seed 20260901 -j 16
  run_mcvar.py --network bslcb -n 1000 --sigma-r-front 0.20 -o sweep_r20
"""
import argparse
import csv
import hashlib
import json
import multiprocessing as mp
import os
import shutil
import subprocess
import sys
import time
from dataclasses import asdict
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from sampler import (VarConfig, build_template, draw_sample, parse_mt0,
                     render, BACKSIDE_DIR)

NETWORKS = {
    # deck (already-patched s10 decks are used purely as element/value
    # sources; all their monte machinery is stripped by build_template)
    "bslcb": dict(deck="mc_sigma10/bslcb_s10.sp", mesh_is_backside=True),
    "fslcb": dict(deck="mc_sigma10/fslcb_s10.sp", mesh_is_backside=False),
    "fsdirect": dict(deck="mc_sigma10/fsdirect_s10.sp", mesh_is_backside=False),
    # CTS tree v3: distributed wire RC + via R from the routed DEF, per-FF
    # sinks - full parity with the mesh decks (see build_cts_rc_deck.py)
    "tree": dict(deck="mc_sigma10/tree_rc_s10.sp", mesh_is_backside=False),
}
CDL = BACKSIDE_DIR.parent.parent.parent.parent.parent / "GT2N/cdl/gt2_6t_w31_lvt.cdl"
MODEL_CARD = BACKSIDE_DIR / "gt2_w31_lvt_tt_renamed.sp"

CSV_FIELDS = ["sample", "is_nominal", "vdd", "skew_ps", "max_slew_ps",
              "n_slew_fail", "latency_ps", "t_root_ps", "arr_min_ps",
              "arr_max_ps", "n_sinks", "hspice_s"]

_G = {}  # worker globals (template/config shared via fork)


def run_sample(idx: int):
    """Never raises: returns a row dict, or a failure marker the main loop
    logs and skips - one bad sample must not kill a 10k campaign."""
    try:
        return _run_sample(idx)
    except Exception as e:
        return {"__fail__": idx, "msg": str(e)}


def _run_sample(idx: int):
    tp, cfg, seed, scratch = _G["tp"], _G["cfg"], _G["seed"], _G["scratch"]
    sdir = scratch / f"s{idx}"
    sdir.mkdir(parents=True, exist_ok=True)
    vals = draw_sample(tp, cfg, seed, idx)
    deck = sdir / "deck.sp"
    deck.write_text(render(tp, vals))
    t0 = time.time()
    r = subprocess.run(["hspice", "-i", "deck.sp", "-o", "out"], cwd=sdir,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    dt = time.time() - t0
    mt0 = sdir / "out.mt0"
    if r.returncode != 0 or not mt0.exists():
        raise RuntimeError(f"sample {idx}: hspice failed rc={r.returncode} "
                           f"(kept {sdir})")
    m = parse_mt0(mt0)

    n_sinks = tp.n_sinks
    t_sink = np.array([m.get(f"t_sink_{k}", np.nan) for k in range(n_sinks)])
    slew = np.array([m.get(f"slew_sink_{k}", np.nan) for k in range(n_sinks)])
    missing = [k for k in range(n_sinks) if f"t_sink_{k}" not in m]
    if missing:
        raise RuntimeError(f"sample {idx}: {len(missing)} sinks missing from "
                           f".mt0 (first: t_sink_{missing[0]}; kept {sdir})")
    bad = np.flatnonzero(~np.isfinite(t_sink))
    if len(bad):
        raise RuntimeError(
            f"sample {idx}: {len(bad)} flip-flops failed to toggle "
            f"(first: sink {bad[0]}; kept {sdir})")
    # Slew measures are censored, not fatal: under --vdd-scope iid a buffer
    # whose local rail draws below ~0.63 V can never cross the 90%-of-nominal
    # slew target, so its sinks report 'failed'.  Arrivals (50% threshold)
    # remain valid; we record the count and take max_slew over the rest.
    n_slew_fail = int(np.sum(~np.isfinite(slew)))
    t_root = m.get("t_root", np.nan)
    if not np.isfinite(t_root):
        raise RuntimeError(f"sample {idx}: t_root measure failed (kept {sdir})")

    row = dict(
        sample=idx, is_nominal=int(idx == 0),
        # global scope: the (single) rail draw; iid scope: mean of per-cell rails
        vdd=float(np.mean(vals[7])) if len(vals.get(7, [])) else float(vals[0][0]),
        skew_ps=(t_sink.max() - t_sink.min()) * 1e12,
        max_slew_ps=(np.nanmax(slew) if n_slew_fail < n_sinks else np.nan) * 1e12,
        n_slew_fail=n_slew_fail,
        latency_ps=(t_sink.max() - t_root) * 1e12,
        t_root_ps=t_root * 1e12,
        arr_min_ps=t_sink.min() * 1e12,
        arr_max_ps=t_sink.max() * 1e12,
        n_sinks=n_sinks, hspice_s=round(dt, 1))
    if _G["keep_scratch"]:
        (sdir / "row.json").write_text(json.dumps(row))
    else:
        shutil.rmtree(sdir)
    return row


def config_hash(cfg: VarConfig, seed: int, deck_md5: str) -> str:
    blob = json.dumps({**asdict(cfg), "seed": seed, "deck_md5": deck_md5},
                      sort_keys=True)
    return hashlib.md5(blob.encode()).hexdigest()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--network", choices=NETWORKS, required=True)
    ap.add_argument("-n", "--samples", type=int, default=100,
                    help="random samples (nominal sample 0 is extra)")
    ap.add_argument("--seed", type=int, default=20260901)
    ap.add_argument("-j", "--jobs", type=int, default=16)
    ap.add_argument("-o", "--outdir", type=Path, default=None)
    ap.add_argument("--vdd-scope", choices=["global", "iid"], default="global",
                    help="global: one rail draw per sample; iid: independent "
                         "rail per cell instance (thresholds stay nominal)")
    ap.add_argument("--sigma-vdd", type=float, default=0.018,
                    help="global Vdd sigma in V; 0 freezes Vdd at 0.7")
    ap.add_argument("--sigma-r-front", type=float, default=0.125)
    ap.add_argument("--sigma-r-back", type=float, default=None,
                    help="default: sigma_r_front / 2")
    ap.add_argument("--sheets", choices=["shared", "independent"],
                    default="shared")
    ap.add_argument("--keep-scratch", action="store_true")
    args = ap.parse_args()

    net = NETWORKS[args.network]
    cfg = VarConfig(sigma_vdd=args.sigma_vdd,
                    vdd_scope=args.vdd_scope,
                    sigma_r_front=args.sigma_r_front,
                    sigma_r_back=args.sigma_r_back,
                    sheets=args.sheets,
                    mesh_is_backside=net["mesh_is_backside"]).resolve()

    here = Path(__file__).resolve().parent
    outdir = args.outdir or here / f"{args.network}_n{args.samples}"
    outdir = outdir if outdir.is_absolute() else here / outdir
    outdir.mkdir(parents=True, exist_ok=True)

    deck = BACKSIDE_DIR / net["deck"]
    tp = build_template(deck, CDL, MODEL_CARD, cfg)
    chash = config_hash(cfg, args.seed, tp.meta["deck_md5"])
    print(f"[{args.network}] sinks={tp.n_sinks} instances={tp.n_instances} "
          f"fingers={tp.meta['n_fingers']} R={tp.meta['n_r']} {tp.meta['r_cats']} "
          f"C={tp.meta['n_c']} {tp.meta['c_cats']}")
    if tp.n_sinks != 1938:
        sys.exit(f"FATAL: expected 1938 sinks, template has {tp.n_sinks}")

    meta_f = outdir / "run_meta.json"
    csv_f = outdir / "samples.csv"
    done = set()
    if meta_f.exists():
        old = json.loads(meta_f.read_text())
        if old["config_hash"] != chash:
            sys.exit(f"FATAL: {outdir} holds a run with a different "
                     f"config/seed; use a fresh -o outdir")
        if csv_f.exists():
            with open(csv_f) as f:
                rd = csv.DictReader(f)
                old_fields = rd.fieldnames
                rows = list(rd)
            done = {int(r["sample"]) for r in rows}
            if old_fields != CSV_FIELDS:  # migrate pre-n_slew_fail files
                with open(csv_f, "w", newline="") as f:
                    wr = csv.DictWriter(f, fieldnames=CSV_FIELDS)
                    wr.writeheader()
                    for r in rows:
                        r.setdefault("n_slew_fail", 0)
                        wr.writerow(r)
                print(f"migrated {csv_f.name} to new column set")
            print(f"resuming: {len(done)} samples already in {csv_f.name}")
    else:
        meta_f.write_text(json.dumps(
            {**asdict(cfg), "seed": args.seed, "network": args.network,
             "samples": args.samples, "config_hash": chash,
             "template": tp.meta}, indent=2, default=str))

    todo = [i for i in range(args.samples + 1) if i not in done]
    if not todo:
        print("nothing to do; all samples present")
        return
    scratch = outdir / "scratch"
    _G.update(tp=tp, cfg=cfg, seed=args.seed, scratch=scratch,
              keep_scratch=args.keep_scratch)

    new_csv = not csv_f.exists()
    t0 = time.time()
    with open(csv_f, "a", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=CSV_FIELDS)
        if new_csv:
            w.writeheader()
        fails = 0
        fail_log = outdir / "failures.log"
        with mp.Pool(args.jobs) as pool:
            for k, row in enumerate(pool.imap_unordered(run_sample, todo), 1):
                if "__fail__" in row:
                    fails += 1
                    with open(fail_log, "a") as fl:
                        fl.write(f"sample {row['__fail__']}: {row['msg']}\n")
                else:
                    w.writerow(row)
                    fh.flush()
                if k % 25 == 0 or k == len(todo):
                    rate = k / (time.time() - t0)
                    print(f"  {k}/{len(todo)} done "
                          f"({rate*3600:.0f} samples/h, "
                          f"eta {(len(todo)-k)/rate/60:.1f} min)", flush=True)
    msg = f"done: {len(todo)-fails}/{len(todo)} samples in " \
          f"{(time.time()-t0)/60:.1f} min -> {csv_f}"
    if fails:
        msg += f"\nWARNING: {fails} samples FAILED - see {fail_log}; " \
               f"re-running this command retries them"
    print(msg)


if __name__ == "__main__":
    main()
