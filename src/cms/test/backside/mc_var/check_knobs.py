#!/usr/bin/env python3
"""Regression gate for the per-transistor variation knobs used by sampler.py.

Validated findings on hspice 2017.03 with the GT2N BSIM-CMG card (run this
again before any big campaign; it must print OK):

  - instance delvto is silently DEAD; instance hfin warns and is ignored
  - instance delvtrand is live, with OVERRIDE semantics (replaces the
    model value) -> sampler writes delvtrand = model nominal + draw
  - instance L and tfin are live
  - fractional M is live -> W_NS emulation M = 1 + 2*dW/(2*W+T)
  - expanding M=k into k parallel M=1 fingers is electrically neutral

Builds one deck with variant buf_x4 copies, checks each knob moves the
delay and expansion does not.
"""
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from sampler import parse_mt0, parse_model_delvtrand, BACKSIDE_DIR

MODEL = BACKSIDE_DIR / "gt2_w31_lvt_tt_renamed.sp"
OUT = Path(__file__).resolve().parent / "knobcheck"

FINGERS = [("MM2", "Y", "net13", "vdd", "pmos_lvt", 4),
           ("MM0", "net13", "A", "vdd", "pmos_lvt", 1),
           ("MM3", "Y", "net13", "vss", "nmos_lvt", 4),
           ("MM1", "net13", "A", "vss", "nmos_lvt", 1)]


def buf(name, dvt, l="1.4e-08", tfin="5e-09", m="1", stock_m=False):
    txt = [f".subckt buf_{name} A Y vdd vss"]
    for dn, d, g, s, mod, mm in FINGERS:
        line = (f"{d} {g} {s} {mod} W=0.031u L={l} tfin={tfin} "
                f"delvtrand={dvt[mod]}")
        if stock_m:
            txt.append(f"{dn} {line} M={mm}")
        else:
            txt += [f"{dn}_f{f} {line} M={m}" for f in range(mm)]
    txt.append(f".ends buf_{name}")
    return txt


def main():
    OUT.mkdir(exist_ok=True)
    nom = parse_model_delvtrand(MODEL)
    dvtp = {**nom, "pmos_lvt": nom["pmos_lvt"] + 0.060}   # +3sig pMOS Vth
    dvtn = {**nom, "nmos_lvt": nom["nmos_lvt"] + 0.045}   # +3sig nMOS Vth
    variants = {
        "nom":    buf("nom", nom),
        "expand": buf("expand", nom, stock_m=True),
        "dvtp":   buf("dvtp", dvtp),
        "dvtn":   buf("dvtn", dvtn),
        "lg":     buf("lg", nom, l="1.4501e-08"),          # +3sig L_g
        "tfin":   buf("tfin", nom, tfin="4.601e-09"),      # -3sig T_NS
        "wns":    buf("wns", nom, m="0.985075"),           # -3sig W_NS via M
    }
    deck = ["* knob regression", ".option ingold=2", ".option autostop",
            f".include {MODEL}", "Vvdd VDD 0 0.7",
            "Vin in 0 PULSE(0 0.7 0.1n 0.01n 0.01n 0.5n 1n)"]
    for name, sub in variants.items():
        deck += sub
        deck += [f"X{name} in y_{name} VDD 0 buf_{name}",
                 f"C{name} y_{name} 0 2f",
                 f".measure tran d_{name} TRIG v(in) VAL=0.35 RISE=1 "
                 f"TARG v(y_{name}) VAL=0.35 RISE=1"]
    deck += [".tran 0.001n 1n", ".end"]
    (OUT / "knob.sp").write_text("\n".join(deck) + "\n")

    r = subprocess.run(["hspice", "-i", "knob.sp", "-o", "knob"], cwd=OUT,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    if r.returncode != 0 or not (OUT / "knob.mt0").exists():
        sys.exit(f"hspice failed rc={r.returncode} (see {OUT}/knob.lis)")
    m = parse_mt0(OUT / "knob.mt0")
    d0 = m["d_nom"] * 1e12
    print(f"nominal buf_x4 delay = {d0:.4f} ps")
    for k in variants:
        if k != "nom":
            print(f"  {k:7s} delta = {(m['d_'+k]-m['d_nom'])*1e15:8.1f} fs")
    fails = []
    if abs(m["d_expand"] - m["d_nom"]) * 1e12 > 0.1:
        fails.append("finger expansion NOT neutral")
    for k in ("dvtp", "dvtn", "lg", "tfin", "wns"):
        if abs(m[f"d_{k}"] - m["d_nom"]) * 1e15 < 5:
            fails.append(f"knob '{k}' dead")
    if fails:
        sys.exit("FAIL:\n  " + "\n  ".join(fails))
    print("all knobs live; finger expansion neutral. OK")


if __name__ == "__main__":
    main()
