#!/usr/bin/env python3
"""Patch a ClockMesh-generated SPICE deck for Monte Carlo (UNIFORM sigma=5%, 15% at 3-sigma).

Creates MC copies of the model card, the CDL, and the deck; originals are
never modified.  HSPICE only accepts distribution functions as the entire
RHS of a .param, so every random draw is a dedicated *_var param that other
expressions reference.

Variation set (all 5% at 3-sigma):
  Vth      per transistor   dvtn_mc/dvtp_mc referenced by delvtrand,
                            sampled per device via .option modmonte=1
                            (NMOS 8.1mV, PMOS 9.6mV = 5% of extracted Vt)
  W, L     per buffer       subckt-local wfac/lfac params in the CDL
  VDD      per iteration    vddval (gated by mc_pr_switch)
  wire R/C per iteration    rfac/cfac (gated by mc_pr_switch)
  TSV R    per TSV          one unique gauss param per Rtsv element
  FF pin C per sink         one unique gauss param per Csink element
                            (gate cap is a device quantity -> local)

Usage: make_mc.py <deck.sp> [-n ITER] [--seed S] [-o OUTDIR]
"""
import argparse
import re
import sys
from pathlib import Path

SIG_VT_N = "24.3m"  # 3-sigma = 15% (sigma=5%) of NMOS Vt 162 mV
SIG_VT_P = "28.8m"  # 3-sigma = 15% (sigma=5%) of PMOS Vt 192 mV
REL5 = "0.15"       # sigma=5% relative (15% at 3-sigma)
VDD_NOM = "0.7"
SIG_VDD = "105m"    # sigma=35mV (5% of 0.7 V; 15% at 3-sigma)


def patch_model_card(src: Path, dst: Path) -> None:
    out, polarity, injected = [], None, False
    for line in src.read_text().splitlines():
        m = re.match(r"\s*\.MODEL\s+\S+\s+(NMOS|PMOS)", line, re.IGNORECASE)
        if m:
            if not injected:
                out += [f".param dvtn_mc=agauss(0,{SIG_VT_N},3)",
                        f".param dvtp_mc=agauss(0,{SIG_VT_P},3)"]
                injected = True
            polarity = m.group(1).upper()
        dm = re.match(r"(\s*\+\s*delvtrand\s*=\s*)([-0-9.eE]+)\s*$", line)
        if dm:
            var = "dvtn_mc" if polarity == "NMOS" else "dvtp_mc"
            line = f"{dm.group(1)}'{dm.group(2)} + mc_mm_switch*{var}'"
        out.append(line)
    dst.write_text("\n".join(out) + "\n")


def patch_cdl(src: Path, dst: Path) -> None:
    out = []
    for line in src.read_text().splitlines():
        out.append(re.sub(r"\b([WL])=([0-9.]+u)\b",
                          lambda m: f"{m.group(1)}='{m.group(2)}*{m.group(1).lower()}fac'",
                          line))
        if re.match(r"\s*\.subckt\s", line, re.IGNORECASE):
            out.append(f".param wfac=gauss(1,{REL5},3) lfac=gauss(1,{REL5},3)")
    dst.write_text("\n".join(out) + "\n")


def patch_deck(src: Path, dst: Path, mc_card: Path, mc_cdl: Path,
               iters: int, seed: int) -> None:
    txt = src.read_text()
    for pat in (r"\.param mc_mm_switch=0", r"\.param mc_pr_switch=0",
                r"^Vvdd VDD 0 0\.7\s*$", r"\.tran\s"):
        if not re.search(pat, txt, re.MULTILINE):
            sys.exit(f"deck is missing expected pattern: {pat}")

    lines_out, local_params, header_at = [], [], None
    ntsv = nffc = 0
    for line in txt.splitlines():
        s = line.strip()
        if s == ".param mc_mm_switch=0":
            line = ".param mc_mm_switch=1"
        elif s == ".param mc_pr_switch=0":
            lines_out.append(".param mc_pr_switch=1")
            lines_out += [
                f".param vdd_var=agauss(0,{SIG_VDD},3)",
                f".param vddval='{VDD_NOM} + mc_pr_switch*vdd_var'",
                f".param r_var=agauss(0,{REL5},3)",
                f".param rfac='1 + mc_pr_switch*r_var'",
                f".param c_var=agauss(0,{REL5},3)",
                f".param cfac='1 + mc_pr_switch*c_var'",
                f".option modmonte=1 randgen=moa seed={seed}",
            ]
            header_at = len(lines_out)  # per-element params get inserted here
            continue
        elif s.startswith(".include"):
            if "renamed.sp" in s or "_tt.sp" in s:
                line = f".include {mc_card}"
            elif ".cdl" in s:
                line = f".include {mc_cdl}"
        elif s.startswith("Vvdd "):
            line = "Vvdd VDD 0 vddval"
        elif s.startswith("Vclk"):
            line = line.replace("PULSE(0 0.7 ", "PULSE(0 'vddval' ")
        elif s.startswith(".measure"):
            line = (line.replace("=0.35 ", "='vddval*0.5' ")
                        .replace("=0.07 ", "='vddval*0.1' ")
                        .replace("=0.63 ", "='vddval*0.9' "))
        elif s.startswith(".tran"):
            line = line.rstrip() + f" sweep monte={iters}"
        elif re.match(r"[RC]\S*\s", line):
            tok = line.split()
            if len(tok) >= 4:
                try:
                    float(tok[3])
                except ValueError:
                    pass
                else:
                    name = tok[0]
                    if name.startswith("Rtsv"):
                        var = f"ftsv{ntsv}"
                        ntsv += 1
                        local_params.append(f".param {var}=gauss(1,{REL5},3)")
                        tok[3] = f"'{tok[3]}*{var}'"
                    elif re.match(r"Csink\d+$", name):
                        var = f"fffc{nffc}"
                        nffc += 1
                        local_params.append(f".param {var}=gauss(1,{REL5},3)")
                        tok[3] = f"'{tok[3]}*{var}'"
                    elif name.startswith("R"):
                        tok[3] = f"'{tok[3]}*rfac'"
                    else:
                        tok[3] = f"'{tok[3]}*cfac'"
                    line = " ".join(tok)
        lines_out.append(line)

    if header_at is not None:
        lines_out[header_at:header_at] = local_params
    dst.write_text("\n".join(lines_out) + "\n")
    print(f"patched: {ntsv} TSV resistors, {nffc} FF pin caps (per-element)")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("deck", type=Path)
    ap.add_argument("-n", "--iters", type=int, default=1000)
    ap.add_argument("--seed", type=int, default=1234)
    ap.add_argument("-o", "--outdir", type=Path,
                    default=Path(__file__).resolve().parent)
    args = ap.parse_args()

    deck = args.deck.resolve()
    txt = deck.read_text()
    card = Path(re.search(r"\.include\s+(\S+(?:renamed|_tt)\.sp)", txt).group(1))
    cdl = Path(re.search(r"\.include\s+(\S+\.cdl)", txt).group(1))

    outdir = args.outdir.resolve()
    outdir.mkdir(parents=True, exist_ok=True)
    mc_card = outdir / card.name.replace(".sp", "_mc.sp")
    mc_cdl = outdir / cdl.name.replace(".cdl", "_mc.cdl")
    mc_deck = outdir / deck.name.replace(".sp", "_mc.sp")

    patch_model_card(card, mc_card)
    patch_cdl(cdl, mc_cdl)
    patch_deck(deck, mc_deck, mc_card, mc_cdl, args.iters, args.seed)
    print(f"model card: {mc_card}\ncdl:        {mc_cdl}\ndeck:       {mc_deck}")
    print(f"run:        python3 run_mc.py {mc_deck.name} -n {args.iters} -j 8")


if __name__ == "__main__":
    main()
