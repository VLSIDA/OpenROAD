#!/usr/bin/env python3
"""Per-sample SPICE netlist generation for GT2N clock-network Monte Carlo.

Python owns the RNG (numpy, seeded, recorded); HSPICE runs one plain
transient per sample.  This gives true per-instance IID draws:

  per transistor finger : dVth (delvto), L_g (L), W_NS (hfin), T_NS (tfin)
  per wire segment      : R multiplier, C multiplier
  per sample (global)   : Vdd

The template is built once from a ClockMesh MC deck (mc_sigma10/*_s10.sp
style): the old HSPICE-monte machinery (mc_*_switch, rfac/cfac, ftsv/fffc,
agauss params, modmonte, sweep monte=) is stripped, every R/C element value
becomes a slot, and every buffer subckt is uniquified per X instance with
its M= fingers expanded to individual devices so each physical transistor
gets its own draw.

Device knobs (validated empirically on hspice 2017.03 / this BSIM-CMG
card by check_knobs.py):
  dVth : instance delvtrand=<model nominal + draw>  (OVERRIDE semantics;
         instance delvto is silently dead on this card)
  L_g  : instance L=<draw>
  T_NS : instance tfin=<draw>
  W_NS : instance hfin is IGNORED by hspice, so W_NS is emulated with a
         fractional M multiplier, M = 1 + 2*dW/(2*W_nom + T_nom) - the
         model's own Weff=NFIN*(2*HFIN+TFIN) first-order sensitivity.
         Exact for drive and capacitance scaling, omits only second-order
         electrostatic W dependence.

Nanosheet correlation: the BSIM-CMG card lumps the stacked sheets into one
device (NFIN=3), so per-sheet independence cannot be expressed directly.
sheets="shared" (default) draws one W_NS/T_NS per transistor; "independent"
emulates fully uncorrelated sheets by scaling sigma_W and sigma_T by
1/sqrt(NFIN) (first-order equivalent for the device-level drive current).
L_g is a single gate over all sheets, so it is always one draw/transistor.

R-C correlation: draw_wire_multipliers() is the single seam where the R and
C vectors are drawn; a future rho_rc joins them there (accepted now, must
be 0.0 until segment pairing is implemented).
"""
import hashlib
import re
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

BACKSIDE_DIR = Path(__file__).resolve().parent.parent


@dataclass
class VarConfig:
    sigma_vth_p: float = 0.020      # V, IID per transistor
    sigma_vth_n: float = 0.015      # V, IID per transistor
    l_nom: float = 14e-9            # gate length
    sigma_l: float = 0.167e-9
    w_nom: float = 31e-9            # nanosheet width  -> BSIM-CMG hfin
    sigma_w: float = 0.167e-9
    t_nom: float = 5e-9             # nanosheet thickness -> BSIM-CMG tfin
    sigma_t: float = 0.133e-9
    vdd_nom: float = 0.7
    sigma_vdd: float = 0.018        # V; scope set by vdd_scope
    vdd_scope: str = "global"       # global: one draw/sample | iid: per cell
                                    # (iid: thresholds/pulse stay at vdd_nom)
    sigma_r_front: float = 0.125    # relative, IID per segment
    sigma_r_back: float = None      # default: sigma_r_front / 2
    sigma_r_tsv: float = None       # default: sigma_r_back
    sigma_c: float = 0.03           # relative, IID per segment (all C)
    sigma_c_sink: float = None      # FF pin caps; default: sigma_c
    rho_rc: float = 0.0             # R-C correlation, seam only for now
    sheets: str = "shared"          # shared | independent (see module doc)
    n_sheet: int = 3                # NFIN in the GT2N card (paper says 4)
    mesh_is_backside: bool = True   # False for the frontside-mesh decks

    def resolve(self):
        if self.sigma_r_back is None:
            self.sigma_r_back = self.sigma_r_front / 2.0
        if self.sigma_r_tsv is None:
            self.sigma_r_tsv = self.sigma_r_back
        if self.sigma_c_sink is None:
            self.sigma_c_sink = self.sigma_c
        if self.rho_rc != 0.0:
            raise NotImplementedError(
                "rho_rc != 0 needs R/C segment pairing (see "
                "draw_wire_multipliers); not implemented yet")
        if self.sheets not in ("shared", "independent"):
            raise ValueError(f"bad sheets mode {self.sheets}")
        return self

    def geom_sigmas(self):
        """Effective per-device sigma for W_NS/T_NS given the sheet mode."""
        scale = 1.0 if self.sheets == "shared" else 1.0 / np.sqrt(self.n_sheet)
        return self.sigma_w * scale, self.sigma_t * scale


# slot kinds, in RNG draw order (order is part of reproducibility)
KIND_VDD, KIND_DVT, KIND_L, KIND_W, KIND_T, KIND_R, KIND_C, KIND_VDDI = range(8)

MC_LINE_DROP = re.compile(
    r"^\.param (mc_mm_switch|mc_pr_switch|vdd_var|vddval|r_var|rfac|"
    r"c_var|cfac|ftsv\d+|fffc\d+)\s*=|^\.option modmonte")
RC_ELEM = re.compile(
    r"^([RC]\S*)\s+(\S+)\s+(\S+)\s+'([0-9.eE+-]+)\*(?:rfac|cfac|ftsv\d+|fffc\d+)'"
    r"\s*(?:\$.*)?$")
X_INST = re.compile(r"^(X\S*)\s+(.*)\s+(gt2_6t_\S+)\s*$")
M_LINE = re.compile(
    r"^(MM\d+)\s+(\S+)\s+(\S+)\s+(\S+)\s+(nmos_\S+|pmos_\S+)"
    r"\s+W=(\S+)\s+L=(\S+)\s+M=(\d+)\s*$")


class Template:
    """Parsed deck: list of str chunks and (kind, index) slots, in file order."""

    def __init__(self):
        self.chunks = []            # str | (kind, idx)
        self.counts = [0] * 8
        self.r_sigma_cat = []       # per R slot: 'front' | 'back' | 'tsv'
        self.c_sigma_cat = []       # per C slot: 'wire' | 'sink'
        self.finger_polarity = []   # per finger: 'p' | 'n'
        self.n_sinks = 0
        self.n_instances = 0
        self.meta = {}

    def slot(self, kind):
        idx = self.counts[kind]
        self.counts[kind] += 1
        self.chunks.append((kind, idx))
        return idx

    def text(self, s):
        self.chunks.append(s)


def parse_model_delvtrand(model_card: Path):
    """{model_name: nominal delvtrand} from the GT2N card."""
    noms, cur = {}, None
    for line in Path(model_card).read_text().splitlines():
        m = re.match(r"\s*\.MODEL\s+(\S+)\s+[NP]MOS", line, re.IGNORECASE)
        if m:
            cur = m.group(1)
            continue
        d = re.match(r"\s*\+\s*delvtrand\s*=\s*([-0-9.eE]+)\s*$", line)
        if d and cur:
            noms[cur] = float(d.group(1))
    if not noms:
        raise ValueError(f"no delvtrand found in {model_card}")
    return noms


def load_cdl_cells(cdl_path: Path):
    """Return {cellname: [(name, n1, n2, n3, model, W, L, M), ...]}."""
    cells, cur, body = {}, None, []
    for line in Path(cdl_path).read_text().splitlines():
        s = line.strip()
        m = re.match(r"^\.subckt\s+(\S+)\s+(.*)$", s, re.IGNORECASE)
        if m:
            cur, body = m.group(1), []
            cells[cur] = {"ports": m.group(2).split(), "devs": body}
            continue
        if re.match(r"^\.ends", s, re.IGNORECASE):
            cur = None
            continue
        if cur:
            dm = M_LINE.match(s)
            if dm:
                body.append(dm.groups())
            elif s and not s.startswith("*"):
                # only an error if the deck actually instantiates this cell
                cells[cur]["bad"] = s
    return cells


def build_template(deck_path: Path, cdl_path: Path, model_card: Path,
                   cfg: VarConfig) -> Template:
    cells = load_cdl_cells(cdl_path)
    dvt_noms = parse_model_delvtrand(model_card)
    tp = Template()
    tp.finger_dvt_nom = []
    used = set()
    root_node = None
    tran_seen = False
    # measure t_root on the same clock edge as the sink measures (the CTS
    # tree deck uses RISE=2 - a delayed root pulse - the mesh decks RISE=1)
    deck_text = Path(deck_path).read_text()
    em = re.search(r"^\.measure tran t_sink_0 .*RISE=(\d)", deck_text, re.M)
    sink_edge = em.group(1) if em else "1"

    for raw in deck_text.splitlines():
        line = raw.rstrip("\n")
        s = line.strip()

        if MC_LINE_DROP.match(s):
            if s.startswith(".param vddval"):
                tp.text(".param vddval=")
                tp.slot(KIND_VDD)
                tp.text("\n")
            continue
        if s.startswith(".include"):
            inc = s.split(None, 1)[1]
            if inc.endswith(".cdl"):
                # replaced by per-instance uniquified subckts (emitted inline
                # at each X line), nothing global to include
                continue
            if "renamed_mc" in inc or "_tt" in inc:
                tp.text(f".include {model_card}\n")
                continue
            tp.text(line + "\n")
            continue

        m = re.match(r"^Vclk\S*\s+(\S+)\s", s)
        if m:
            root_node = m.group(1)

        rc = RC_ELEM.match(s)
        if rc:
            name, n1, n2, base = rc.group(1), rc.group(2), rc.group(3), float(rc.group(4))
            if name[0] == "R":
                if name.startswith("Rtsv"):
                    cat = "tsv"
                elif name.startswith("Rclk_i_mesh"):
                    cat = "back" if cfg.mesh_is_backside else "front"
                else:
                    cat = "front"
                tp.text(f"{name} {n1} {n2} ")
                tp.slot(KIND_R)
                tp.r_sigma_cat.append(cat)
                tp.text(f" $ base={base:g}\n")
                tp.meta.setdefault("r_bases", []).append(base)
            else:
                cat = "sink" if re.match(r"Csink\d+$", name) else "wire"
                tp.text(f"{name} {n1} {n2} ")
                tp.slot(KIND_C)
                tp.c_sigma_cat.append(cat)
                tp.text(f" $ base={base:g}\n")
                tp.meta.setdefault("c_bases", []).append(base)
            continue

        xm = X_INST.match(s)
        if xm:
            iname, nodes, cell = xm.groups()
            if cell not in cells:
                raise ValueError(f"{iname}: cell {cell} not in {cdl_path}")
            if "bad" in cells[cell]:
                raise ValueError(
                    f"cell {cell}: unhandled CDL line: {cells[cell]['bad']}")
            ucell = f"u{tp.n_instances}_{cell.split('gt2_6t_')[1]}"
            inst_idx = tp.n_instances
            tp.n_instances += 1
            used.add(cell)
            if cfg.vdd_scope == "iid":
                ntoks = nodes.split()
                ntoks = [f"vddu{inst_idx}" if t == "VDD" else t for t in ntoks]
                nodes = " ".join(ntoks)
            # uniquified subckt: fingers expanded, per-finger variation slots
            tp.text(f".subckt {ucell} {' '.join(cells[cell]['ports'])}\n")
            for (dn, d, g, sn, model, W, L, M) in cells[cell]["devs"]:
                pol = "p" if model.startswith("pmos") else "n"
                if model not in dvt_noms:
                    raise ValueError(f"model {model} has no delvtrand "
                                     f"nominal in {model_card}")
                for f in range(int(M)):
                    tp.text(f"{dn}_f{f} {d} {g} {sn} {model} W={W} L=")
                    tp.slot(KIND_L)
                    tp.text(" tfin=")
                    tp.slot(KIND_T)
                    tp.text(" delvtrand=")
                    tp.slot(KIND_DVT)
                    tp.text(" M=")
                    tp.slot(KIND_W)  # W_NS emulated as fractional M
                    tp.text("\n")
                    tp.finger_polarity.append(pol)
                    tp.finger_dvt_nom.append(dvt_noms[model])
            tp.text(f".ends {ucell}\n")
            tp.text(f"{iname} {nodes} {ucell}\n")
            if cfg.vdd_scope == "iid":
                tp.text(f"Vvddu{inst_idx} vddu{inst_idx} 0 ")
                tp.slot(KIND_VDDI)
                tp.text("\n")
            continue

        if s.startswith(".measure tran t_sink_"):
            tp.n_sinks += 1
        if s.startswith(".tran"):
            if root_node is None:
                raise ValueError("no Vclk* source found before .tran")
            tp.text(f".measure tran t_root WHEN v({root_node})='vddval*0.5' RISE={sink_edge}\n")
            tp.text(re.sub(r"\s+sweep\s+monte=\d+", "", line) + "\n")
            tran_seen = True
            continue

        tp.text(line + "\n")

    if not tran_seen:
        raise ValueError("deck has no .tran")
    if tp.counts[KIND_VDD] != 1:
        raise ValueError("expected exactly one vddval param in deck")
    tp.meta.update(
        deck=str(deck_path), cdl=str(cdl_path), model_card=str(model_card),
        deck_md5=hashlib.md5(Path(deck_path).read_bytes()).hexdigest(),
        n_sinks=tp.n_sinks, n_instances=tp.n_instances,
        n_fingers=tp.counts[KIND_DVT], n_r=tp.counts[KIND_R],
        n_c=tp.counts[KIND_C],
        r_cats={c: tp.r_sigma_cat.count(c) for c in ("front", "back", "tsv")},
        c_cats={c: tp.c_sigma_cat.count(c) for c in ("wire", "sink")},
        cells=sorted(used))
    # freeze arrays used at render time
    tp.r_base = np.array(tp.meta.pop("r_bases", []))
    tp.c_base = np.array(tp.meta.pop("c_bases", []))
    tp.r_sig_idx = np.array([{"front": 0, "back": 1, "tsv": 2}[c]
                             for c in tp.r_sigma_cat], dtype=int)
    tp.c_sig_idx = np.array([{"wire": 0, "sink": 1}[c]
                             for c in tp.c_sigma_cat], dtype=int)
    tp.pmask = np.array([p == "p" for p in tp.finger_polarity])
    tp.dvt_nom = np.array(tp.finger_dvt_nom)
    return tp


def draw_wire_multipliers(rng, tp: Template, cfg: VarConfig):
    """The single seam for wire R/C draws (future rho_rc pairing goes here)."""
    r_sigma = np.array([cfg.sigma_r_front, cfg.sigma_r_back,
                        cfg.sigma_r_tsv])[tp.r_sig_idx]
    c_sigma = np.array([cfg.sigma_c, cfg.sigma_c_sink])[tp.c_sig_idx]
    r_mult = 1.0 + r_sigma * rng.standard_normal(tp.counts[KIND_R])
    c_mult = 1.0 + c_sigma * rng.standard_normal(tp.counts[KIND_C])
    # negative-value guard (only reachable if a sigma is pushed >~33%)
    np.clip(r_mult, 0.01, None, out=r_mult)
    np.clip(c_mult, 0.01, None, out=c_mult)
    return r_mult, c_mult


def draw_sample(tp: Template, cfg: VarConfig, base_seed: int, idx: int):
    """Return {kind: value array} for sample idx.  idx 0 = nominal."""
    nf = tp.counts[KIND_DVT]
    n_vi = tp.counts[KIND_VDDI]
    if idx == 0:
        return {
            KIND_VDD: np.array([cfg.vdd_nom]),
            KIND_VDDI: np.full(n_vi, cfg.vdd_nom),
            KIND_DVT: tp.dvt_nom.copy(),
            KIND_L: np.full(nf, cfg.l_nom),
            KIND_W: np.ones(nf),          # M multiplier (W_NS emulation)
            KIND_T: np.full(nf, cfg.t_nom),
            KIND_R: tp.r_base.copy(),
            KIND_C: tp.c_base.copy(),
        }
    rng = np.random.default_rng(
        np.random.SeedSequence(entropy=base_seed, spawn_key=(idx,)))
    # fixed draw order: vdd, dvt, L, W, T, wire R, wire C
    if cfg.vdd_scope == "iid":
        # per-cell rails; thresholds and root pulse stay at the nominal rail
        vdd = cfg.vdd_nom
        vddi = cfg.vdd_nom + cfg.sigma_vdd * rng.standard_normal(n_vi)
    else:
        vdd = cfg.vdd_nom + cfg.sigma_vdd * rng.standard_normal()
        vddi = np.full(n_vi, vdd)
    dvt_sigma = np.where(tp.pmask, cfg.sigma_vth_p, cfg.sigma_vth_n)
    dvt = tp.dvt_nom + dvt_sigma * rng.standard_normal(nf)
    sw, st = cfg.geom_sigmas()
    lg = cfg.l_nom + cfg.sigma_l * rng.standard_normal(nf)
    dw = sw * rng.standard_normal(nf)
    t = cfg.t_nom + st * rng.standard_normal(nf)
    # W_NS -> fractional M via the model's Weff = NFIN*(2*HFIN + TFIN)
    mfac = 1.0 + 2.0 * dw / (2.0 * cfg.w_nom + cfg.t_nom)
    r_mult, c_mult = draw_wire_multipliers(rng, tp, cfg)
    return {
        KIND_VDD: np.array([vdd]),
        KIND_VDDI: vddi,
        KIND_DVT: dvt,
        KIND_L: np.clip(lg, 1e-9, None),
        KIND_W: np.clip(mfac, 0.01, None),
        KIND_T: np.clip(t, 0.5e-9, None),
        KIND_R: tp.r_base * r_mult,
        KIND_C: tp.c_base * c_mult,
    }


def render(tp: Template, vals: dict) -> str:
    fmt = {KIND_VDD: "%.6f", KIND_VDDI: "%.6f", KIND_DVT: "%.6e",
           KIND_L: "%.5e", KIND_W: "%.6f", KIND_T: "%.5e",
           KIND_R: "%.6g", KIND_C: "%.5g"}
    out = []
    for ch in tp.chunks:
        if isinstance(ch, str):
            out.append(ch)
        else:
            kind, idx = ch
            out.append(fmt[kind] % vals[kind][idx])
    return "".join(out)


# ---------------------------------------------------------------- mt0 parse

def parse_mt0(path: Path) -> dict:
    """Parse an HSPICE .mt0 (single run or sweep row) into {name: value}.

    Handles both MEASFORM=1 (an 'index ...' names line, possibly wrapped,
    then value rows) and the classic wrapped names-block/values-block form.
    'failed' measures become NaN.
    """
    toks_names, toks_vals = [], []
    in_vals = False
    for line in Path(path).read_text().splitlines():
        if line.startswith("$") or line.startswith("."):
            continue
        toks = line.split()
        if not toks:
            continue
        if not in_vals:
            numeric = all(_is_num_or_failed(t) for t in toks)
            if toks_names and numeric:
                in_vals = True
            else:
                toks_names += toks
                continue
        toks_vals += toks
    if not toks_names or not toks_vals:
        raise ValueError(f"{path}: could not parse names/values")
    if toks_names[0] == "index":
        pass
    vals = [float("nan") if t.lower() == "failed" else float(t)
            for t in toks_vals]
    n = len(toks_names)
    if len(vals) % n != 0:
        raise ValueError(f"{path}: {len(vals)} values not a multiple of "
                         f"{n} names")
    if len(vals) != n:
        raise ValueError(f"{path}: expected a single row, got {len(vals)//n}")
    return dict(zip(toks_names, vals))


def _is_num_or_failed(t: str) -> bool:
    if t.lower() == "failed":
        return True
    try:
        float(t)
        return True
    except ValueError:
        return False
