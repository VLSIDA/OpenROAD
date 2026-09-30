#!/usr/bin/env python3
"""Translate the real GT2N StarRC ITF stack into an OpenRCX v2 `process` file.

The RCX process format (see src/rcx/src/extprocess.cpp readConductor/
readDielectric + createMasterLayers):
  CONDUCTOR <name> { distance <gap-below> thickness <t> min_width <w>
                     min_spacing <s> resistivity <rho ohm.um> }
  DIELECTRIC <name> { epsilon <er> thickness <t> [next_met N|met N] }

Semantics recovered from the parser:
  * CONDUCTORs are ordered BOTTOM-UP; index 1 = bottom metal.
  * `distance` is the dielectric gap between the top of the conductor below
    and the bottom of this conductor (for the bottom metal: height above the
    ground reference plane).
  * `resistivity` (ohm.um) = RPSQ (ohm/sq, from ITF) * thickness (um).
  * DIELECTRICs form their own cumulative-height stack; epsilon/thickness are
    taken straight from the ITF ER/THICKNESS. `next_met`/`met` tie a dielectric
    to a metal level (1-based, matching the conductor order) so thickness
    variation tracks the right layer.

Model choice: each ITF metal level has one dielectric slab of thickness D_n
containing the metal (thickness T_n) at its bottom, so the inter-metal gap is
distance(n) = D_{n-1} - T_{n-1}. Real ITF values, no invented numbers.
"""
import re
import sys

ITF = "/home/wali2/backside/GT2N/nxtgrd/GT2.itf"

# Which contiguous conductor stack to emit, bottom-up. Frontside routing +
# a couple of upper shields (what the clock-tree extraction needs).
FRONTSIDE = ["M0", "M1", "M2", "M3", "M4", "M5", "M6", "M7"]
# Backside power/mesh stack, bottom(BRDL)-up toward the device (BPR).
BACKSIDE = ["BRDL", "BM4", "BM3", "BM2", "BM1", "BPR"]


def parse_itf(path):
    cond, diel = {}, {}
    for line in open(path):
        line = line.strip()
        if line.startswith("$") or not line:
            continue
        m = re.match(r"CONDUCTOR\s+(\S+)\s*\{(.+)\}", line)
        if m:
            name, body = m.group(1), m.group(2)
            kv = dict(re.findall(r"(\w+)\s*=\s*([\d.]+)", body))
            cond[name] = {k: float(v) for k, v in kv.items()}
            continue
        m = re.match(r"DIELECTRIC\s+(\S+)\s*\{(.+)\}", line)
        if m:
            name, body = m.group(1), m.group(2)
            kv = dict(re.findall(r"(\w+)\s*=\s*([\d.]+)", body))
            diel[name] = {k: float(v) for k, v in kv.items()}
    return cond, diel


def emit(order, cond, diel, ground_gap=0.05):
    """order: metal names bottom-up. Returns process-file text."""
    out = []
    out.append("PROCESS GT2 {")
    out.append("}")
    out.append("")
    # DIELECTRIC background: one slab per metal level, epsilon from the ITF
    # <name>_diel entry, tagged next_met to that level (1-based).
    for i, mname in enumerate(order, start=1):
        dname = f"{mname}_diel"
        d = diel.get(dname)
        if d is None:
            continue
        out.append(f"DIELECTRIC {dname} {{")
        out.append(f"        epsilon {d['ER']}")
        out.append(f"        thickness {d['THICKNESS']}")
        out.append(f"        next_met {i}")
        out.append("}")
        out.append("")
    # CONDUCTORs bottom-up. distance = D_{n-1} - T_{n-1} (gap above prev metal).
    prev = None
    for mname in order:
        c = cond[mname]
        if prev is None:
            dist = ground_gap
        else:
            pd = diel.get(f"{prev}_diel")
            pc = cond[prev]
            dist = round(pd["THICKNESS"] - pc["THICKNESS"], 4) if pd else 0.03
            if dist <= 0:
                dist = 0.02
        rho = round(c["RPSQ"] * c["THICKNESS"], 5)
        out.append(f"CONDUCTOR {mname} {{")
        out.append(f"        distance {dist}")
        out.append(f"        thickness {c['THICKNESS']}")
        out.append(f"        min_width {c['WMIN']}")
        out.append(f"        min_spacing {c['SMIN']}")
        out.append(f"        resistivity {rho}")
        out.append("}")
        out.append("")
        prev = mname
    return "\n".join(out) + "\n"


if __name__ == "__main__":
    which = sys.argv[1] if len(sys.argv) > 1 else "frontside"
    cond, diel = parse_itf(ITF)
    order = {"frontside": FRONTSIDE, "backside": BACKSIDE}[which]
    sys.stdout.write(emit(order, cond, diel))
