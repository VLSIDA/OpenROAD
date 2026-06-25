#!/usr/bin/env python3
"""Parse OpenROAD SPEF and compute clock mesh dynamic wire + pin-cap power.

P1 = CTS tree wires   (clknet_* nets) + mesh buffer input pin caps
P2 = Mesh output wires (clk_mesh net) + FF sink clock pin caps
P_dynamic_total = P1 + P2

Usage:
  python3 parse_spef_power.py results/mesh_power-tcl.spef [results/mesh_pin_caps-tcl.txt] [--vdd 0.7] [--freq_ghz 2.222]
"""
import re, sys, os

spef_file    = sys.argv[1]
pin_cap_file = None
vdd          = 0.7
freq_ghz     = 2.222

i = 2
while i < len(sys.argv):
    if sys.argv[i] == '--vdd':        vdd          = float(sys.argv[i+1]); i += 2
    elif sys.argv[i] == '--freq_ghz': freq_ghz     = float(sys.argv[i+1]); i += 2
    elif not sys.argv[i].startswith('--'):
        pin_cap_file = sys.argv[i]; i += 1
    else: i += 1

freq = freq_ghz * 1e9

# ── Read pin cap file (optional) ──────────────────────────────────────────
C_buf_pin_fF  = 0.0
C_sink_pin_fF = 0.0
n_buf_pins    = 0
n_sinks       = 0

if pin_cap_file and os.path.exists(pin_cap_file):
    with open(pin_cap_file) as f:
        for line in f:
            line = line.strip()
            if line.startswith('#'): continue
            parts = line.split()
            if len(parts) < 2: continue
            key, val = parts[0], parts[1]
            if   key == 'BUF_PIN_CAP_fF':  C_buf_pin_fF  = float(val)
            elif key == 'BUF_PIN_COUNT':    n_buf_pins    = int(val)
            elif key == 'SINK_PIN_CAP_fF':  C_sink_pin_fF = float(val)
            elif key == 'SINK_COUNT':       n_sinks       = int(val)
            elif key == 'VDD':              vdd           = float(val)
            elif key == 'FREQ_HZ':          freq          = float(val); freq_ghz = freq / 1e9

# ── Pass 1: read NAME_MAP and C_UNIT ──────────────────────────────────────
name_map  = {}   # index string -> net name  e.g. "5375" -> "clk_mesh"
c_unit_pf = 1.0  # default: PF (pico-farads)

with open(spef_file) as f:
    in_map = False
    for line in f:
        line = line.strip()

        # C_UNIT line e.g.  *C_UNIT 1 PF  or  *C_UNIT 1 FF
        if line.startswith('*C_UNIT'):
            parts = line.split()
            if len(parts) >= 3:
                mult  = float(parts[1])
                unit  = parts[2].upper()
                if   unit == 'PF': c_unit_pf = mult * 1.0
                elif unit == 'FF': c_unit_pf = mult * 1e-3
                elif unit == 'NF': c_unit_pf = mult * 1e3
                elif unit == 'F':  c_unit_pf = mult * 1e12
            continue

        if line == '*NAME_MAP':
            in_map = True
            continue
        if in_map:
            if line == '' or line.startswith('*D_NET') or line.startswith('*PORTS'):
                in_map = False
            else:
                # e.g.  *5375 clk_mesh
                m = re.match(r'^\*(\d+)\s+(\S+)', line)
                if m:
                    name_map[m.group(1)] = m.group(2)

# ── Pass 2: sum *D_NET caps for clknet_* and clk_mesh ─────────────────────
C_cts_pf  = 0.0
C_mesh_pf = 0.0
n_cts     = 0

with open(spef_file) as f:
    for line in f:
        if not line.startswith('*D_NET'):
            continue
        parts = line.split()
        if len(parts) < 3:
            continue

        # net identifier: either *INDEX or a plain name
        raw  = parts[1]
        idx  = re.match(r'^\*(\d+)$', raw)
        if idx:
            net_name = name_map.get(idx.group(1), raw)
        else:
            net_name = raw.lstrip('*|')

        try:
            cap_pf = float(parts[2]) * c_unit_pf   # convert to pF
        except ValueError:
            continue

        if net_name == 'clk_mesh':
            C_mesh_pf = cap_pf
        elif net_name.startswith('clknet_'):
            C_cts_pf += cap_pf
            n_cts += 1

# Convert pF -> F for power formula
C_cts_f      = C_cts_pf      * 1e-12
C_mesh_f     = C_mesh_pf     * 1e-12
C_buf_pin_f  = C_buf_pin_fF  * 1e-15
C_sink_pin_f = C_sink_pin_fF * 1e-15

vv_f = vdd * vdd * freq

P1_wire_mW    = C_cts_f      * vv_f * 1e3
P1_pin_mW     = C_buf_pin_f  * vv_f * 1e3
P1_mW         = P1_wire_mW + P1_pin_mW

P2_wire_mW    = C_mesh_f     * vv_f * 1e3
P2_pin_mW     = C_sink_pin_f * vv_f * 1e3
P2_mW         = P2_wire_mW + P2_pin_mW

print()
print("================================================================")
print("  Clock Mesh Dynamic Power  (C * V^2 * f, switching only)")
print("================================================================")
print(f"  VDD = {vdd} V    Freq = {freq_ghz} GHz    C_UNIT = {c_unit_pf} pF")
print()
print(f"  P1  CTS tree  (root -> mesh buffer inputs)")
print(f"      clknet_* wire nets : {n_cts}")
print(f"      C_wire_cts         : {C_cts_pf*1e3:.2f} fF  ({C_cts_pf:.4f} pF)  -> {P1_wire_mW:.3f} mW")
if n_buf_pins:
    print(f"      C_buf_pin_caps     : {C_buf_pin_fF:.2f} fF  ({n_buf_pins} pins)  -> {P1_pin_mW:.3f} mW")
print(f"      P1 total           : {P1_mW:.3f} mW")
print()
print(f"  P2  Mesh output  (mesh buffer outputs -> FF sinks)")
print(f"      C_wire_mesh        : {C_mesh_pf*1e3:.2f} fF  ({C_mesh_pf:.4f} pF)  -> {P2_wire_mW:.3f} mW")
if n_sinks:
    print(f"      C_sink_pin_caps    : {C_sink_pin_fF:.2f} fF  ({n_sinks} sinks)  -> {P2_pin_mW:.3f} mW")
print(f"      P2 total           : {P2_mW:.3f} mW")
print()
print(f"  P_dynamic_total  = P1 + P2 = {P1_mW + P2_mW:.3f} mW")
print()
print("  (excludes mesh buffer internal transistor switching power)")
print()
print(f"  CTS baseline switching power : 0.855 mW  (report_power Switching col)")
print(f"  CTS baseline total clock pwr : 2.10  mW  (internal + switching)")
print("================================================================")
