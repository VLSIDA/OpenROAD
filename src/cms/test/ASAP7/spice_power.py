#!/usr/bin/env python3
"""Estimate clock mesh switching power from SPICE file capacitances.

P_mesh = C_total * VDD^2 * f

Grounded C elements are the capacitors charged/discharged each clock cycle.
Coupling Cc elements (same-net) carry no net voltage swing so are excluded.

Usage:
  python3 spice_power.py mesh.spice [--vdd 0.7] [--freq_ghz 1.0]
"""
import re, sys

def parse_val(s):
    """Parse SPICE value string (supports e-notation, no suffix units)."""
    try:
        return float(s)
    except ValueError:
        return None

spice_file = sys.argv[1]
vdd = 0.7
freq_ghz = 1.0
i = 2
while i < len(sys.argv):
    if sys.argv[i] == '--vdd':       vdd = float(sys.argv[i+1]);      i += 2
    elif sys.argv[i] == '--freq_ghz': freq_ghz = float(sys.argv[i+1]); i += 2
    else: i += 1

c_total_f  = 0.0
c_count    = 0
cc_total_f = 0.0
cc_count   = 0

with open(spice_file) as f:
    for line in f:
        line = line.strip()
        if not line or line.startswith('*') or line.startswith('.'):
            continue
        parts = line.split()
        name = parts[0].upper()

        if name.startswith('CC'):
            # Coupling cap: Cc_name node1 node2 value
            if len(parts) >= 4:
                val = parse_val(parts[3])
                if val is not None:
                    cc_total_f += val
                    cc_count   += 1
        elif name.startswith('C'):
            # Grounded cap: C_name node 0 value
            if len(parts) >= 4:
                val = parse_val(parts[3])
                if val is not None:
                    c_total_f += val
                    c_count   += 1

freq_hz = freq_ghz * 1e9
p_w = c_total_f * (vdd ** 2) * freq_hz

print(f"SPICE file     : {spice_file}")
print(f"VDD            : {vdd} V")
print(f"Clock freq     : {freq_ghz} GHz  (period = {1/freq_ghz:.3g} ns)")
print()
print(f"Grounded caps  : {c_count:6d}   C_total = {c_total_f*1e15:9.1f} fF  = {c_total_f*1e12:.3f} pF")
print(f"Coupling caps  : {cc_count:6d}   C_total = {cc_total_f*1e15:9.1f} fF  (same-net, excluded)")
print()
print(f"Mesh clock switching power:")
print(f"  P = {c_total_f*1e15:.1f} fF * ({vdd}V)^2 * {freq_ghz}GHz")
print(f"  P = {p_w*1e3:.3f} mW")
print()
print(f"CTS baseline reference  : Clock = 2.10 mW  (from report_power)")
print(f"Mesh (this result)      : Clock = {p_w*1e3:.2f} mW")
if p_w > 0:
    ratio = 2.10 / (p_w * 1e3)
    print(f"CTS / Mesh ratio        : {ratio:.2f}x  ({'mesh saves power' if ratio > 1 else 'mesh uses more power'})")
