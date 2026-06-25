#!/usr/bin/env python3
"""Patch a mesh SPICE file to add complete power measurement.

Total mesh power = output side + input side
  - I(Vvdd)          : buffer internal switching + mesh RC wire + sink cap
  - sum I(Vclk_N)    : gate cap charging at each buffer input (upstream load)

Writes patched file alongside original with _power.spice suffix.

Usage:
  python3 patch_spice_power.py mesh.spice
"""
import re, sys

spice_in  = sys.argv[1]
spice_out = spice_in.replace('.spice', '_power.spice')

with open(spice_in) as f:
    lines = f.readlines()

# Find period and VDD from first PULSE line
period_ns = None
vdd       = None
for line in lines:
    m = re.search(r'PULSE\(\s*0\s+(\S+)\s+\S+\s+\S+\s+\S+\s+\S+\s+(\S+)n\)', line, re.IGNORECASE)
    if m:
        vdd       = float(m.group(1))
        period_ns = float(m.group(2))
        break

if period_ns is None:
    print("ERROR: no PULSE source found"); sys.exit(1)

# Find max arrival and collect all Vclk source names
max_arrival_ns = 0.0
vclk_names = []
for line in lines:
    m = re.match(r'(Vclk_\d+)\s', line, re.IGNORECASE)
    if m:
        vclk_names.append(m.group(1))
    m2 = re.search(r'PULSE\(\s*0\s+\S+\s+(\S+)n', line, re.IGNORECASE)
    if m2:
        td = float(m2.group(1))
        if td > max_arrival_ns:
            max_arrival_ns = td

# Find and patch .tran line
tran_idx  = None
tran_step = None
old_stop  = None
for i, line in enumerate(lines):
    m = re.match(r'(\.tran\s+)(\S+)(n\s+)(\S+)(n)', line, re.IGNORECASE)
    if m:
        tran_idx  = i
        tran_step = float(m.group(2))
        old_stop  = float(m.group(4))
        break

if tran_idx is None:
    print("ERROR: .tran not found"); sys.exit(1)

# Extend by one full period so rising+falling edges are both covered
new_stop_ns  = old_stop + period_ns
measure_from = max_arrival_ns
measure_to   = max_arrival_ns + period_ns

lines[tran_idx] = f'.tran {tran_step}n {new_stop_ns:.5f}n\n'

# Build power measurement block
power_lines = ['\n* ---- Power measurement ----\n']

# Output side: VDD current (buffer switching + mesh RC + sink caps)
power_lines.append(
    f'.measure tran avg_I_out AVG I(Vvdd) FROM={measure_from:.5f}n TO={measure_to:.5f}n\n'
)
power_lines.append(
    f'.measure tran P_out_mW param=\'{vdd}*avg_I_out*1e3\' $ output-side power (mW)\n'
)

# Input side: sum of all Vclk source currents (buffer gate cap charging)
for name in vclk_names:
    power_lines.append(
        f'.measure tran avg_I_{name} AVG I({name}) FROM={measure_from:.5f}n TO={measure_to:.5f}n\n'
    )

# Build sum expression for total input current
if vclk_names:
    sum_expr = '+'.join([f'avg_I_{n}' for n in vclk_names])
    power_lines.append(
        f'.measure tran avg_I_in param=\'{sum_expr}\' $ total input current (A)\n'
    )
    power_lines.append(
        f'.measure tran P_in_mW param=\'{vdd}*avg_I_in*1e3\' $ input-side power (mW)\n'
    )
    power_lines.append(
        f'.measure tran P_total_mW param=\'abs(P_out_mW)+abs(P_in_mW)\' $ TOTAL mesh power (mW)\n'
    )

end_idx = next(i for i, l in enumerate(lines) if l.strip().lower() == '.end')
for j, pl in enumerate(power_lines):
    lines.insert(end_idx + j, pl)

with open(spice_out, 'w') as f:
    f.writelines(lines)

print(f"Patched  : {spice_out}")
print(f"Period   : {period_ns} ns   VDD: {vdd} V")
print(f"Measure  : FROM={measure_from:.4f}ns  TO={measure_to:.4f}ns")
print(f"Buffers  : {len(vclk_names)} Vclk sources found")
print(f"Tran     : stop extended {old_stop:.5f}n -> {new_stop_ns:.5f}n")
print(f"\nAfter HSpice, look for:")
print(f"  P_out_mW   = output side (mesh RC + buffers)")
print(f"  P_in_mW    = input side  (buffer gate caps)")
print(f"  P_total_mW = complete mesh power")
