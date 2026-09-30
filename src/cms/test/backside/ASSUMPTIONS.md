# Backside Clock-Mesh Flow — Assumptions Inventory

Every simplification made in the flow, what it affects, and what would replace
it. Severity: **H** = could change conclusions, **M** = affects accuracy,
**L** = minor/conventional.

## SPICE / skew (gcd_mesh_skew.sp)

| # | assumption | sev | replacement |
|---|---|---|---|
| 1 | **Analytic RC, no coupling caps** — ground C from ITF plate+fringe (1.5x fringe factor), context-independent; Cc dropped entirely | M | real OpenRCX rules generated via FasterCap/StarRC golden SPEF (PDK ships nxtgrd/qrc inputs) |
| 2 | **TSV = pure 149 ohm series R** — no via capacitance, no distributed stack RC; one cut of each via type per crossing (V0 54.99 + VSD 36.86 + VBPR 32.0 + BV0 25.10 from ITF) | M | RC ladder for the stack + via caps; redundant-cut layout would lower R |
| 3 | **Tree abstracted as PULSE sources** at mesh-buffer inputs (STA arrivals + slews baked in); arrivals from `estimate_parasitics -global_routing`, not extraction | M | `-full_tree` mode (tree simulated transistor-level) + extracted tree parasitics |
| 4 | **tt / 0.7 V / 25 C only**; Monte-Carlo mismatch disabled | H for signoff | corner sweep (PDK has all W/Vt tt libs; other corners would need PDK regen) |
| 5 | Skew = 50 % VDD crossing, **first rising edge only** | L | add falling-edge + duty-cycle measures |
| 6 | FF loads = **liberty scalar pin cap** (linear C, no Miller) | L | transistor-level FF clock pins from CDL |
| 7 | Transistor deck contains only **w31_lvt** cells (buffers); w13/other-Vt appear only as caps | L (no such cells instantiated) | include per-family model cards (beware .MODEL name collisions across width cards) |
| 8 | **setRC wire values are analytic ITF derivations** (R/um = RPSQ/WMIN assumes min width; C approximate) — platform README itself flags this | M | calibrated extraction (same fix as #1) |

## PDN / IR

| # | assumption | sev | replacement |
|---|---|---|---|
| 9 | **vsrc = entire BM2 straps are ideal 0.7 V sources** — no bump pitch, no package R/L (fine at gcd scale: 8.6 um < any bump pitch) | H at chip scale | vsrc points only at real backside bump sites + package model |
| 10 | **Static IR only** — no di/dt, no decap modeling; budget assumes dynamic fits in the other half | M | dynamic IR (PSM supports transient w/ activity) + decap insertion |
| 11 | IR **budget 2.5 % of Vdd = policy convention** (half of a 5 % rail share of a 10 % total-noise allowance), not chip-specific timing analysis | L | derive from actual setup slack sensitivity (ps of slack per mV) |
| 12 | Sizing at **alpha = 1** (every net toggles every cycle) — ultra-conservative; realistic peak 0.3-0.5 | L (conservative) | activity from simulation (.saif/.vcd) |
| 13 | **EM check = rule-of-thumb mA/um**, not PDK EM rules | M | real current-density limits (not in the open PDK) |
| 14 | Historic caveat: pre-2026-07-05 IR numbers used single-liberty sessions (understated); worksheet demand now uses **report_power** (validated within ~20 % of hand calc for switching) | — | done — use report_power path |

## Clock-mesh design choices

| # | assumption | sev | note |
|---|---|---|---|
| 15 | Sink capacity **C = min(lib, 16)** — 16 is a heuristic clock-fanout target | L | rarely binding (max real fanout 4) |
| 16 | **x4 buffers everywhere; drive TSV at every intersection** (user choice) — buffers are 77 % of clock power | M (power) | sparser drivers / smaller sinks if power matters |
| 17 | **Mesh pitch 1.0 um** chosen, not swept | M | pitch sweep vs skew/power tradeoff |
| 18 | **2x min spacing (0.112 um)** = interference rule — heuristic, not noise-analyzed | L | coupling analysis (needs #1) |
| 19 | **Overlap-as-connection**: b_* nets physically overlap clk_mesh metal but keep separate names — LVS/STA see two nets; SPICE ties them via node aliases | M (verification) | net merge for signoff netlists (write_mesh_verilog handles the Verilog view) |
| 20 | **TSV cell abstraction**: passive Y=A LEF; internal nano-TSV stack per the GT2N paper; custom-cell GDS/DRC assumed clean | M | DRC/LVS the bridge cell against PDK icv_runset |
| 21 | BPR-break IR-safety + strap counts validated **on gcd only** (35 FFs, 8.6 um core) | H for generality | rerun on aes/jpeg (worksheet + flow scale automatically) |

## Measurement / environment

| # | assumption | sev | note |
|---|---|---|---|
| 22 | gcd is a toy: 0.72 pF switched cap — all "few straps suffice" conclusions scale with C*f | H | aes is the ready stress test |
| 23 | Remaining 165 frontside DRC treated as research residue | L | ordinary detail-route cleanup |
| 24 | hspice M-2017.03 vs BSIM-CMG card version compatibility trusted (job concluded cleanly) | L | spot-check vs paper's published cell delays |
