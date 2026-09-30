# Backside Clock Mesh — Results (jpeg, GT2N 2nm)

Frontside clock TREE (M1–M5 CTS) + clock MESH distribution, comparing a **backside**
mesh (BM1/BM2 + gt2_6t_TSV bridge) against a **frontside** mesh (M5/M6, no TSVs).
All SPICE at VDD=0.7 V, 1 ns period, 4384 flip-flops, x4 sink buffers, balanced PDN,
6 Ω nano-TSV. Skew = P1–P99 of sink arrivals; σ = std-dev; insertion = mean arrival.

## Headline comparison

| config | sink taps | skew (P1–P99) | σ | insertion | power | skew vs FS | power vs FS |
|--------|-----------|---------------|------|-----------|--------|------------|-------------|
| **F_max=8**  | 600\* | **10.8 ps** | 2.37 | 298.3 ps | 5.915 mW | **−61.6%** ✅ | +20.6% |
| **F_max=16** | 333   | **17.2 ps** | 3.56 | 309.0 ps | 5.602 mW | **−38.8%** ✅ | +14.2% |
| grid baseline | 452  | 22.0 ps | 4.40 | —        | 5.826 mW | −21.7% ✅ | +18.8% |
| F_max=24 | 242      | 30.0 ps | 6.28 | 313.9 ps | 5.538 mW | +6.8% ❌ | +12.9% |
| F_max=80 | 101      | 95.5 ps | 19.0 | 323.7 ps | 5.354 mW | +240% ❌ | +9.2% |
| **frontside (ref)** | — | 28.1 ps | 6.84 | 325.6 ps | 4.904 mW | — | — |

\* F_max=8 saturates the 600-site candidate grid (0 free sites, 32 FFs spilled/dropped →
deck holds 4352 of 4384). Practical floor for this 25×25 mesh pitch.

## PDN configuration (differs by case — by design)

| case | clock mesh on | PDN loaded | PDN style |
|------|---------------|-----------|-----------|
| Frontside | M5/M6 (frontside) | `3_place.odb` | upstream stock ORFS PDN — dense, ~26–27 narrow straps/net @ 0.2 µm |
| Backside (all F_max) | BM1/BM2 (backside) | `jpeg_pdn_balanced.odb` | balanced rebuild — 4 wide straps/net |

Both PDNs sit on the **backside** (BM1/BM2) — GT2N is a BSPDN tech, so power is always
delivered from the back regardless of which side the clock mesh is on. The PDNs differ
because of where the mesh lives: the **backside** mesh shares BM1/BM2 with the PDN, so
the PDN was thinned to the balanced 4-strap version to leave routing room for the mesh;
the **frontside** mesh (M5/M6) doesn't touch BM1/BM2, so the PDN keeps full stock density.

IMPORTANT: this PDN difference does **not** affect the skew/insertion/power numbers above
— the SPICE decks drive the clock net from an **ideal 0.7 V supply** (`Vvdd VDD 0 0.7`),
so PDN geometry never enters the netlist. PDN only matters for (1) routing/legalization
feasibility and (2) IR-drop (PSM), which is NOT included here. For a power-integrity-fair
comparison, run PSM on both — watch the backside case's thinner PDN + 1225 BPR cuts.

## Cluster parameters (radius-capped K-means)

K_initial = ⌈N_FF / F_max⌉.  R_max = √(F_max / (π·ρ)), ρ = 0.68 FF/µm².

| F_max | R_max | K_initial | taps placed | actual max cluster |
|-------|-------|-----------|-------------|--------------------|
| 8  | 1.94 µm | 548 | 600 | 17 (169 clusters > 8) |
| 16 | 2.74 µm | 274 | 333 | — |
| 24 | 3.35 µm | 183 | 242 | — |
| 80 | 6.12 µm | 55  | 101 | — |

## Key conclusions

- **Backside sells on SKEW and INSERTION, not power.** It beats frontside on skew
  (10.8 vs 28.1 ps at F_max=8) and on insertion (298 vs 326 ps) thanks to thick,
  low-R BM1/BM2. It costs +9–21% power (structural — the mesh-driver tier, which the
  mesh's purpose forbids shrinking).
- **F_max is the whole dial.** Skew swings 9× (10.8→95.5 ps) across the sweep while
  power moves only 11%. Small clusters → low skew; power barely responds because the
  sink tier is ~27% of clock power (mostly fixed FF-pin drive).
- **Only F_max=8 and 16 beat frontside skew.** F_max≥24 are worse than frontside on
  skew AND cost more power → dominated, dropped.
- **Recommended operating points:**
  - **Flagship (lowest skew): F_max=8** — 10.8 ps for +20% power. Clean grid ceiling caveat.
  - **Balanced: F_max=16** — 17.2 ps for +14% power, no grid spill, all 4384 FFs present.

## Why backside wins: interconnect R/C (verified from the SPICE decks)

The skew advantage is a pure **resistance** story. Mesh R/C extracted from the decks,
cross-checked against the GT2N sheet-R and FasterCap mesh caps (pitch ~3.2 um):

| mesh layer | dominant R/segment | R per um | tech check (ohm/sq / width) |
|------------|--------------------|----------|-----------------------------|
| Backside BM1/BM2      | 24.3 ohm  | 7.48 ohm/um | 0.419 / 0.056 = 7.48 (match) |
| Frontside M6 (horiz)  | 84.7 ohm  | 26.6 ohm/um | 1.009 / 0.038 = 26.6 (match) |
| Frontside M5 (vert)   | 532.9 ohm | 167 ohm/um  | 3.506 / 0.021 = 167  (match) |

- Backside R is **3.6x lower than frontside M6 and 22x lower than frontside M5**.
  Frontside deck has 242 segments at 533 ohm and 270 at 96 ohm; backside tops out ~37 ohm.
- **Capacitance is comparable** (~1.2e-16 F/node backside field-solved vs ~1.0e-16 F/node
  frontside analytic) — NOT lower on backside.
- So the interconnect win is **entirely on R** (thick, wide, low-sheet-R BM1/BM2 vs thin
  M5/M6). That is the mechanism behind the lower skew/insertion; it is not an RC-both win.
  All R and C values match the tech, confirming the decks model the interconnect correctly.

## Deck verification (F_max=8, jpeg_bk_f8.sp) — audited

All correct: buffer model (x4, pin order A Y VDD 0), 1225 buffers (625 mesh + 600 sink),
626 sources (VDD + 625 driver PULSE w/ STA arrivals), 1225 TSVs @ 6 Ω, all wire/stub
categories present, mesh cap field-solve override applied, mesh R matches 0.419 Ω/sq,
FF pin caps 0.569 fF. Connectivity: mesh = 1 equipotential component (625/625 drivers);
all 4352 FFs driven, 0 orphans; leaf = proper distributed RC tree. HSpice clean.

Caveats: (1) F_max=8 cap not strictly enforced — 169 clusters exceed 8, max 17, due to
grid saturation (clustering limit, not a wiring bug). (2) 32 FFs dropped (4352/4384).

## Files (sweep_final/)

| file | what |
|------|------|
| f{8,16,24,80}.mt0 | HSpice measure output per F_max |
| f8.sp, f80.sp | canonical SPICE decks (f8 = audited) |
| f8.odb | OpenROAD DB for F_max=8 (only odb preserved; others overwritten) |
| frontside.mt0, frontside.sp | frontside 26×26 reference |
| f{8,16,24}.log | OpenROAD flow logs (CMS-0703 tap counts, DRT, DPL) |
| flow_backside.tcl | single-session backside flow (edit -capacity to set F_max) |
| flow_frontside.tcl | frontside mesh flow |

Regenerate a config: set `-capacity <F_max>` in flow_backside.tcl, run
`openroad -threads 72 -exit jpeg_backside_26x26.tcl`, then `hspice <deck>.sp -o <tag>`.
