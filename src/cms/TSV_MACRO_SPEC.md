# Backside Clock-Mesh TSV — Floorplan Macro-Placement Algorithm (SPEC)

Status: DESIGN — not yet implemented. Revert point committed 2026-06-24
(OpenROAD `bspdn-cms`, ORFS `bspdn-gt2n`).

## Goal
Stop doing TSV insertion + BPR breaking *after* the PDN exists (place→nudge→
`break_bpr_at_tsv.tcl` surgery). Instead, reserve TSV sites + layer-selective
keepouts **at floorplan, before the PDN is built**, so:
- the PDN routes around them and **breaks the 2 BPR rails automatically**,
- tapcells skip them automatically,
- placement leaves them clean,
- the router still reaches pins A (M1) and Y (BM1) and crosses over on upper layers.

## The cornerstone: ONE frozen grid
A single deterministic function computes the FINAL deformed/pruned mesh grid and
is reused by BOTH phases. It must produce the "multiple small grids" (fragments)
exactly, so the TSV placed early lands on the same vertical wire the mesh draws
late.

### `computeFrozenGrid(pitch, h_layer, v_layer, core, pdnParams, tsvDims, spacing)`
1. Ideal lines: H-line y's and V-line x's at `pitch`, track-aligned.
2. **In-core clamp** (existing CMS-122): bump start in by tracks, clamp far edge,
   so no node/keepout lands in the BPR-free margin.
3. **Strap positions ANALYTICALLY** from pdn.tcl params (NOT from a built PDN):
   BM3 vertical x's = `offset + k*pitch` (pitch 2.16, offset 1.08, width 0.36);
   BM4 horizontal y's likewise; BPR followpins at every row boundary.
   The via-column pads on BM1/BM2 sit at the BM3 strap x's.
4. **Deform** (same rules as today, fed by #3 instead of scanned sboxes):
   - shift each V wire clear of the BM3 strap bands (keep 0.056); DROP if it can't.
   - notch each H wire at BM4 straps.
   - prune orphan H segments (no surviving V crossing) -> fragments.
5. Output: { surviving V wires, H segments, intersections(x,y),
   fragments[] (connected components of the mesh) }. Deterministic.

## Phase 1 — RESERVE (floorplan, PRE_TAPCELL hook, before PDN)
Input: frozen grid.
1. **Driver coverage (DESIGN MUST #1):** for EVERY fragment, choose >=1 hosting
   intersection so no mesh fragment is left without a TSV (else floating clock
   island). Density policy = DECISION below.
2. For each chosen intersection:
   a. **Offset** off the V×H crossing by ~1 row along the V wire (X stays on the
      vertical line so Y still taps the mesh); pick a neighbor row clear of locked
      taps/macros.
   b. **Place the TSV cell** (FIXED) with Y-pad center on the V-line x, on the row.
   c. **Carve the layer-selective keepout** (~0.31 x 0.176 um, centered on TSV):
      - placement blockage (no std cells),
      - obstruction on **BPR ONLY** (breaks the followpin rails for the crossing),
      - LEAVE M1 / BM1 / BM2 open (pins A, Y, and mesh stay routable),
      - LEAVE BM3 / BM4 open too. *** VERIFIED: obstructing BM3/BM4 collapses the
        entire upper PDN (pdngen drops all BM3-V/BM4-H straps + via columns ->
        0 vertical straps). The coarse straps live above M1/BM1, don't conflict
        with the TSV, and their via columns land at strap crossings off the
        keepouts (where BPR is intact). BPR-only keepout: BM3-V/BM4-H straps
        survive AND BPR breaks 64/64. ***
3. The TSV cells now live in the odb (their positions persist). The grid itself
   is recomputed in Phase 2 (same inputs -> identical), not serialized.

## Between phases (normal flow)
- TAPCELL: taps skip keepouts (no tap where we break the rail).
- PDN: followpins/straps stop at keepout edges -> both BPR rails break, no surgery.
- **ASSERT (DESIGN MUST #2):** after PDN, compare analytic BM3/BM4 strap x/y to
  the actual generated PDN sboxes; ERROR if they differ (catches grid drift).

## Phase 2 — CONNECT (post-CTS, where CMS runs today)
Input: frozen grid (recomputed identically), pre-placed TSVs.
1. Recompute frozen grid (identical to Phase 1).
2. Draw mesh wires: V on BM1, H on BM2, per the surviving segments.
3. For each pre-placed TSV (found by master `gt2_6t_TSV`):
   - place its driving buffer ADJACENT (DESIGN MUST #4), wire buffer -> TSV.A (M1),
   - wire TSV.Y (BM1) -> mesh (tap on the V wire; with Y on the line it abuts).
4. (Reuse existing connect logic, adapted to pre-placed TSVs; do NOT re-place TSVs.)

## DECISIONS NEEDED
- D1 **TSV density:** one per intersection (max drivers, more area/PDN holes) vs.
  one per fragment (min) vs. a target spacing (e.g., every Nth node). Affects
  skew vs. IR/area.
- D2 **Offset policy:** always +1 row, or pick whichever neighbor row is clear.
- D3 **Keepout type:** placement-blockage + PDN-blockage (lighter; recommended)
  vs. true macro (auto-everything but heavier, 64 macros).
- D4 **Grid handoff:** recompute deterministically (recommended) vs. serialize to
  a sidecar/odb property.
- D5 **Buffer adjacency:** region hint vs. explicit placement next to each TSV.

## Files to change (once spec is agreed)
1. `src/cms/src/ClockMesh.cc` (+ `.hh`): `computeFrozenGrid()` shared fn;
   `reserveMeshTsvSites()` (Phase 1); adapt connect path to pre-placed TSVs.
2. `src/cms/tcl/cms.tcl` (+ `Cms.i`): new `reserve_clock_mesh` command.
3. ORFS `platforms/gt2n/tsv_reserve.tcl` (PRE_TAPCELL hook) + `config.mk`
   (`export PRE_TAPCELL_TCL=...`).
4. `src/cms/test/gt2n/`: update drivers to 2-phase; retire `break_bpr_at_tsv.tcl`.
