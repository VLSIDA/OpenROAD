# Backside Clock-Mesh Synthesis — Algorithm Reference

Reference for writing up the GT2N backside clock-mesh flow. Each section is a
self-contained algorithm: purpose, inputs, steps, and the design rationale
(the *why*, which is the part not obvious from the code).

## 0. Problem & topology

GT2N is a 2nm backside-power (BSPDN) node. The clock **tree** stays on the
frontside (M1–M5); the clock **mesh** lives on the backside (BM1/BM2, fine
0.056 µm wires). The clock crosses front↔back through a passive bridge cell
`gt2_6t_TSV` (pin A on frontside M1, pin Y on backside BM1; Y=A passive,
internally M1→M0→nTSV(VSD cut)→BPR→BV0→BM1).

```
clk → CTS tree → drive-buffer → drive-TSV(A→Y) → MESH(BM1/BM2)
    → sink-TSV(Y→A) → sink-buffer → FF clock pins
```

Layer routing levels: BM2=4, BM1=5, BPR=6, M0=7, M1=8 … M5=13.
- **BPR** = backside power followpin rails (one per row boundary, alternating
  vdd/vss, *shared* between adjacent rows). SPACING = 0.112 µm.
- **BM3/BM4** = coarse backside power straps (vertical/horizontal).

**Hard constraint (advisor):** BPR SPACING 0.112 µm means a signal pad cannot
fit between two intact rails → a TSV crossing requires a **rail break**.

## 1. Frozen grid — `computeFrozenGrid()`

**Purpose.** One deterministic deformed+pruned mesh grid, computed identically
at *floorplan* (reserve phase, before PDN) and *post-CTS* (connect phase). A TSV
reserved early must land on exactly the vertical wire the mesh draws later.

**Inputs.** core bbox; h/v layer track grids; pitch; PDN vertical-strap params
(strap_pitch, strap_offset = core.xMin→first strap center, strap_width).

**Steps.**
1. Core-based bbox; align grid start to the layer track grid.
2. **In-core clamp:** bump aligned start by one track pitch until ≥ core.min +
   margin; clamp the far edge inside core. *(Fixes the via-stack climb: nodes
   landing in the BPR-free margin outside core let the router build full
   front↔back via stacks.)*
3. **Collect PDN vertical straps:** scan the real PDN for BM3 vertical straps
   (preferred — catches the vdd/vss strap *pair* per pitch); fall back to the
   analytic band `core.xMin + offset + k·pitch` when no PDN exists yet.
4. **Shift V wires clear** of straps (`shiftVClearOfPdn`); **notch H wires**
   where they cross a strap (`notchHByPdn`); **prune orphan H segments** with no
   surviving V crossing.
5. Intersections = V×H crossings; **union-find** into connected fragments.

**Output.** `FrozenGrid{v_wires, h_wires, intersections, fragments, aligned_pitch}`.
**Why deterministic:** pure computation (reads core + tracks, no DB writes);
reserve and connect call it with identical inputs → identical grids.

## 2. Reserve TSV sites — `reserveMeshTsvSites()` (floorplan, pre-tapcell/PDN)

**Purpose.** Place drive-TSV cells + keepouts at the frozen-grid intersections
before tapcell/PDN, so the PDN builds *around* them.

**Steps.** For the frozen grid: place a TSV (FIRM) at intersections giving ≥1
driver per fragment plus a target spacing; offset each ~1 row off the V×H
crossing with **Y on the vertical line**. Add a layer-selective keepout that
blocks placement + power (**BPR only** — *not* BM3/BM4, which would collapse the
upper PDN) but leaves M1/BM1/BM2 open so pins stay routable.

**Why BPR-only keepout:** obstructing BM3/BM4 makes pdngen drop all the upper
straps; obstructing BPR alone makes pdngen break just the followpins at the
keepout (64/64 broken, straps intact).

## 3. Drive proxy BTerms — `createProxyBTermsWithSeparateNets()` / `setupProxyBTerms()`

**Purpose.** Tie the drive-buffer output up through the TSV into the mesh.

**The connection principle (KEY).** The mesh is a **special wire** — the signal
router never routes it, and a TSV.Y pin mid-stripe doesn't abut it. So to tie a
TSV.Y to the mesh: put Y on its **own 2-pin net** with a **proxy BTerm** whose
BPin sits on the mesh layer **overlapping the stripe**. The router routes the
2-pin Y→BTerm net (a normal signal net it *will* route); the BPin's overlap with
the stripe is the physical tie. Net names differ (`b_clk_buf_*` vs `clk_mesh`)
but the metal overlaps = electrically connected on silicon.

**Steps.** Per intersection with a buffer + TSV:
- FRONTSIDE net `clk_buf_x_y` = buffer output + TSV.A (router routes frontside).
- BACKSIDE net `b_clk_buf_x_y` = TSV.Y.
- Proxy BTerm on the backside net; BPin box on `buf_bterm_layer` (the
  lower-routing-level of h/v) at the intersection, ±`width/2`, overlapping the
  mesh stripe.

## 4. Sink taps — `createSinkTaps()` (sink side; mirror of §3)

**Purpose.** Bring the mesh back up to local sink-buffers that drive the FFs.

**Capacity C.** From liberty: buf_x4 out `max_capacitance` 1.147 pF / DFF CLK
`capacitance` 0.0008265 pF ≈ **1387** — the electrical max, far too loose for a
clock (no load balancing, awful slew). So **C = min(lib≈1387, 16)** = a clock
fanout target. With taps at every H-wire gap the real fanout is ~1–4, so C
rarely binds; it's a safety cap, and the dense taps do the load distribution.

**Steps.**
1. **Geometry** from the mesh net's special wires: V-wire x-centers; H-segments
   (y, xlo, xhi). *(Mesh net = `<base>_mesh`, e.g. `clk_mesh`, NOT the base
   clock name.)*
2. **Candidate taps:** per H-segment, one per gap between adjacent in-segment V
   wires — midpoint x `mx`, nearest placement row `ry` to the H-wire y, keep
   `hy`.
3. **Sinks:** every FF clock input pin (mterm sigType CLOCK, or name CLK/CK).
4. **Assignment (assign-FIRST):** each FF → nearest candidate tap (Manhattan)
   with load < C; spill to next-nearest when full. Record per-tap FF list.
5. **Place USED taps only** (≥1 assigned FF — no empty buffers in FF-free
   regions): sink-TSV (FIRM, x site-snapped so Y lands ~on `mx`) + sink-buffer
   (movable, placed *outside* the TSV keepout → `detailed_placement` legalizes
   it onto a powered row). Connect:
   - TSV.Y → `b_sink_i` net + proxy BTerm (BPin on the mesh H-wire at `(mx,hy)`)
     — the §3 connection principle, mirrored.
   - TSV.A → `sink_tap_i` → sink-buffer input.
   - sink-buffer output → `sink_drv_i` → assigned FFs (disconnected from `clk`).
6. Keepout + BPR break left to §5 (`break_bpr_at_tsvs`), which sweeps all TSVs.

**Why place-used + buffer-outside-keepout:** no clock load wasted on empty
regions; the buffer ends up on intact BPR (powered), keepout/break only at the
TSV footprint.

## 5. BPR surgery — `breakBprAtTsvs()`

**Purpose.** Break the BPR power rails at every TSV (the §0 hard constraint),
relocate stranded cells, keep the PDN connected.

**Steps.**
1. **Per TSV:** cut window = footprint + halo; placement blockage spanning
   `relocate_rows` above+below. *(The two bounding rails are **shared** with the
   adjacent rows, so breaking them strands cells there too → relocate the
   3-row band.)*
2. **Trim:** for each POWER/GROUND BPR rail SBox that overlaps cut windows in y,
   surviving x-spans = rail minus the merged cut intervals; recreate the
   surviving SBoxes, destroy the original. *(Per-TSV break, not continuous —
   rails stay intact between TSVs so non-TSV columns keep power.)*
3. **Floating-stub cleanup:** a BPR SBox with no via overlapping it is
   electrically floating (power reaches BPR only via the strap column; the
   alternating vdd/vss straps can leave an edge piece of one net unfed) →
   blockage + delete.
4. **Tap deletion:** delete tap cells overlapping any blockage (tapcell's
   `findBlockages` only sees block macros, not dbBlockages, so taps land in
   keepouts and must be cleaned post-hoc).
5. Caller runs `detailed_placement` to relocate everything.

**IR result (gcd):** the per-TSV break is IR-safe — mesh IR ≈ baseline ~0.01 %,
PSM-0040 (fully connected). Verified on BSPDN with the corrected tap LEF + BM4
vsrc + BM3/BM4 setRC.

## 6. Routing wrapper (stays TCL)

Temporary `dbObstruction` on **BPR over the core** (the clock crosses front↔back
*inside* the TSV cells, so the signal router never needs BPR — without this it
routes clock on BPR and shorts the rails), then `set_routing_layers -signal
M2-M5 -clock BM2-M5`, global + detailed route, then **remove the obstruction**
so the saved odb has BPR back to normal.

## Command surface (consolidated from TCL)

```
create_clock_mesh   -clock -h_layer -v_layer -pitch -buffers -cts_buffers   (§1 connect + mesh)
setup_proxy_bterms  -clock -proxy_layer                                     (§3)
create_sink_taps    -h_layer -v_layer -buffer [-capacity 16] [-tsv_master] [-halo]   (§4)
break_bpr_at_tsvs   [-bpr_layer BPR] [-tsv_master] [-tap_master] [-halo] [-relocate_rows]  (§5)
# + routing wrapper (§6) in TCL
```
Reserve phase (floorplan): `reserve_clock_mesh` (§1 reserve + §2).

**Verified (gcd):** 64 drive-TSVs + 22 sink-taps (35 FFs, max fanout 4), 86 cut
windows, 16 rails trimmed, ~102 floating stubs cleaned, 67 taps deleted, routed.
