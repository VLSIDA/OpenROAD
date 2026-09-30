# mc_var — per-instance Monte Carlo for the Ibex clock-network triangle

10,000-sample HSPICE variability study of BS+LCB / FS+LCB / FS-Direct on the
Ibex placement (5 µm pitch, 9×9 grid, 1,938 FFs), TT / 0.7 V / 25 °C.

## Why per-sample Python netlists (not HSPICE `sweep monte=`)

* Per-**segment** wire IID needs ~33k independent draws per deck; HSPICE only
  accepts distributions as an entire `.param` RHS, i.e. 33k dedicated monte
  params per iteration — untested at this scale on hspice 2017.03 and with
  multi-GB `.mc0` bookkeeping.
* Per-**transistor** draws: subckt-local `gauss` params draw per cell
  instance (the old `wfac/lfac` were per *buffer*), and `modmonte` cannot
  express the nanosheet-correlation flag or finger-level IID on M=4/M=12
  devices.
* Checkpointing, recorded seed, per-sample Vdd in the CSV, fail-loud per-FF
  validation, and a future R–C correlation coefficient are all trivial when
  numpy owns the RNG.

Measured cost: ~8 s/solo sample, ~2,250 samples/h at `-j 16` on this host.

## Device knobs (empirical, `check_knobs.py` — run it before any campaign)

| parameter | mechanism | note |
|---|---|---|
| ΔVth | instance `delvtrand=` (nominal + draw) | OVERRIDE semantics; instance `delvto` is silently dead on this card |
| L_g | instance `L=` | live |
| T_NS | instance `tfin=` | live |
| W_NS | fractional `M = 1 + 2·δW/(2·W+T)` | instance `hfin` is ignored by hspice; M uses the model's own Weff sensitivity (exact for drive/cap scaling) |

M=k fingers are expanded to k independent devices (verified electrically
neutral at nominal), so "per transistor" means per physical finger.

## Sampling semantics

* **IID/transistor**: ΔVth (p 20 mV, n 15 mV), L_g (14 nm ± 0.167), W_NS
  (31 nm ± 0.167), T_NS (5 nm ± 0.133). `--sheets shared` (default): the
  stacked sheets of one device share the geometry draw. `--sheets
  independent`: emulates fully uncorrelated sheets by σ/√NFIN on W/T
  (first-order device-level equivalent; the card lumps sheets, NFIN=3 —
  note the paper says 4 sheets).
* **IID/segment**: wire R (front σ configurable, default 12.5%; back =
  front/2; TSVs use the backside σ), wire C 3% (FF pin caps `CsinkN` get
  `sigma_c_sink`, default = wire C σ). R and C independent;
  `draw_wire_multipliers()` in sampler.py is the seam for a future rho_RC.
* **Global/sample**: Vdd = N(0.7 V, 18 mV), recorded per row in the CSV.
* Vth draws are zero-mean around the *extracted* nominals (delvtrand
  +0.0944 n / −0.0170 p on top of the card's Vt ≈162/192 mV); the spec's
  "μ = 177 mV" is treated as the nominal's identity, not an absolute reset.
* Sample 0 of every run is the deterministic nominal (excluded from stats).
* Reproducibility: sample i uses `SeedSequence(entropy=seed, spawn_key=(i,))`
  with a fixed draw order — resume-safe and scheduler-independent. Using the
  same seed for all three networks makes the global-Vdd sequence identical
  across them (common random numbers for paired comparison).

## Fail-loud

A sample aborts the whole run (scratch kept for post-mortem) if hspice
fails, any of the 1,938 `t_sink_N` is missing from the .mt0, or any
t_sink/slew_sink measures `failed` (FF didn't toggle).

## Commands

```bash
cd mc_var
python3 check_knobs.py                      # must print OK

# validation (done, results in validate_bslcb_100/)
python3 run_mcvar.py --network bslcb -n 100 --seed 20260901 -j 16 -o validate_bslcb_100

# baselines: 10k samples each (~4.5 h/network at -j 16; try -j 32,
# throughput is license-bound more than CPU-bound on this 128-core host)
python3 run_mcvar.py --network bslcb    -n 10000 --seed 20260901 -j 32 -o bslcb_10k
python3 run_mcvar.py --network fslcb    -n 10000 --seed 20260901 -j 32 -o fslcb_10k
python3 run_mcvar.py --network fsdirect -n 10000 --seed 20260901 -j 32 -o fsdirect_10k

# wire-R sweep (BS+LCB, 1k samples/point, backside = half of frontside)
for s in 0.05 0.10 0.15 0.20; do
  python3 run_mcvar.py --network bslcb -n 1000 --seed 20260901 -j 32 \
      --sigma-r-front $s -o sweep_r$s
done

# reports
python3 mc_stats.py summary bslcb_10k fslcb_10k fsdirect_10k
python3 mc_stats.py hist    bslcb_10k fslcb_10k fsdirect_10k
python3 mc_stats.py sweep   sweep_r0.05 sweep_r0.10 sweep_r0.15 sweep_r0.20
```

Interrupted runs resume by re-running the same command (samples.csv is the
checkpoint; run_meta.json pins seed+config and refuses a mismatched outdir).

## Validation status (2026-09-01)

* knob check OK; finger expansion neutral (<0.1 ps).
* Nominal BS+LCB skew 6.81 ps — matches the prior 105k-sample HSPICE-monte
  study mean (6.812 ps) on the same decks.
* 100-sample BS+LCB (seed 20260901, σ_R 12.5/6.25%): skew μ 7.154 /
  σ 0.365 / μ+3σ 8.250 / max 8.190 ps; slew μ+3σ 15.26 ps; latency μ+3σ
  55.64 ps; Vdd draw σ 18.7 mV ≈ spec.
* FS+LCB nominal skew 15.28 ps, FS-Direct 29.78 ps, all 1,938 sinks toggle.

Caveats: base decks are `mc_sigma10/{bslcb,fslcb,fsdirect}_s10.sp` (their
old monte machinery is stripped at template build); specifying instance
`tfin` shifts absolute delay ~1% vs the model-default path — uniform across
all samples/networks, so comparisons and spreads are unaffected.
