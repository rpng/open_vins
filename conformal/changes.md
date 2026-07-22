# conformal — changes to the codebase & experiment runbook

This document explains everything that was added to this repository to implement the
project described in `conformal_explainer.pdf` ("Conformal Uncertainty for Visual–Inertial
State Estimation"), and gives an intern step-by-step instructions to carry out the planned
experiments.

> **Read the PDF first**, at least Part IV (the work plan). This file maps every planned
> task to a concrete, readably-named file and tells you what order to do things in.

---

## 1. What was added, and the one rule behind it

Everything new lives under a single new top-level directory: **`conformal/`**. Nothing under
`ov_msckf/`, `ov_core/`, `ov_init/`, `ov_eval/`, or `config/` was modified.

**Why the core is untouched (do not break this):** the paper's central defensibility claim
(PDF §4.4 and §18.3) is that OpenVINS runs *byte-for-byte unmodified*, so **any** change in
behaviour is attributable to the two things conformal touches — the process-noise **Q** and the
measurement-noise **R**. The moment you edit a filter source file, that argument is gone.
conformal therefore only ever:

* **reads** already-public OpenVINS getters (state, covariance, tracker diagnostics), and
* **overrides five numbers** at runtime — the 4 IMU noise densities inside **Q** (Net B) and
  the 1 pixel scalar inside **R** (Net A). See PDF §4.4, "Five numbers".

Where those five numbers live in the *existing* code (you will point the injection at these,
not edit them):

| Number | Symbol in the repo | File |
|---|---|---|
| 4 IMU noise densities (Q) | `ov_msckf::NoiseManager::{sigma_w, sigma_a, sigma_wb, sigma_ab}` | `ov_msckf/src/utils/NoiseManager.h` |
| 1 pixel scalar (R) | `ov_msckf::UpdaterOptions::sigma_pix` (builds `R` in `UpdaterMSCKF.cpp:211,282`) | `ov_msckf/src/update/UpdaterOptions.h` |
| χ² outlier gate (reads P/R; matters for A3) | `up_msckf_chi2_multipler` | `config/euroc_mav/estimator_config.yaml` |
| FEJ toggle (confound control) | `use_fej` | `config/euroc_mav/estimator_config.yaml` |

---

## 2. The complete file map

Each file has a header comment naming the exact PDF section it implements. Most are
**scaffolds**: real interfaces + references to OpenVINS symbols, with the science left as
clearly-marked `TODO(intern)`. Two are fully implemented and runnable today (marked ✅).

```
conformal/
├── README.md                         orientation; points here
├── theory/
│   └── verify_theory.py          ✅  §6  numerically verifies Thm 1, Cor 1.1, Thm 2, Thm 3
├── stage1_dumps/                     Stage 1 (C++): run OpenVINS once, dump HDF5
│   ├── run_asl_msckf.cpp             §18.2  ROS-free ASL/EuRoC runner (~150 lines)
│   ├── DiagnosticsLogger.hpp         §18.3  "the only real patch" — dumps diagnostics+residuals+GT
│   └── CMakeLists.txt                §18.1  standalone build that links the unmodified ov_* libs
├── stage2_train/                     Stage 2 (Python): train the two heads (minutes on a laptop)
│   ├── net_a_visual_deepsets.py      §8.1  Net A — per-feature σ_pix (DeepSets, target 26,305 params)
│   ├── net_b_imu_dilated_tcn.py      §8.2  Net B — 4 IMU log-scales (dilated TCN, target 102,468 params)
│   ├── heteroscedastic_nll.py    ✅  §8.3  the training loss L = ½(e/σ)² + ½·log σ²
│   ├── hdf5_dump_dataset.py          §8.3  loads Stage-1 HDF5; **sequence-disjoint** splits
│   ├── train_heads.py                §8.3  offline trainer for Net A / Net B
│   └── split_conformal_per_modality.py  §9  the conformal repair (one q_α per modality)
├── stage3_eval/                      Stage 3 (Python): inject σ back, evaluate
│   ├── corruptions_visual.py         §11.1  motion blur, brightness, noise+JPEG, occlusion
│   ├── corruptions_inertial.py       §11.1  vibration, clipping, bias jumps (+ co-degradation)
│   ├── metrics_nees_ate_rte.py       §7  NEES, coverage, ECE, sharpness (ATE/RTE via ov_eval)
│   ├── plug_learned_sigma.py         §17  builds the σ sidecar for each arm (stock/learned/conformal/oracle)
│   └── make_money_plot.py            §16  the headline figure (3 curves + Thm-2 vertical line)
├── experiments/                      the claims and ablations, one file each
│   ├── claim_c0_consistency_collapse.py       §13.1  C0 — THE PAPER (also the Week-2 spike)
│   ├── claim_c1_no_robustness_tax.py          §13.2  C1
│   ├── claim_c2_per_feature_vs_scalar_R.py     §13.3  C2 (DEFERRED to journal)
│   ├── claim_c3_conformal_repair_and_threshold.py §13.4  C3a repair + C3b threshold
│   ├── claim_c4_closed_loop_collisions.py      §13.5  C4 (DEFERRED to journal)
│   ├── ablation_a3_uniform_miscalibration.py   §14.1  A3 — the cleanest experiment
│   ├── ablation_a4_scalar_vs_per_feature_R.py  §14/§13.3  A4 (carries C2 in the conference paper)
│   ├── ablation_a5_appearance_vs_diagnostics.py §14.2  A5 — why transfer works
│   ├── ablation_a6_netb_std_pool.py            §14/§8.2  A6 — the std-pool claim
│   ├── ablation_a7_split_leakage.py            §14/§9.1  A7 — calibration-split leakage
│   └── the_fej_confound_control.py             §15  FEJ on/off + exact-Jacobian control arm
├── validation_gates/                 §19  three non-negotiable gates, IN THIS ORDER
│   ├── gate1_groundtruth_frame_check.py   §19.1  verify the T_BS ground-truth frame (KILL GATE)
│   ├── gate2_reproduce_euroc_ate.sh       §19.2  reproduce published EuRoC ATE (KILL GATE)
│   └── gate3_python_frontend_crosscheck.py §19.3  independent tracker cross-check (contingency)
└── configs/
    └── conformal_euroc.yaml             §4.4  human-readable map of the five numbers → OpenVINS keys
```

---

## 3. The mental model: three decoupled stages

The whole schedule is feasible because the pipeline is **decoupled** (PDF §17): no gradient
ever flows through the filter, so there is no C++/PyTorch autograd bridge. The three stages
talk through **files on disk**:

```
Stage 1 (C++, run ONCE)   ASL folders → unmodified OpenVINS → HDF5 {Δ, diagnostics, residuals, GT}
Stage 2 (Python, run MANY)  HDF5 → train Net A & Net B  (minutes on a laptop)
Stage 3 (Python/C++)        learned σ → back into OpenVINS Q,R → evaluate
```

Stage 1 is the expensive, one-time systems work. Once its HDF5 dumps exist, you can iterate
Stage 2 hundreds of times without recompiling anything.

---

## 4. Prerequisites

* **C++ (Stage 1):** OpenVINS built **ROS-free** (`cmake -DENABLE_ROS=OFF`, needs OpenCV +
  Eigen3 + Ceres), plus an HDF5 binding — **HighFive** (header-only over libhdf5) is
  recommended. A Docker image is a good fallback (PDF §18.1).
* **Python (Stages 2–3):** `numpy`, `scipy`, `torch`, `opencv-python`, `h5py`, `matplotlib`,
  `pandas`. (`pypdf` was installed only to read the PDF; it is not a project dependency.)
* **Data:** EuRoC MAV (anchor + training), then TUM-VI / UMA-VI (zero-shot), UZH-FPV
  (aggressive motion, journal), TartanAir / Flightmare (simulation, journal). See PDF §11.

---

## 5. Step-by-step: carrying out the experiments

Do these **in order**. The order is not cosmetic — it front-loads the only irreducible
uncertainty (does NEES actually break?) so you know by end of Week 2 whether there is a paper
(PDF §21.1).

### Step 0 — Verify the theory (do this today; needs only NumPy)
```bash
python conformal/theory/verify_theory.py
```
Every assertion should pass and the printed numbers should match PDF §6 (e.g. `a=(3,3)`→Γ=1.0000,
`a=(4,1)`→Γ=1.535, crossover κ=2→4.000). This is your independent check that the maths the
whole project rests on is right, before any systems work.

### Step 1 — Build the Stage-1 systems (Week 1)
1. Build OpenVINS ROS-free (see Prerequisites). This produces the `ov_*` libraries.
2. Implement the `TODO(intern)` blocks in `conformal/stage1_dumps/`:
   * `run_asl_msckf.cpp` — the three ASL loaders (IMU CSV, cam CSV, GT CSV). ASL timestamps
     are **nanoseconds** (÷1e9). Keep it ROS-free.
   * `DiagnosticsLogger.hpp` — pick the HDF5 binding and fill in the writers.
3. Build the runner:
   ```bash
   cd conformal/stage1_dumps && mkdir build && cd build
   cmake -DOPENVINS_ROOT=../../.. ..   # then point the link line at your ov_* build tree
   make -j
   ```

> ### ⚠️ The two implementation traps (PDF §20) — get these wrong and you train on a bug
> **Trap 1 — IMU ordering.** Feed *all* IMU samples with `t ≤ t_frame` **before** the camera
> frame at `t_frame`. Out-of-order feeding makes propagation silently wrong and the error
> masquerades as sensor noise — the networks would then learn to model a bug as physics. The
> merge loop in `run_asl_msckf.cpp` already enforces this; keep it.
> **Trap 2 — residual-dumping order.** Dump residuals **before** outlier rejection. The
> rejected features are exactly the ones carrying the degradation signal; gate them away before
> logging and you train only on measurements that already behaved. `DiagnosticsLogger.hpp`
> records `passed_chi2_gate` as a *flag* — never filter rows on it.

### Step 2 — Pass the validation gates (Week 1) — **KILL GATES**
Run these in order; **nothing downstream is trustworthy until they pass** (PDF §19, §22).

1. **Gate 1** — `conformal/validation_gates/gate1_groundtruth_frame_check.py`.
   Vicon reports the *marker body* pose, not the IMU frame; you must apply `T_BS` from the
   sensor YAML. Check: on a **static** segment, GT-derived specific force must equal raw IMU −
   gravity − bias. **This is the mistake that silently kills projects** — nothing fails loudly,
   training converges, and every number is wrong. If it fails, fix `T_BS` before doing anything else.
2. **Gate 2** — `conformal/validation_gates/gate2_reproduce_euroc_ate.sh`.
   Reproduce the published EuRoC ATE with the **default** config. This validates the entire
   Stage-1 chain against an external reference. The repo already has the harness
   (`benchmark/euroc_benchmark.sh`) and a committed reference run (`openvins_benchmark/summary.csv`);
   fill in the published ATE table and the comparison loop.
3. **Gate 3** — `conformal/validation_gates/gate3_python_frontend_crosscheck.py`.
   *Contingency only* under the ICRA timeline (PDF §19.3): run it **only if** track counts or
   residual scales look suspicious after Gates 1–2.

> **Kill gate (PDF §22):** if Gates 1–2 are not green by end of Week 1, ICRA is off — revert to
> the IROS timeline. Gate 1 has an unbounded debugging tail; decide fast.

### Step 3 — Produce the Stage-1 dumps
Run the built runner over every EuRoC sequence:
```bash
./run_asl_msckf ../../config/euroc_mav/estimator_config.yaml /data/EuRoC/MH_01_easy MH_01_easy.h5
# ... repeat for all 11 sequences
```
You now have one HDF5 per sequence containing IMU windows, per-feature diagnostics, residuals,
state/covariance, and GT.

### Step 4 — Train the two heads (Stage 2, iterate freely)
1. Implement the HDF5 reads in `hdf5_dump_dataset.py`. **Use sequence-disjoint splits** — calibration
   and test must never share a trajectory, or coverage leaks (PDF §9.1). Frame-level splitting is
   the *wrong* way and is exactly what ablation A7 measures.
2. Confirm architecture parameter counts (guards against silent drift):
   ```bash
   python conformal/stage2_train/net_a_visual_deepsets.py   # target 26,305 params
   python conformal/stage2_train/net_b_imu_dilated_tcn.py    # target 102,468 params
   python conformal/stage2_train/heteroscedastic_nll.py      # ✅ self-test: recovers σ ≈ sqrt(E[e²])
   ```
   Reconcile `PER_FEATURE_DIM` / `FRAME_CTX_DIM` (Net A) with the exact columns you dumped so the
   counts land on target.
3. Train:
   ```bash
   python conformal/stage2_train/train_heads.py --head A --h5-dir /data/dumps --out netA.pt
   python conformal/stage2_train/train_heads.py --head B --h5-dir /data/dumps --out netB.pt
   ```

### Step 5 — Calibrate the conformal repair (PDF §9)
`conformal/stage2_train/split_conformal_per_modality.py` fits **one q_α per modality** (visual,
inertial) on the sequence-disjoint **calibration** pool. Its `__main__` self-test demonstrates a
3×-overconfident model being repaired to nominal coverage. Report **block-bootstrap CIs** on every
coverage number (PDF §13.4) — with only a handful of calibration sequences the quantile is itself
uncertain.

### Step 6 — Inject σ back and evaluate (Stage 3)
`conformal/stage3_eval/plug_learned_sigma.py` produces a **σ sidecar** (per-frame IMU σ, per-feature
pixel σ) for each **arm**: `stock`, `learned`, `conformalised`, `oracle`. A thin Stage-3 variant
of the runner reads this sidecar and sets the five numbers before propagate/update — still not
touching the filter's algorithms. Metrics come from `metrics_nees_ate_rte.py` (NEES is the one the
paper lives on; ATE/RTE via the repo's `ov_eval`).

### Step 7 — Claim C0 (Week 2) — the de-risk spike, **and a kill gate**
`conformal/experiments/claim_c0_consistency_collapse.py`. Run the three arms
(stock / learned / oracle) across corruption severities 0–5 on held-out sequences, crossed with
FEJ on/off, over multiple seeds. **Success (PDF §13.1):** a severity range exists where the ATE
ratio learned:stock is < ~1.2 while the NEES ratio is > ~5 — accuracy looks fine while consistency
collapses. `check_success()` encodes this. **If it fails, there is no paper — pivot immediately**
(PDF §22). This is deliberately quick-and-dirty: you want a yes/no, not a polished result.

### Step 8 — The ablations
* **A3 first** (`ablation_a3_uniform_miscalibration.py`, Week 3) — the cleanest experiment; needs
  **no networks**, just multiply Q and R by λ∈{0.1,…,4}. Prediction (Cor 1.1): **ATE exactly
  unchanged, NEES ∝ 1/λ**. ⚠️ **A3 trap (PDF §14.1):** the χ² Mahalanobis gate reads P and is *not*
  scale-invariant — either scale `up_msckf_chi2_multipler` by λ too, or disable the gate, and
  **state which in the paper**. `check_success()` checks ATE-invariance + a log-log slope of −1.
* **A4** — scalar vs per-feature R on the corruption suite (carries C2 in the conference paper).
* **A5** (Week 5) — appearance vs tracker-diagnostic head; the transfer ranking should *flip*
  between in-distribution (EuRoC) and zero-shot (TUM-VI). This is what proves transfer works.
* **A6** — Net B with/without the std-pool (the `use_std_pool` flag already exists on `NetB`),
  evaluated under the high-frequency inertial corruptions.
* **A7** — frame-level vs sequence-disjoint calibration split; quantifies coverage leakage.

### Step 9 — The FEJ confound control (PDF §15) — put this in the annual review
`conformal/experiments/the_fej_confound_control.py`. Every corruption sweep runs FEJ **on** and
**off**, plus an exact-analytic-Jacobian **simulation** arm. If the NEES explosion persists with
FEJ on *and* in the exact-Jacobian sim, it cannot be linearisation — it must be the learned
covariance. This is the strongest reviewer objection; A3 (observability-independent) backs it up.

### Step 10 — Claims C1, C3, and the money plot (Week 4)
* **C1** (`claim_c1_no_robustness_tax.py`) — on **clean** EuRoC, conformalised ≈ stock (ATE within
  ±10%, NEES no worse). Pre-empts "you made it honest by making it worse."
* **C3** (`claim_c3_conformal_repair_and_threshold.py`) — **C3a:** conformalised Q,R restore
  consistency zero-shot (NEES back to a band around n; coverage near nominal). **C3b (the
  spotlight):** the measured overconfidence â and κ predict the crossover severity via Thm 2, and
  it matches the empirical break point. If C3b fails but C3a holds, demote theory to motivation and
  lead with A3 (decide early — PDF §22).
* **Money plot** (`make_money_plot.py`) — three NEES/n curves vs severity (naive explodes,
  conformalised graceful, stock flat-but-blunt) with the Thm-2 crossover as a vertical line. If the
  line lands where the naive curve turns up, that is the whole paper in one figure (PDF §16).

---

## 6. Schedule, kill gates, and what's deferred

**Target: ICRA 2027, deadline 15 Sept 2026** (PDF §21). Week-by-week:

| Week | Block | Files |
|---|---|---|
| 1 | Stage 1 + Gates 1–2 | `stage1_dumps/*`, `validation_gates/gate1*`, `gate2*` |
| 2 | De-risk spike: does NEES break? | `claim_c0_*` |
| 3 | Conformal wrapper + A3 | `split_conformal_*`, `ablation_a3_*` |
| 4 | C1 + money plot v1 | `claim_c1_*`, `claim_c3_*`, `make_money_plot.py` |
| 5 | TUM-VI + A5 | `ablation_a5_*`, zero-shot eval |
| 6 | Writing sprint | (paper.tex) |
| 7 | Review + repair | — |
| 8 | Buffer | — |

**Kill gates (decide now, PDF §22):** Gates 1–2 not green by end of Week 1 → revert to IROS;
NEES doesn't visibly break by end of Week 2 → pivot; TUM-VI not producing clean dumps by Aug 21 →
cut it (the corruption suite alone supports C0/C1/C3/A3/A4/A5); Thm-2 overlay doesn't match at
Week 4 → demote theory to motivation.

**Deferred to the journal version** (stubs exist so you know where they live, but do **not** spend
conference-timeline effort on them): **C2** (`claim_c2_*`, UZH-FPV per-feature R — A4 substitutes)
and **C4** (`claim_c4_*`, Flightmare closed-loop collisions).

---

## 7. Quick reference — file → PDF section → what to implement

| File | PDF | Status | Your job |
|---|---|---|---|
| `theory/verify_theory.py` | §6 | ✅ runnable | run it; keep numbers in sync with §6 |
| `stage1_dumps/run_asl_msckf.cpp` | §18.2 | scaffold | 3 ASL loaders; honour Trap 1 |
| `stage1_dumps/DiagnosticsLogger.hpp` | §18.3 | scaffold | HDF5 writers; honour Trap 2 |
| `stage1_dumps/CMakeLists.txt` | §18.1 | scaffold | point link line at your ov_* build |
| `stage2_train/net_a_visual_deepsets.py` | §8.1 | near-complete | reconcile dims → 26,305 params |
| `stage2_train/net_b_imu_dilated_tcn.py` | §8.2 | near-complete | verify → 102,468 params |
| `stage2_train/heteroscedastic_nll.py` | §8.3 | ✅ runnable | — |
| `stage2_train/hdf5_dump_dataset.py` | §8.3 | scaffold | HDF5 reads; sequence-disjoint splits |
| `stage2_train/train_heads.py` | §8.3 | scaffold | dataloaders + train loop |
| `stage2_train/split_conformal_per_modality.py` | §9 | mostly done | block-bootstrap CI; adaptive q_α |
| `stage3_eval/corruptions_visual.py` | §11.1 | near-complete | calibrate severities on EuRoC |
| `stage3_eval/corruptions_inertial.py` | §11.1 | near-complete | confirm IMU units/order |
| `stage3_eval/metrics_nees_ate_rte.py` | §7 | mostly done | wire ATE/RTE to ov_eval |
| `stage3_eval/plug_learned_sigma.py` | §17 | scaffold | net forward + sidecar writer |
| `stage3_eval/make_money_plot.py` | §16 | near-complete | feed it aggregated results |
| `experiments/claim_c0_*` | §13.1 | scaffold | arm runner + sweep |
| `experiments/claim_c1_*` | §13.2 | scaffold | clean-EuRoC sweep |
| `experiments/claim_c3_*` | §13.4 | scaffold | repair + threshold comparison |
| `experiments/claim_c2_*`, `claim_c4_*` | §13.3, §13.5 | deferred | journal only |
| `experiments/ablation_a3_*` | §14.1 | mostly done | config scaling; honour A3 trap |
| `experiments/ablation_a4_*…a7_*` | §14 | scaffold | per-ablation sweep |
| `experiments/the_fej_confound_control.py` | §15 | scaffold | FEJ toggle + exact-Jac sim arm |
| `validation_gates/gate1_*` | §19.1 | scaffold | T_BS static-segment check |
| `validation_gates/gate2_*` | §19.2 | scaffold | fill published ATE + compare loop |
| `validation_gates/gate3_*` | §19.3 | near-complete | contingency cross-check |
| `configs/conformal_euroc.yaml` | §4.4 | reference | — |

---

*Companion PDF: `../conformal_explainer.pdf`. When in doubt, the file's header comment cites the
exact section to re-read.*
