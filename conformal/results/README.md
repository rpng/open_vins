# Experiment results downloaded from the remote server

The consolidated comparison against published ORB-SLAM3, VINS-Fusion, and
OKVIS2 EuRoC results is in
[`comparisons/EUROC_POPULAR_METHODS_COMPARISON.md`](comparisons/EUROC_POPULAR_METHODS_COMPARISON.md),
with machine-readable values in
[`comparisons/euroc_ate_comparison.csv`](comparisons/euroc_ate_comparison.csv).

## Conformal Stage 1

The complete validated EuRoC Stage-1 suite is in [`stage1/`](stage1/).

- [`stage1_suite_summary.csv`](stage1/stage1_suite_summary.csv) is the consolidated benchmark table.
- [`stage1_suite_metadata.json`](stage1/stage1_suite_metadata.json) records the image and aggregate counts.
- [`stage1_suite_checksums.sha256`](stage1/stage1_suite_checksums.sha256) covers every downloaded artifact.
- `*_stage1.h5` are the eleven canonical training dumps.
- `*.run.log`, `*.validation.txt`, and `*.runtime_seconds.txt` are the per-sequence audit files.

All eleven canonical dumps pass `validate_dump.py`. Together they contain 23,302 frames and
74,329 pre-gate feature candidates.

Two diagnostic artifacts are intentionally retained but must not be used for training:

- `MH_01_easy_stage1.unaligned.h5` is the superseded run produced before estimator-to-GT world
  alignment was added.
- `MH_04_difficult_stage1.failed-stereo-count.run.log` records the initial failure that led to
  timestamp-based stereo synchronization.

MH05 is structurally valid but scientifically atypical: it has 700 feature candidates, an
11.7% gate pass rate, and much larger residuals. Keep it in the held-out test split as currently
planned; do not mix it into training without explicitly studying that distribution shift.

## Stock OpenVINS ATE/RPE benchmark

The previously downloaded stock benchmark remains in [`../../openvins_benchmark/`](../../openvins_benchmark/).
Its [`summary.csv`](../../openvins_benchmark/summary.csv) contains APE/RPE results for all eleven
EuRoC sequences and is the input to conformal Gate 2.

## Conformal Stage 2

The downloaded Stage-2 artifacts are in [`stage2/`](stage2/). All eleven
sequence arrays pass `reports/stage2_data_validation.json`, and
`stage2_all_checksums.sha256` covers the full remote result set.

Accepted:

- `netA_geomvalid_seed7_epoch50_lr3e-4.pt`: geometry-valid Net A. Training NLL
  1.04437; frozen calibration NLL 1.14010.
- `netB_guarded_seed7_maxepoch50_lr3e-4.pt`: accepted only as an epoch-zero
  constant four-channel Q correction. Validation selected epoch 0; frozen
  calibration NLL 3.26105 equals the fit-only constant baseline. This is not
  evidence for a conditional TCN.

Rejected but retained for audit:

- `netA_seed7_epoch50.pt`: trained before the image-diagonal GT-projection
  validity gate; calibration NLL was approximately 1.11e10.
- `netB_seed7_epoch50.pt`: unstable optimization.
- `netB_stable_seed7_epoch50_lr3e-4.pt`: sequence overfit; calibration NLL 233.
- `netB_selected_seed7_maxepoch50_lr3e-4.pt`: transferring only the selected
  epoch count into an all-sequence refit failed; calibration NLL 14.21.

The guarded Net-B result is a scientifically useful negative result: the
current Stage-1 inputs and derived targets do not support the claim that a
window-conditioned TCN generalizes across EuRoC sequences. Stage 3 must label
the inertial arm as a constant-Q correction unless a new target/input design is
predeclared and retrained without using calibration or test data for tuning.

The fixed 90% conformal calibration is stored in
`stage2/conformal/conformal_alpha0.10.json`. It used only MH04 and V1_03:

- Visual: `q_alpha=1.170186`, pooled coverage 83.67% before and 90.02% after;
  block-bootstrap 95% interval `[1.105554, 1.239139]`.
- Inertial: `q_alpha=1.237830`, pooled coverage 84.66% before and 90.01% after;
  block-bootstrap 95% interval `[1.163790, 1.310996]`.

These are pooled score guarantees, not uniform trajectory guarantees. After
calibration, MH04 visual coverage is 88.73% and V1_03 inertial coverage is
88.51%. The JSON explicitly records that MH05, V2_02, and V2_03 were not opened
during fitting.

## Conformal Stage 3

The first three-sequence artifacts in `stage3/` are retained only as an invalid
pilot. Their visual sidecar lookup rates were approximately 0.2--3.2%, so most
visual updates silently used stock measurement noise. Do not use their
trajectory comparisons as learned-noise results.

The subsequent `stage3_all11/` attempt established valid stock baselines for
all eleven sequences, but its first non-stock run also failed: exact
`(timestamp_us, feature_id)` lookup reached only 3.8%. An audit showed only
54.3% feature-ID overlap and only 7.5% sub-pixel observation matches. Feature
identity and candidate timing change with the filter trajectory, so no relaxed
cross-run lookup is scientifically defensible. Retain this directory as a
negative audit, not as a learned-noise benchmark.

The replacement is `stage3_online_all11/`. Net A runs causally inside each
MSCKF update using the live feature batch, pre-decision residuals, and
hypothetical stock chi-square values. The C++ forward pass must match an
embedded PyTorch reference before execution, and the refactored updater must
first reproduce the MH01 stock ATE within 20%. The noncausal GT-oracle arm is
excluded rather than mislabeled. The three untouched test sequences (MH05,
V2_02, V2_03) remain the primary benchmark; the other eight results are labeled
train/calibration diagnostics.

The complete 146 MB replacement bundle is downloaded and all 130 entries in
`stage3_online_all11_checksums.sha256` pass locally. It contains 33 HDF5 runs,
the exported model, preflight output, validation/run logs, and six reports.
Primary held-out-test means are:

| arm | ATE RMSE (m) | RPE 1 s RMSE (m) | ATE ratio to stock | gate pass rate |
|---|---:|---:|---:|---:|
| stock | 0.1453 | 0.0405 | 1.00× | 74.07% |
| learned | 0.3571 | 0.0620 | 2.11× | 99.96% |
| conformalised | 0.4002 | 0.0712 | 2.79× | 100.00% |

Thus the current learned and conformalised visual-noise arms are rejected.
Net A frequently saturates at `sigma_pix≈exp(7)=1096.63`; conformal scaling
raises the maximum to 1283.27. This nearly disables chi-squared rejection and
degrades held-out trajectory accuracy. V2_03 learned is the only primary case
with an RPE improvement (0.914× stock), but its ATE is still 1.116× stock and
does not overturn the aggregate negative result.

## Validation gates

See [`validation_gates.md`](validation_gates.md) for the final Gate 1 and Gate 2 results. The two
known theory/self-test discrepancies remain intentionally unresolved and were not used as gates.
