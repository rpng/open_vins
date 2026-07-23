# Experiment results downloaded from the remote server

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

## Validation gates

See [`validation_gates.md`](validation_gates.md) for the final Gate 1 and Gate 2 results. The two
known theory/self-test discrepancies remain intentionally unresolved and were not used as gates.
