# Remote continuation handoff

## Workspace and remote

- Local repository: `/home/himkesh/Downloads/OPEN_VINS`
- Remote: `himkesh@10.24.36.121`
- SSH key: `/home/himkesh/.ssh/id_ed25519`
- Remote repository: `/home/himkesh/open_vins_conformal`
- Remote data/results root: `/mnt/euro_mav/conformal_dumps`
- Local results root: `conformal/results`

The user authorized SSH/SCP. Remote access is working.

## Completed

- Stage 1: all 11 EuRoC sequences validated and downloaded.
- Stage 2: all 11 arrays, accepted Net A, epoch-zero constant-Q Net B, and
  fixed calibration artifacts validated and downloaded.
- The first Stage-3 test-only pilot is downloaded under `results/stage3/`, but
  it is invalid for learned visual-noise comparison.
- The corrected-offset all-11 stock pilots passed. Its first non-stock run
  failed exact lookup at 3.8%; ID and observation audits proved cross-run
  sidecars scientifically invalid.

## Completed replacement

`stage3_eval/run_remote_stage3_online_all11.sh` ran all 11 sequences in three
arms: stock, learned, and conformalised. It:

1. exports the accepted PyTorch Net-A checkpoint to self-verifying HDF5;
2. builds the causal live-inference runner;
3. verifies C++/PyTorch model parity at load time;
4. verifies that the refactored updater preserves MH01 stock behavior;
5. reuses the eleven already-validated corrected-offset stock pilots;
6. runs 22 causal non-stock trajectories;
7. writes primary held-out-test and labeled all-11 supplementary tables; and
8. writes SHA-256 checksums under
   `/mnt/euro_mav/conformal_dumps/stage3_online_all11`.

Primary test sequences remain MH05, V2_02, and V2_03. The other eight are
train/calibration diagnostics, not unbiased generalization evidence. The
remote job exited, its completion marker is present, and all remote checksums
passed. The complete 146 MB directory was downloaded to
`conformal/results/stage3_online_all11`; all 130 checksums also pass locally.

Primary test mean ATE RMSE is 0.1453 m stock, 0.3571 m learned, and 0.4002 m
conformalised. Mean ATE ratios are 2.11× and 2.79× stock. Net A saturates at
very large sigma values and drives gate acceptance to nearly 100%, so both
non-stock arms are rejected. Preserve these artifacts as the final negative
result for the current model.

Do not interpret or publish any non-stock trajectory unless the model parity,
stock parity, structural validation, and live-inference reports all pass.
