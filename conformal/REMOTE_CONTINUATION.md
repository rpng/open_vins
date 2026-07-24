# Remote continuation handoff

## Workspace and remote

- Local repository: `/home/himkesh/Downloads/OPEN_VINS`
- Remote: `himkesh@10.24.36.121`
- SSH key: `/home/himkesh/.ssh/id_ed25519`
- Remote repository: `/home/himkesh/open_vins_conformal`
- Remote data/results root: `/mnt/euro_mav/conformal_dumps`
- Local results root: `conformal/results`

The user authorized SSH/SCP. Codex remote operations are currently blocked by
an internal approval-service error (`X-OpenAI-Internal-Codex-Responses-Lite`),
not by the SSH key or server.

## Completed

- Stage 1: all 11 EuRoC sequences validated and downloaded.
- Stage 2: all 11 arrays, accepted Net A, epoch-zero constant-Q Net B, and
  fixed calibration artifacts validated and downloaded.
- The first Stage-3 test-only pilot is downloaded under `results/stage3/`, but
  it is invalid for learned visual-noise comparison.
- The corrected-offset all-11 stock pilots passed. Its first non-stock run
  failed exact lookup at 3.8%; ID and observation audits proved cross-run
  sidecars scientifically invalid.

## Prepared replacement

`stage3_eval/run_remote_stage3_online_all11.sh` runs all 11 sequences in three
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
train/calibration diagnostics, not unbiased generalization evidence.

## Files that must be uploaded

- `conformal/stage1_dumps/run_asl_msckf.cpp`
- `ov_msckf/src/update/ConformalHooks.h`
- `ov_msckf/src/update/UpdaterMSCKF.h`
- `ov_msckf/src/update/UpdaterMSCKF.cpp`
- `ov_msckf/src/core/VioManager.h`
- `ov_msckf/src/core/VioManager.cpp`
- `conformal/Dockerfile.stage1.incremental`
- all scripts in `conformal/stage3_eval/`, including
  `run_remote_stage3_online_all11.sh`

Then run:

```bash
bash /home/himkesh/open_vins_conformal/conformal/stage3_eval/run_remote_stage3_online_all11.sh
```

On success, download the entire remote `stage3_online_all11` directory to
`conformal/results/stage3_online_all11` and verify:

```bash
sha256sum -c stage3_online_all11_checksums.sha256
```

Do not interpret or publish any non-stock trajectory unless the model parity,
stock parity, structural validation, and live-inference reports all pass.
