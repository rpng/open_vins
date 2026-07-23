# Remote continuation handoff

Resume this work autonomously. Do not ask the user to copy/paste terminal
commands. Use SSH/SCP yourself and report only results or genuine blockers.

## Workspace and remote

- Local repository: `/home/himkesh/Downloads/OPEN_VINS`
- Remote: `himkesh@10.24.36.121`
- SSH key: `/home/himkesh/.ssh/id_ed25519`
- Remote repository: `/home/himkesh/open_vins_conformal`
- Remote Stage-2 results: `/mnt/euro_mav/conformal_dumps/stage2`
- Local Stage-2 results: `conformal/results/stage2`

The user has explicitly authorized SSH/SCP and requested that Codex perform all
remote commands, monitoring, and downloads itself.

## Completed and downloaded

- All 11 Stage-1 HDF5 files and stock benchmarks.
- All 11 Stage-2 Net-B target sidecars.
- Portable Net-A and Net-B NPZ arrays.
- Stage-2 array validation passes for all 11 sequences.
- Aggregate checksums pass.
- Geometry-valid Net A is accepted:
  `checkpoints/netA_geomvalid_seed7_epoch50_lr3e-4.pt`
  - training NLL: 1.04437
  - frozen calibration NLL: 1.14010

## Rejected audit checkpoints

Retain these as audit artifacts but do not use them:

- `netA_seed7_epoch50.pt`: invalid extreme reprojection labels before the
  image-diagonal geometry gate; calibration NLL was about 1.11e10.
- `netB_seed7_epoch50.pt`: unstable optimization.
- `netB_stable_seed7_epoch50_lr3e-4.pt`: sequence overfit; calibration NLL 233.
- `netB_selected_seed7_maxepoch50_lr3e-4.pt`: epoch-count transfer/refit failed;
  calibration NLL 14.21 versus a training-fit constant baseline around 3.14.

## Exact next action

The final guarded Net-B code is local but has not been uploaded/run:

- `conformal/stage2_train/train_heads.py`
- `conformal/stage2_train/run_remote_train_netb_selected.sh`

Upload those two files to the remote repository's matching directory, run
`run_remote_train_netb_selected.sh`, monitor until completion, download the
entire remote Stage-2 result directory back into local
`conformal/results/stage2`, and verify `stage2_all_checksums.sha256`.

The guarded trainer:

- fits on MH01, MH02, V101, V102;
- validates on MH03, V201;
- never uses conformal-calibration trajectories for model selection;
- initializes at the fit-only optimal constant four-channel scale;
- permits epoch zero to win;
- retains the actual best validation-selected weights;
- stops after ten non-improving epochs;
- compares frozen calibration NLL with the fit-only constant baseline.

Accept learned Net B only if the final audit supports it. If epoch zero wins,
report that the experiment supports constant Q correction rather than a
conditional TCN. Do not tune against calibration or test.

After Net B is resolved, clearly mark accepted/rejected checkpoints in the
results README, then implement conformal quantile fitting without using test
sequences.
