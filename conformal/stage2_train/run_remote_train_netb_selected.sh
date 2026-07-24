#!/usr/bin/env bash
# Leakage-safe Net-B epoch selection and all-training-sequence refit.
set -euo pipefail

repo_root=/home/himkesh/open_vins_conformal
stage2_dir=/mnt/euro_mav/conformal_dumps/stage2
report_dir="${stage2_dir}/reports"
image=rocm/pytorch:latest

mkdir -p "${stage2_dir}/checkpoints/smoke" "${report_dir}"
grep -q '"status": "PASS"' "${report_dir}/stage2_data_validation.json"

run_trainer() {
  docker run --rm \
    --user "$(id -u):$(id -g)" \
    --shm-size=2g \
    -e OMP_NUM_THREADS=16 \
    -v "${repo_root}:/repo:ro" \
    -v "${stage2_dir}:/stage2" \
    -w /repo/conformal/stage2_train \
    "${image}" "$@"
}

run_trainer python3 train_heads.py \
  --head B \
  --h5-dir /stage2/training_arrays \
  --sidecar-dir /stage2/training_arrays \
  --out /stage2/checkpoints/smoke/netB_guarded_maxepoch1.pt \
  --epochs 1 --lr 3e-4 --seed 7 --device cpu \
  | tee "${report_dir}/train_netB_guarded_smoke.log"

run_trainer python3 train_heads.py \
  --head B \
  --h5-dir /stage2/training_arrays \
  --sidecar-dir /stage2/training_arrays \
  --out /stage2/checkpoints/netB_guarded_seed7_maxepoch50_lr3e-4.pt \
  --epochs 50 --lr 3e-4 --seed 7 --device cpu \
  | tee "${report_dir}/train_netB_guarded_seed7_maxepoch50_lr3e-4.log"

(
  cd "${stage2_dir}"
  find checkpoints reports netb_targets training_arrays -type f \
    ! -name stage2_all_checksums.sha256 -print0 \
    | sort -z \
    | xargs -0 sha256sum > stage2_all_checksums.sha256
)

echo "Leakage-safe Net-B training complete: ${stage2_dir}"
