#!/usr/bin/env bash
# Run on the benchmark server after run_remote_prepare.sh has passed.
set -euo pipefail

repo_root=/home/himkesh/open_vins_conformal
stage2_dir=/mnt/euro_mav/conformal_dumps/stage2
array_dir="${stage2_dir}/training_arrays"
checkpoint_dir="${stage2_dir}/checkpoints"
report_dir="${stage2_dir}/reports"
image=rocm/pytorch:latest

mkdir -p "${checkpoint_dir}/smoke" "${report_dir}"
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

{
  echo "PyTorch/container preflight"
  run_trainer python3 -c \
    'import torch, numpy; print("torch", torch.__version__, "numpy", numpy.__version__, "cuda", torch.cuda.is_available())'
  run_trainer python3 net_a_visual_deepsets.py
  run_trainer python3 net_b_imu_dilated_tcn.py
} | tee "${report_dir}/training_preflight.log"

run_trainer python3 train_heads.py \
  --head A \
  --h5-dir /stage2/training_arrays \
  --out /stage2/checkpoints/smoke/netA_geomvalid_epoch1.pt \
  --epochs 1 --lr 3e-4 --seed 7 --device cpu \
  | tee "${report_dir}/train_netA_geomvalid_smoke.log"

run_trainer python3 train_heads.py \
  --head B \
  --h5-dir /stage2/training_arrays \
  --sidecar-dir /stage2/training_arrays \
  --out /stage2/checkpoints/smoke/netB_stable_epoch1.pt \
  --epochs 1 --lr 3e-4 --seed 7 --device cpu \
  | tee "${report_dir}/train_netB_stable_smoke.log"

run_trainer python3 train_heads.py \
  --head A \
  --h5-dir /stage2/training_arrays \
  --out /stage2/checkpoints/netA_geomvalid_seed7_epoch50_lr3e-4.pt \
  --epochs 50 --lr 3e-4 --seed 7 --device cpu \
  | tee "${report_dir}/train_netA_geomvalid_seed7_epoch50_lr3e-4.log"

run_trainer python3 train_heads.py \
  --head B \
  --h5-dir /stage2/training_arrays \
  --sidecar-dir /stage2/training_arrays \
  --out /stage2/checkpoints/netB_stable_seed7_epoch50_lr3e-4.pt \
  --epochs 50 --lr 3e-4 --seed 7 --device cpu \
  | tee "${report_dir}/train_netB_stable_seed7_epoch50_lr3e-4.log"

(
  cd "${stage2_dir}"
  find checkpoints reports netb_targets training_arrays -type f \
    ! -name stage2_all_checksums.sha256 -print0 \
    | sort -z \
    | xargs -0 sha256sum > stage2_all_checksums.sha256
)

echo "Stage-2 training complete: ${stage2_dir}"
