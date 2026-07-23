#!/usr/bin/env bash
set -euo pipefail

repo_root=/home/himkesh/open_vins_conformal
stage2_dir=/mnt/euro_mav/conformal_dumps/stage2
report_dir="${stage2_dir}/reports"
conformal_dir="${stage2_dir}/conformal"
image=rocm/pytorch:latest

mkdir -p "${conformal_dir}" "${report_dir}"

test -s "${repo_root}/conformal/stage2_train/fit_conformal_scalers.py"
test -s "${repo_root}/conformal/stage2_train/split_conformal_per_modality.py"
test -s "${stage2_dir}/checkpoints/netA_geomvalid_seed7_epoch50_lr3e-4.pt"
test -s "${stage2_dir}/checkpoints/netB_guarded_seed7_maxepoch50_lr3e-4.pt"

{
  docker run --rm \
    --user "$(id -u):$(id -g)" \
    -e OMP_NUM_THREADS=16 \
    -e PYTHONPYCACHEPREFIX=/tmp/pycache \
    -v "${repo_root}:/repo:ro" \
    -v "${stage2_dir}:/stage2" \
    -w /repo/conformal/stage2_train \
    "${image}" \
    python3 -m py_compile \
      fit_conformal_scalers.py split_conformal_per_modality.py

  docker run --rm \
    --user "$(id -u):$(id -g)" \
    -e OMP_NUM_THREADS=16 \
    -v "${repo_root}:/repo:ro" \
    -v "${stage2_dir}:/stage2" \
    -w /repo/conformal/stage2_train \
    "${image}" \
    python3 fit_conformal_scalers.py \
      --array-dir /stage2/training_arrays \
      --net-a /stage2/checkpoints/netA_geomvalid_seed7_epoch50_lr3e-4.pt \
      --net-b /stage2/checkpoints/netB_guarded_seed7_maxepoch50_lr3e-4.pt \
      --alpha 0.1 \
      --bootstrap-replicates 1000 \
      --out /stage2/conformal/conformal_alpha0.10.json
} 2>&1 | tee "${report_dir}/fit_conformal_alpha0.10.log"

test -s "${conformal_dir}/conformal_alpha0.10.json"

(
  cd "${stage2_dir}"
  find checkpoints conformal reports netb_targets training_arrays -type f \
    ! -name stage2_all_checksums.sha256 -print0 \
    | sort -z \
    | xargs -0 sha256sum > stage2_all_checksums.sha256
)

echo "Stage-2 conformal calibration complete: ${stage2_dir}"
