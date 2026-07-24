#!/usr/bin/env bash
# Run on the benchmark server after the Stage-2 Python files have been copied.
set -euo pipefail

repo_root=/home/himkesh/open_vins_conformal
dataset_root=/mnt/euro_mav
stage1_dir=/mnt/euro_mav/conformal_dumps
stage2_dir=/mnt/euro_mav/conformal_dumps/stage2
target_dir="${stage2_dir}/netb_targets"
array_dir="${stage2_dir}/training_arrays"
report_dir="${stage2_dir}/reports"

mkdir -p "${target_dir}" "${array_dir}" "${report_dir}"

docker run --rm \
  --user "$(id -u):$(id -g)" \
  -v "${repo_root}:/repo:ro" \
  -v "${dataset_root}:/data" \
  openvins-conformal:stage1 \
  python3 /repo/conformal/stage2_train/derive_netb_targets.py \
    --stage1-dir /data/conformal_dumps \
    --dataset-root /data \
    --out-dir /data/conformal_dumps/stage2/netb_targets \
    --overwrite \
  | tee "${report_dir}/derive_netb_targets.log"

docker run --rm \
  --user "$(id -u):$(id -g)" \
  -v "${repo_root}:/repo:ro" \
  -v "${dataset_root}:/data" \
  openvins-conformal:stage1 \
  python3 /repo/conformal/stage2_train/export_stage2_npz.py \
    --stage1-dir /data/conformal_dumps \
    --netb-sidecar-dir /data/conformal_dumps/stage2/netb_targets \
    --out-dir /data/conformal_dumps/stage2/training_arrays \
  | tee "${report_dir}/export_stage2_npz.log"

docker run --rm \
  --user "$(id -u):$(id -g)" \
  -v "${repo_root}:/repo:ro" \
  -v "${dataset_root}:/data" \
  openvins-conformal:stage1 \
  python3 /repo/conformal/stage2_train/validate_stage2_arrays.py \
    --array-dir /data/conformal_dumps/stage2/training_arrays \
    --out-dir /data/conformal_dumps/stage2/reports \
  | tee "${report_dir}/validate_stage2_arrays.log"

(
  cd "${stage2_dir}"
  find netb_targets training_arrays reports -type f -print0 \
    | sort -z \
    | xargs -0 sha256sum > stage2_prepare_checksums.sha256
)

echo "Stage-2 preparation complete: ${stage2_dir}"
