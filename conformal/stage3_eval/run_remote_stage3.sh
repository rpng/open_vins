#!/usr/bin/env bash
set -euo pipefail

repo_root=/home/himkesh/open_vins_conformal
dataset_root=/mnt/euro_mav
stage1_dir=/mnt/euro_mav/conformal_dumps
stage2_dir="${stage1_dir}/stage2"
stage3_dir="${stage1_dir}/stage3"
input_dir="${stage3_dir}/inputs"
prediction_dir="${stage3_dir}/predictions"
sidecar_dir="${stage3_dir}/sidecars"
run_dir="${stage3_dir}/runs"
log_dir="${stage3_dir}/logs"
report_dir="${stage3_dir}/reports"
stage1_image=openvins-conformal:stage1
stage3_image=openvins-conformal:stage3
pytorch_image=rocm/pytorch:latest

mkdir -p "${input_dir}" "${prediction_dir}" "${sidecar_dir}" \
  "${run_dir}" "${log_dir}" "${report_dir}"

docker run --rm \
  --user "$(id -u):$(id -g)" \
  -v "${repo_root}:/repo:ro" \
  -v "${dataset_root}:/data" \
  "${stage1_image}" \
  python3 /repo/conformal/stage3_eval/export_stage3_inputs.py \
    --stage1-dir /data/conformal_dumps \
    --netb-target-dir /data/conformal_dumps/stage2/netb_targets \
    --out-dir /data/conformal_dumps/stage3/inputs \
  2>&1 | tee "${log_dir}/export_stage3_inputs.log"

docker run --rm \
  --user "$(id -u):$(id -g)" \
  -e OMP_NUM_THREADS=16 \
  -v "${repo_root}:/repo:ro" \
  -v "${dataset_root}:/data" \
  -w /repo/conformal/stage2_train \
  "${pytorch_image}" \
  python3 /repo/conformal/stage3_eval/build_sigma_sidecars.py \
    --input-dir /data/conformal_dumps/stage3/inputs \
    --net-a /data/conformal_dumps/stage2/checkpoints/netA_geomvalid_seed7_epoch50_lr3e-4.pt \
    --net-b /data/conformal_dumps/stage2/checkpoints/netB_guarded_seed7_maxepoch50_lr3e-4.pt \
    --conformal /data/conformal_dumps/stage2/conformal/conformal_alpha0.10.json \
    --out-dir /data/conformal_dumps/stage3/predictions \
  2>&1 | tee "${log_dir}/build_sigma_sidecars.log"

docker run --rm \
  --user "$(id -u):$(id -g)" \
  -v "${repo_root}:/repo:ro" \
  -v "${dataset_root}:/data" \
  "${stage1_image}" \
  python3 /repo/conformal/stage3_eval/convert_sigma_npz_to_h5.py \
    --input-dir /data/conformal_dumps/stage3/predictions \
    --out-dir /data/conformal_dumps/stage3/sidecars \
  2>&1 | tee "${log_dir}/convert_sigma_npz_to_h5.log"

docker run --rm \
  --user "$(id -u):$(id -g)" \
  -v "${repo_root}:/repo:ro" \
  -v "${dataset_root}:/data" \
  "${stage1_image}" \
  python3 /repo/conformal/stage3_eval/validate_sigma_sidecars.py \
    --sidecar-dir /data/conformal_dumps/stage3/sidecars \
  2>&1 | tee "${log_dir}/validate_sigma_sidecars.log"

docker build \
  -f "${repo_root}/conformal/Dockerfile.stage1.incremental" \
  -t "${stage3_image}" \
  "${repo_root}" \
  2>&1 | tee "${log_dir}/build_stage3_image.log"

sequences=(MH_05_difficult V2_02_medium V2_03_difficult)
arms=(stock learned conformalised oracle)
failures=()
for sequence in "${sequences[@]}"; do
  for arm in "${arms[@]}"; do
    output="/data/conformal_dumps/stage3/runs/${sequence}_${arm}.h5"
    if [[ "${arm}" == stock ]]; then
      if ! docker run --rm \
        --user "$(id -u):$(id -g)" \
        -v "${dataset_root}:/data" \
        "${stage3_image}" \
        /catkin_ws/devel/lib/conformal_stage1/run_asl_msckf \
        /catkin_ws/src/open_vins/config/euroc_mav/estimator_config.yaml \
        "/data/${sequence}" "${sequence}" "${output}" 0 \
        2>&1 | tee "${log_dir}/${sequence}_${arm}.log"; then
        failures+=("${sequence}_${arm}:runner")
        continue
      fi
    else
      sidecar="/data/conformal_dumps/stage3/sidecars/${sequence}_${arm}_sigma.h5"
      if ! docker run --rm \
        --user "$(id -u):$(id -g)" \
        -v "${dataset_root}:/data" \
        "${stage3_image}" \
        /catkin_ws/devel/lib/conformal_stage1/run_asl_msckf \
        /catkin_ws/src/open_vins/config/euroc_mav/estimator_config.yaml \
        "/data/${sequence}" "${sequence}" "${output}" 0 "${sidecar}" \
        2>&1 | tee "${log_dir}/${sequence}_${arm}.log"; then
        failures+=("${sequence}_${arm}:runner")
        continue
      fi
    fi
    if ! docker run --rm \
      --user "$(id -u):$(id -g)" \
      -v "${dataset_root}:/data" \
      "${stage3_image}" \
      python3 /catkin_ws/src/open_vins/conformal/stage1_dumps/validate_dump.py \
      "/data/conformal_dumps/stage3/runs/${sequence}_${arm}.h5" \
      2>&1 | tee "${log_dir}/${sequence}_${arm}.validation.log"; then
      failures+=("${sequence}_${arm}:validation")
    fi
  done
done

if [[ "${#failures[@]}" -eq 0 ]]; then
  docker run --rm \
    --user "$(id -u):$(id -g)" \
    -v "${repo_root}:/repo:ro" \
    -v "${dataset_root}:/data" \
    "${stage1_image}" \
    python3 /repo/conformal/stage3_eval/evaluate_stage3_runs.py \
      --run-dir /data/conformal_dumps/stage3/runs \
      --log-dir /data/conformal_dumps/stage3/logs \
      --out-dir /data/conformal_dumps/stage3/reports \
    2>&1 | tee "${log_dir}/evaluate_stage3_runs.log"
else
  printf '%s\n' "${failures[@]}" | tee "${report_dir}/stage3_failures.txt"
fi

(
  cd "${stage3_dir}"
  find inputs predictions sidecars runs logs reports -type f -print0 \
    | sort -z \
    | xargs -0 sha256sum > stage3_checksums.sha256
)

echo "Stage-3 benchmark complete: ${stage3_dir}"
if [[ "${#failures[@]}" -ne 0 ]]; then
  exit 1
fi
