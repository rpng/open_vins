#!/usr/bin/env bash
set -euo pipefail

repo_root=/home/himkesh/open_vins_conformal
dataset_root=/mnt/euro_mav
dump_root=/mnt/euro_mav/conformal_dumps
source_stage3="${dump_root}/stage3_all11"
stage2_dir="${dump_root}/stage2"
stage3_dir="${dump_root}/stage3_online_all11"
model_dir="${stage3_dir}/model"
preflight_dir="${stage3_dir}/preflight"
run_dir="${stage3_dir}/runs"
log_dir="${stage3_dir}/logs"
report_dir="${stage3_dir}/reports"
stage1_image=openvins-conformal:stage1
stage3_image=openvins-conformal:stage3-live
pytorch_image=rocm/pytorch:latest
visual_conformal_scale=1.1701864004

sequences=(
  MH_01_easy MH_02_easy MH_03_medium MH_04_difficult MH_05_difficult
  V1_01_easy V1_02_medium V1_03_difficult
  V2_01_easy V2_02_medium V2_03_difficult
)
declare -A start_offsets=(
  [MH_01_easy]=40
  [MH_02_easy]=35
  [MH_03_medium]=5
  [MH_04_difficult]=10
  [MH_05_difficult]=0
  [V1_01_easy]=0
  [V1_02_medium]=0
  [V1_03_difficult]=0
  [V2_01_easy]=0
  [V2_02_medium]=0
  [V2_03_difficult]=0
)
arms=(learned conformalised)
failures=()

mkdir -p "${model_dir}" "${preflight_dir}" "${run_dir}" "${log_dir}" "${report_dir}"

# Export the accepted checkpoint once. The HDF5 includes a reference forward
# pass that the C++ implementation must reproduce before any trajectory runs.
docker run --rm \
  --user "$(id -u):$(id -g)" \
  -v "${repo_root}:/repo:ro" \
  -v "${dataset_root}:/data" \
  -w /repo/conformal/stage2_train \
  "${pytorch_image}" \
  python3 /repo/conformal/stage3_eval/export_neta_weights_npz.py \
    --checkpoint /data/conformal_dumps/stage2/checkpoints/netA_geomvalid_seed7_epoch50_lr3e-4.pt \
    --output /data/conformal_dumps/stage3_online_all11/model/netA_live.npz \
  2>&1 | tee "${log_dir}/export_neta_weights.log"

docker run --rm \
  --user "$(id -u):$(id -g)" \
  -v "${repo_root}:/repo:ro" \
  -v "${dataset_root}:/data" \
  "${stage1_image}" \
  python3 /repo/conformal/stage3_eval/convert_neta_weights_to_h5.py \
    --input /data/conformal_dumps/stage3_online_all11/model/netA_live.npz \
    --output /data/conformal_dumps/stage3_online_all11/model/netA_live.h5 \
  2>&1 | tee "${log_dir}/convert_neta_weights.log"

docker build \
  -f "${repo_root}/conformal/Dockerfile.stage1.incremental" \
  -t "${stage3_image}" \
  "${repo_root}" \
  2>&1 | tee "${log_dir}/build_stage3_live_image.log"

# The UpdaterMSCKF loop was refactored into causal prepare/infer/apply phases.
# Prove that leaving the provider unset still preserves the accepted stock
# behavior before any learned-noise run is allowed.
if ! docker run --rm \
  --user "$(id -u):$(id -g)" \
  -v "${dataset_root}:/data" \
  "${stage3_image}" \
  /catkin_ws/devel/lib/conformal_stage1/run_asl_msckf \
    /catkin_ws/src/open_vins/config/euroc_mav/estimator_config.yaml \
    /data/MH_01_easy MH_01_easy \
    /data/conformal_dumps/stage3_online_all11/preflight/MH_01_easy_stock_refactor.h5 \
    40 \
  2>&1 | tee "${log_dir}/MH_01_easy_stock_refactor.log"; then
  failures+=("MH_01_easy_stock_refactor:runner")
fi

if [[ "${#failures[@]}" -eq 0 ]]; then
  if ! docker run --rm \
    --user "$(id -u):$(id -g)" \
    -v "${repo_root}:/repo:ro" \
    -v "${dataset_root}:/data" \
    "${stage1_image}" \
    python3 /repo/conformal/stage3_eval/compare_stock_pilots.py \
      --reference /data/conformal_dumps/stage3_all11/runs/MH_01_easy_stock.h5 \
      --candidate /data/conformal_dumps/stage3_online_all11/preflight/MH_01_easy_stock_refactor.h5 \
    2>&1 | tee "${log_dir}/MH_01_easy_stock_refactor.parity.log"; then
    failures+=("MH_01_easy_stock_refactor:parity")
  fi
fi

if [[ "${#failures[@]}" -ne 0 ]]; then
  printf '%s\n' "${failures[@]}" | tee "${report_dir}/stage3_failures.txt"
  exit 1
fi

# Reuse only the eleven stock pilots that already passed the corrected
# sequence-offset and ATE sanity gates. Validate each again before copying.
for sequence in "${sequences[@]}"; do
  source_dump="${source_stage3}/runs/${sequence}_stock.h5"
  if ! docker run --rm \
    --user "$(id -u):$(id -g)" \
    -v "${repo_root}:/repo:ro" \
    -v "${dataset_root}:/data" \
    "${stage1_image}" \
    python3 /repo/conformal/stage3_eval/validate_stock_pilot.py \
      --dump "/data/conformal_dumps/stage3_all11/runs/${sequence}_stock.h5" \
      --max-ate-rmse-m 1.0 \
      --min-feature-candidates 1000 \
    2>&1 | tee "${log_dir}/${sequence}_stock.sanity.log"; then
    failures+=("${sequence}_stock:source-sanity")
    break
  fi
  cp "${source_dump}" "${run_dir}/${sequence}_stock.h5"
done

if [[ "${#failures[@]}" -ne 0 ]]; then
  printf '%s\n' "${failures[@]}" | tee "${report_dir}/stage3_failures.txt"
  exit 1
fi

run_live_filter() {
  local sequence=$1
  local arm=$2
  local visual_scale=1.0
  if [[ "${arm}" == conformalised ]]; then
    visual_scale="${visual_conformal_scale}"
  fi
  local output="/data/conformal_dumps/stage3_online_all11/runs/${sequence}_${arm}.h5"
  local imu_sidecar="/data/conformal_dumps/stage3_all11/sidecars/${sequence}_${arm}_sigma.h5"
  if ! docker run --rm \
    --user "$(id -u):$(id -g)" \
    -v "${dataset_root}:/data" \
    "${stage3_image}" \
    /catkin_ws/devel/lib/conformal_stage1/run_asl_msckf \
      /catkin_ws/src/open_vins/config/euroc_mav/estimator_config.yaml \
      "/data/${sequence}" "${sequence}" "${output}" "${start_offsets[$sequence]}" \
      "${imu_sidecar}" \
      /data/conformal_dumps/stage3_online_all11/model/netA_live.h5 \
      "${visual_scale}" \
    2>&1 | tee "${log_dir}/${sequence}_${arm}.log"; then
    failures+=("${sequence}_${arm}:runner")
    return 1
  fi
  if ! docker run --rm \
    --user "$(id -u):$(id -g)" \
    -v "${dataset_root}:/data" \
    "${stage3_image}" \
    python3 /catkin_ws/src/open_vins/conformal/stage1_dumps/validate_dump.py \
      "${output}" \
    2>&1 | tee "${log_dir}/${sequence}_${arm}.validation.log"; then
    failures+=("${sequence}_${arm}:validation")
    return 1
  fi
  local predictions
  predictions=$(
    sed -n 's/^.*live Net-A batches=[0-9][0-9]* predictions=//p' \
      "${log_dir}/${sequence}_${arm}.log" |
      tail -n 1
  )
  if [[ -z "${predictions}" ]] || [[ "${predictions}" -le 0 ]]; then
    failures+=("${sequence}_${arm}:missing-live-neta")
    return 1
  fi
  printf 'PASS causal_live_neta_predictions=%s\n' "${predictions}" \
    | tee "${log_dir}/${sequence}_${arm}.live_neta.validation.log"
  return 0
}

abort_runs=false
for sequence in "${sequences[@]}"; do
  for arm in "${arms[@]}"; do
    if ! run_live_filter "${sequence}" "${arm}"; then
      abort_runs=true
      break
    fi
  done
  if [[ "${abort_runs}" == true ]]; then
    break
  fi
done

if [[ "${#failures[@]}" -eq 0 ]]; then
  if ! docker run --rm \
    --user "$(id -u):$(id -g)" \
    -v "${repo_root}:/repo:ro" \
    -v "${dataset_root}:/data" \
    "${stage1_image}" \
    python3 /repo/conformal/stage3_eval/evaluate_stage3_runs.py \
      --run-dir /data/conformal_dumps/stage3_online_all11/runs \
      --log-dir /data/conformal_dumps/stage3_online_all11/logs \
      --out-dir /data/conformal_dumps/stage3_online_all11/reports \
      --require-live-neta \
      --arms stock learned conformalised \
      --sequences "${sequences[@]}" \
    2>&1 | tee "${log_dir}/evaluate_stage3_runs.log"; then
    failures+=("evaluation-or-live-neta-validation")
  fi
fi

if [[ "${#failures[@]}" -ne 0 ]]; then
  printf '%s\n' "${failures[@]}" | tee "${report_dir}/stage3_failures.txt"
fi

printf '%s\n' \
  "inference=causal live batch Net-A inside MSCKF update" \
  "visual_inputs=live pre-decision residual and hypothetical stock chi-square" \
  "primary_split=MH_05_difficult,V2_02_medium,V2_03_difficult" \
  "supplementary_splits=train,calibration" \
  "arms=stock,learned,conformalised" \
  "oracle_arm=excluded because ground-truth visual noise is noncausal" \
  "visual_conformal_scale=${visual_conformal_scale}" \
  "inertial_model=accepted epoch-zero constant-Q correction" \
  > "${report_dir}/stage3_methodology.txt"

(
  cd "${stage3_dir}"
  find model preflight runs logs reports -type f -print0 \
    | sort -z \
    | xargs -0 sha256sum > stage3_online_all11_checksums.sha256
)

echo "Stage-3 online all-11 benchmark complete: ${stage3_dir}"
if [[ "${#failures[@]}" -ne 0 ]]; then
  exit 1
fi
