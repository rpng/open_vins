#!/usr/bin/env bash
# All-in-one EuRoC run + evaluation for OpenVINS (ROS-free)
set -euo pipefail

DATASET_ROOT=${1:-"${HOME}/datasets/mav0"}
CONFIG_FILE=${2:-"${HOME}/workspace/open_vins/config/euroc_mav/estimator_config.yaml"}
RESULTS_DIR=${3:-"${HOME}/workspace/open_vins/results/euroc_demo_$(basename "${DATASET_ROOT}")"}

GT_CSV="${DATASET_ROOT}/state_groundtruth_estimate0/data.csv"
OV_ROOT="${HOME}/workspace/open_vins"
BUILD_DIR="${OV_ROOT}/examples_integration/build"

if [[ ! -f "${GT_CSV}" ]]; then
  echo "[ERREUR] Ground truth introuvable : ${GT_CSV}" >&2
  echo "Assurez-vous que DATASET_ROOT pointe vers un dataset EuRoC (contient state_groundtruth_estimate0)." >&2
  exit 1
fi

mkdir -p "${RESULTS_DIR}"

echo "==> Conversion ground truth -> TUM (${RESULTS_DIR}/groundtruth.txt)"
python3 "${OV_ROOT}/convert_euroc_gt.py" "${GT_CSV}" "${RESULTS_DIR}/groundtruth.txt"

if [[ ! -x "${BUILD_DIR}/euroc_reader_example" ]]; then
  echo "[ERREUR] euroc_reader_example non trouvé. Compilez via cmake .. && make dans ${BUILD_DIR}" >&2
  exit 1
fi

pushd "${BUILD_DIR}" >/dev/null
echo "==> Exécution OpenVINS sur ${DATASET_ROOT}"
time ./euroc_reader_example "${DATASET_ROOT}" "${CONFIG_FILE}" 2>&1 | tee "${RESULTS_DIR}/vio_output.log"
cp trajectory_estimated.txt "${RESULTS_DIR}/trajectory_estimated.txt"
popd >/dev/null

echo "==> Évaluation APE (SE3, Umeyama)"
if command -v evo_ape >/dev/null 2>&1; then
  evo_ape tum "${RESULTS_DIR}/groundtruth.txt" "${RESULTS_DIR}/trajectory_estimated.txt" --align -r full | tee "${RESULTS_DIR}/ape.txt"
else
  echo "[AVERTISSEMENT] evo_ape non installé. Saute l'évaluation APE." >&2
fi

echo "==> Évaluation RPE 10 m (drift)"
if command -v evo_rpe >/dev/null 2>&1; then
  evo_rpe tum "${RESULTS_DIR}/groundtruth.txt" "${RESULTS_DIR}/trajectory_estimated.txt" --delta 10 --pose_relation trans_part -r full | tee "${RESULTS_DIR}/rpe10.txt"
else
  echo "[AVERTISSEMENT] evo_rpe non installé. Saute l'évaluation RPE." >&2
fi

echo "==> Résultats disponibles dans ${RESULTS_DIR}"
ls -lh "${RESULTS_DIR}" | sed 's/^/   /'
