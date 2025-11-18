#!/usr/bin/env bash
set -e  # 중간에 에러 나면 바로 종료

# 이 스크립트(run_hilti_all.sh)가 있는 디렉토리 절대경로 계산
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

DATASETS=(
  exp02_construction_multilevel
  exp15_attic_to_upper_gallery
  exp21_outside_building
)

for ds in "${DATASETS[@]}"; do
  echo "====================================="
  echo "  Running dataset: $ds"
  echo "====================================="

  roslaunch ov_msckf hilti.launch dataset:=$ds

  echo "Finished dataset: $ds"
  echo
  sleep 5
done

echo "====================================="
echo "  All datasets finished."
echo "  Running post-processing: removePr.py"
echo "====================================="

# 스크립트 파일이 있는 디렉토리 기준으로 상대경로 실행
python3 "$SCRIPT_DIR/removePr.py"

echo "Post-processing done."