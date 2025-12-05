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

# 1. posegraph.launch를 백그라운드에서 실행하고 PID 저장
  echo "Starting roslaunch loop_fusion posegraph.launch..."
  roslaunch loop_fusion posegraph.launch &
  POSEGRAPH_PID=$!
  echo "posegraph.launch started with PID: $POSEGRAPH_PID"

  roslaunch ov_msckf hilti.launch dataset:=$ds

# 3. ov_msckf 종료 후 posegraph 프로세스 종료
  echo "Terminating posegraph.launch (PID: $POSEGRAPH_PID)..."
  # 'kill' 명령을 사용하여 백그라운드 프로세스 종료
  # 만약 프로세스가 이미 종료되었을 수도 있으므로 실패해도 스크립트가 멈추지 않도록 '|| true' 추가
  kill $POSEGRAPH_PID || true
  # 자식 프로세스들이 완전히 종료될 시간을 잠시 기다림 (선택 사항이지만 안전을 위해)
  sleep 2

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