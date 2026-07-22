#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

REMOTE_HOST="${REMOTE_HOST:-himkesh@10.24.36.121}"
REMOTE_REPO="${REMOTE_REPO:-/home/himkesh/open_vins}"
REMOTE_RESULTS="${REMOTE_RESULTS:-/mnt/euro_mav/openvins_benchmark}"
SSH_KEY="${SSH_KEY:-/home/himkesh/.ssh/id_ed25519}"

ssh -i "$SSH_KEY" -o BatchMode=yes \
  "$REMOTE_HOST" "mkdir -p '$REMOTE_REPO' '$REMOTE_RESULTS'"

rsync -az \
  -e "ssh -i $SSH_KEY -o BatchMode=yes" \
  --exclude '.git/' \
  --exclude '.agents/' \
  --exclude '.codex/' \
  --exclude 'catkin_ws/' \
  --exclude 'EuRoC_MAV/' \
  --exclude 'openvins_benchmark/' \
  "$REPO_DIR/" "$REMOTE_HOST:$REMOTE_REPO/"

printf 'OpenVINS source deployed to %s:%s\n' "$REMOTE_HOST" "$REMOTE_REPO"
printf 'Run remotely:\n'
printf '  cd %q && DATA_ROOT=/mnt/euro_mav RESULTS_ROOT=%q bash benchmark/euroc_benchmark.sh all\n' \
  "$REMOTE_REPO" "$REMOTE_RESULTS"
