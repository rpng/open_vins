#!/usr/bin/env bash
set -e

# Calculate absolute path
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

DATASETS=(
  exp02_construction_multilevel
  exp15_attic_to_upper_gallery
  exp21_outside_building
)

# Function to kill background processes on script exit (Ctrl+C)
cleanup() {
    echo "Caught signal, killing all background processes..."
    pkill -P $$ # Kill all child processes of this script
}
trap cleanup SIGINT SIGTERM

for ds in "${DATASETS[@]}"; do
  echo "====================================="
  echo "  Running dataset: $ds"
  echo "====================================="

  # 1. Start posegraph.launch in background
  echo "Starting roslaunch loop_fusion posegraph.launch..."
  roslaunch loop_fusion posegraph.launch &
  POSEGRAPH_PID=$!
  echo "posegraph.launch started with PID: $POSEGRAPH_PID"

  # CRITICAL: Wait for posegraph to initialize completely
  # Adjust sleep time depending on node weight
  sleep 5

  # 2. Run the main processing node
  # If this command fails, the script will exit due to 'set -e'
  # Consider using '|| true' if you want to proceed even if processing fails
  roslaunch ov_msckf hilti.launch dataset:=$ds

  # 3. Terminate posegraph.launch nicely
  echo "Terminating posegraph.launch (PID: $POSEGRAPH_PID)..."

  # Send SIGINT (Ctrl+C) instead of default SIGTERM so roslaunch can shutdown children
  kill -INT $POSEGRAPH_PID || true
  
  # Wait for the process to actually exit prevents zombie processes
  wait $POSEGRAPH_PID 2>/dev/null || true

  # 4. Force Cleanup (Safety Net)
  # Sometimes nodes hang. Kill specific nodes by name to ensure clean state for next loop.
  # Replace 'posegraph_node_name' with the actual node name if known, or use broader patterns.
  # Using pkill -f to find running roslaunch processes related to loop_fusion
  echo "Ensuring no lingering ROS nodes..."
  pkill -f "roslaunch loop_fusion posegraph.launch" || true
  
  # Optional: Sleep to allow OS to free up ports
  sleep 3

  echo "Finished dataset: $ds"
  echo
done

echo "====================================="
echo "  All datasets finished."
echo "  Running post-processing: removePr.py"
echo "====================================="

python3 "$SCRIPT_DIR/removePr.py"

echo "Post-processing done."