# OpenVINS EuRoC benchmark

This harness builds OpenVINS for ROS Noetic in Docker, runs the stereo-inertial
configuration on all 11 EuRoC MAV sequences, and evaluates the saved
trajectories with evo.

## Remote server

- Server: `himkesh@10.24.36.121`
- Source: `/home/himkesh/open_vins`
- Dataset: `/mnt/euro_mav`
- Results: `/mnt/euro_mav/openvins_benchmark`
- Image: `openvins-euroc:noetic`

Run or resume the full benchmark on the server:

```bash
cd /home/himkesh/open_vins
DATA_ROOT=/mnt/euro_mav \
RESULTS_ROOT=/mnt/euro_mav/openvins_benchmark \
bash benchmark/euroc_benchmark.sh all
```

Existing trajectories are reused. Add `--rerun` to replace them. To process
one sequence, add `--sequence MH_01_easy`; its metric summary is written to a
separate `summary_MH_01_easy.csv`, preserving the full `summary.csv`.

The evaluation uses SE(3) Umeyama alignment, a 0.02-second association limit,
APE translation, and RPE translation with a 1-meter delta. Each sequence
directory contains the raw estimate and timing trace, TUM trajectories,
OpenVINS/ROS logs, evo text reports, PDF plots, and evo ZIP archives.

ROS Noetic may print a `class_loader::LibraryUnloadException` while unloading
`compressed_depth_image_transport` after serial bag playback. In the completed
run this occurred only during teardown: every saved estimate extended beyond
the last available ground-truth timestamp.
