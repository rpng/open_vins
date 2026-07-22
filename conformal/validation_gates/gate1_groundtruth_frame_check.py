#!/usr/bin/env python3
"""
gate1_groundtruth_frame_check.py  --  Gate 1: verify the ground-truth frame convention.

Companion to: Section 19.1 (Gate 1) and Section 19 ("The validation gates -- non-negotiable,
and in this order"). Week 1 (Section 21). This is a KILL GATE (Section 22): "Gates 1-2 not
green by end of Week 1 -> ICRA is off; revert to the IROS timeline."

WHY THIS GATE EXISTS (Section 19.1): Vicon motion capture reports the pose of the MARKER BODY,
not the IMU frame. The transform T_BS from the sensor YAML must be applied to convert between
them. Getting this wrong biases EVERY computed error, which silently corrupts every sigma the
networks learn. NOTHING FAILS LOUDLY -- training converges, losses look reasonable, and every
downstream number is wrong. "This is the mistake that quietly kills projects of this shape."

THE CHECK (Section 19.1): on a STATIC segment, confirm that acceleration derived from ground
truth matches the raw IMU reading minus gravity and bias. If those two curves do not agree, the
transform is wrong.

    a_from_gt(t)  ==  R_gt(t) * (a_imu(t) - b_a)  +  g          (in the world frame)
  equivalently in the IMU frame:
    a_imu(t) - b_a  ==  R_gt(t)^T * (a_from_gt(t) - g)

On a truly static segment a_from_gt ~ 0, so the raw accel should read ~ R^T * (-g) + b_a, i.e.
just gravity rotated into the (correctly-transformed) IMU frame plus bias. A mis-applied T_BS
shows up as a persistent direction/scale mismatch here.

TODO(intern): load a static EuRoC segment (e.g. the initial stationary period of MH_01), the
raw IMU, and the GT; apply T_BS from config/euroc_mav/kalibr_imucam_chain.yaml (or the ASL
sensor.yaml); plot/compare the two acceleration curves; assert agreement within tolerance.
"""

from __future__ import annotations

import numpy as np

GRAVITY = np.array([0.0, 0.0, -9.81])  # world-frame gravity; match your OpenVINS gravity_mag/sign


def acceleration_from_groundtruth(p_gt: np.ndarray, t_gt: np.ndarray) -> np.ndarray:
    """Second time-derivative of GT position -> world-frame acceleration (finite differences).
    On a static segment this should be ~0. TODO(intern): smooth before differentiating twice."""
    v = np.gradient(p_gt, t_gt, axis=0)
    a = np.gradient(v, t_gt, axis=0)
    return a


def predicted_specific_force(a_world: np.ndarray, R_gt_world_to_imu: np.ndarray,
                             bias_accel: np.ndarray) -> np.ndarray:
    """What the accelerometer SHOULD read: f = R^T (a_world - g) + b_a. Compare to raw IMU."""
    return (R_gt_world_to_imu @ (a_world - GRAVITY)) + bias_accel


def check_gate1(seq_dir: str, static_window_s: tuple[float, float], tol_mps2: float = 0.3) -> bool:
    """Return True iff GT-derived specific force matches the raw IMU on the static window.

    static_window_s: (t0, t1) seconds of a known-stationary period.
    tol_mps2: max allowed mean abs discrepancy (m/s^2). Tighten once you trust the pipeline.
    """
    raise NotImplementedError(
        "TODO(intern): load raw IMU + GT for the window, apply T_BS, compare raw accel to "
        "predicted_specific_force; return mean|discrepancy| < tol_mps2. FAIL -> T_BS is wrong; "
        "do not proceed to any training (Section 19.1).")


if __name__ == "__main__":
    print(__doc__.split("TODO")[0])
