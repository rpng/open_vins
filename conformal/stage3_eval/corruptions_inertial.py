#!/usr/bin/env python3
"""
corruptions_inertial.py  --  the inertial channel of the corruption suite (+ co-degradation).

Companion to: Section 11.1 ("The corruption suite, specified") -- the Inertial rows.

INERTIAL CORRUPTIONS (Section 11.1 table), applied to the raw IMU stream:
    vibration    band-limited vibration = propeller resonance; the failure Net B's
                 standard-deviation pool is designed to detect (Section 8.2, ablation A6).
    clipping     saturation -- readings that are physically un-integrable.
    bias_jump    thermal or mechanical shock to the IMU (a step in the bias).

CO-DEGRADATION (Section 11.1, and previews the thesis' Chapter 2 on correlated failure):
corruptions are applied both INDEPENDENTLY and in a co-degradation mode where visual and
inertial degrade TOGETHER, which is the physically realistic case -- a violent manoeuvre
blurs the camera and stresses the IMU at the same instant. co_degrade() ties a single
severity to both channels.

Convention matches corruptions_visual.py: severity 0 = clean, 1..5 increasing, 5 severe but
not catastrophic (ImageNet-C style).

IMU array convention: [N, 6] = [gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z], sample
rate ~200 Hz (Section 2.2). TODO(intern): confirm units/order match run_asl_msckf.cpp.
"""

from __future__ import annotations

import numpy as np

SEVERITIES = (0, 1, 2, 3, 4, 5)
IMU_RATE_HZ = 200.0


def _check_severity(severity: int) -> None:
    if severity not in SEVERITIES:
        raise ValueError(f"severity must be in {SEVERITIES}, got {severity}")


def vibration(imu: np.ndarray, severity: int, freq_hz: float = 55.0, seed: int | None = None) -> np.ndarray:
    """Add band-limited sinusoidal vibration (propeller resonance) to accel + gyro channels.

    The energy lives in the high-frequency content -- exactly what the std-pool detects and
    mean/max miss (Section 8.2). A6 relies on this being genuinely high-frequency.
    """
    _check_severity(severity)
    if severity == 0:
        return imu
    amp_acc = [0.0, 0.3, 0.7, 1.4, 2.5, 4.0][severity]   # m/s^2
    amp_gyr = [0.0, 0.02, 0.05, 0.1, 0.18, 0.3][severity]  # rad/s
    rng = np.random.default_rng(seed)
    n = imu.shape[0]
    t = np.arange(n) / IMU_RATE_HZ
    phase = rng.uniform(0, 2 * np.pi, size=6)
    # Narrow band around freq_hz: main tone + small jitter sidebands.
    tone = np.sin(2 * np.pi * freq_hz * t[:, None] + phase[None, :])
    out = imu.astype(np.float64).copy()
    out[:, 0:3] += amp_gyr * tone[:, 0:3]
    out[:, 3:6] += amp_acc * tone[:, 3:6]
    return out


def clipping(imu: np.ndarray, severity: int) -> np.ndarray:
    """Saturate the IMU to a shrinking range -> physically un-integrable readings."""
    _check_severity(severity)
    if severity == 0:
        return imu
    gyr_lim = [np.inf, 8.0, 6.0, 4.0, 3.0, 2.0][severity]   # rad/s
    acc_lim = [np.inf, 60.0, 45.0, 32.0, 24.0, 16.0][severity]  # m/s^2
    out = imu.astype(np.float64).copy()
    out[:, 0:3] = np.clip(out[:, 0:3], -gyr_lim, gyr_lim)
    out[:, 3:6] = np.clip(out[:, 3:6], -acc_lim, acc_lim)
    return out


def bias_jump(imu: np.ndarray, severity: int, at_frac: float = 0.5, seed: int | None = None) -> np.ndarray:
    """Inject a step change in bias partway through (thermal/mechanical shock)."""
    _check_severity(severity)
    if severity == 0:
        return imu
    step_gyr = [0.0, 0.01, 0.03, 0.06, 0.1, 0.16][severity]
    step_acc = [0.0, 0.05, 0.12, 0.25, 0.45, 0.7][severity]
    rng = np.random.default_rng(seed)
    out = imu.astype(np.float64).copy()
    k = int(len(out) * at_frac)
    out[k:, 0:3] += step_gyr * rng.standard_normal(3)
    out[k:, 3:6] += step_acc * rng.standard_normal(3)
    return out


INERTIAL_CORRUPTIONS = {
    "vibration": vibration,
    "clipping": clipping,
    "bias_jump": bias_jump,
}


def co_degrade(frames, imu, severity: int, visual_fn, inertial_fn):
    """Apply a matched severity to BOTH channels (Section 11.1 co-degradation mode).

    Args:
        frames: iterable of images ; imu: [N,6] array.
        visual_fn: one of corruptions_visual.VISUAL_CORRUPTIONS
        inertial_fn: one of INERTIAL_CORRUPTIONS
    Returns (corrupted_frames, corrupted_imu). This is the physically-realistic arm and the
    preview of the thesis' correlated-failure chapter.
    """
    corrupted_frames = [visual_fn(f, severity) for f in frames]
    corrupted_imu = inertial_fn(imu, severity)
    return corrupted_frames, corrupted_imu
