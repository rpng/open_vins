#!/usr/bin/env python3
"""
corruptions_visual.py  --  the visual channel of the corruption suite.

Companion to: Section 11.1 ("The corruption suite, specified") -- the Visual rows, and
              Section 13.1 (claim C0, which rests on this suite being precise).

The corruption suite is the experimental instrument on which C0 rests -- it gives the
controlled x-axis (degradation severity) needed to plot a failure curve and overlay the
Theorem-2 threshold (the "money plot", Section 16). Following the ImageNet-C convention:
    * severity 0 = clean, severities 1..5 = increasing degradation,
    * level 5 is "severe but not catastrophic" (Section 11.1).

VISUAL CORRUPTIONS (Section 11.1 table):
    motion_blur          kernel length ∝ severity -- directly attacks KLT tracking; the
                         canonical aggressive-flight failure.
    brightness_gamma     simulates low light; reduces corner count and response.
    sensor_noise_jpeg    additive noise + compression artefacts; produces FALSE corners
                         (tracks that survive but are wrong).
    occlusion            sudden loss of a spatial region; tests the set network's handling
                         of varying feature counts.

These are applied to EuRoC frames that were NEVER seen in training (zero-shot). See also
co-degradation (corruptions_inertial.py): visual+inertial degrade together, the physically
realistic case, since a violent manoeuvre blurs the camera AND stresses the IMU.

TODO(intern): calibrate each severity->parameter map so that level 5 is severe-but-not-
catastrophic on EuRoC (a few frames of hand-tuning). The maps below are reasonable starts.
"""

from __future__ import annotations

import cv2
import numpy as np

SEVERITIES = (0, 1, 2, 3, 4, 5)


def _check_severity(severity: int) -> None:
    if severity not in SEVERITIES:
        raise ValueError(f"severity must be in {SEVERITIES}, got {severity}")


def motion_blur(img: np.ndarray, severity: int, angle_deg: float = 0.0) -> np.ndarray:
    """Linear motion blur; kernel length grows with severity (Section 11.1)."""
    _check_severity(severity)
    if severity == 0:
        return img
    length = [0, 3, 5, 9, 15, 23][severity]  # TODO(intern): calibrate on EuRoC
    kernel = np.zeros((length, length), dtype=np.float32)
    kernel[length // 2, :] = 1.0
    M = cv2.getRotationMatrix2D((length / 2 - 0.5, length / 2 - 0.5), angle_deg, 1.0)
    kernel = cv2.warpAffine(kernel, M, (length, length))
    kernel /= kernel.sum() + 1e-12
    return cv2.filter2D(img, -1, kernel)


def brightness_gamma(img: np.ndarray, severity: int) -> np.ndarray:
    """Low-light: multiplicative dimming + gamma (Section 11.1). Reduces corner response."""
    _check_severity(severity)
    if severity == 0:
        return img
    gain = [1.0, 0.75, 0.6, 0.45, 0.3, 0.2][severity]
    gamma = [1.0, 1.2, 1.5, 1.8, 2.2, 2.6][severity]
    x = (img.astype(np.float32) / 255.0) * gain
    x = np.power(np.clip(x, 0, 1), gamma)
    return (x * 255.0).astype(img.dtype)


def sensor_noise_jpeg(img: np.ndarray, severity: int) -> np.ndarray:
    """Additive Gaussian noise + JPEG artefacts (Section 11.1). Produces FALSE corners."""
    _check_severity(severity)
    if severity == 0:
        return img
    sigma = [0, 5, 10, 18, 28, 40][severity]
    quality = [100, 60, 45, 32, 22, 14][severity]
    noisy = img.astype(np.float32) + np.random.normal(0, sigma, img.shape)
    noisy = np.clip(noisy, 0, 255).astype(np.uint8)
    ok, enc = cv2.imencode(".jpg", noisy, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    return cv2.imdecode(enc, cv2.IMREAD_GRAYSCALE) if ok else noisy


def occlusion(img: np.ndarray, severity: int, seed: int | None = None) -> np.ndarray:
    """Black occlusion patches covering a growing image fraction (Section 11.1)."""
    _check_severity(severity)
    if severity == 0:
        return img
    frac = [0.0, 0.05, 0.12, 0.22, 0.35, 0.5][severity]
    rng = np.random.default_rng(seed)
    out = img.copy()
    h, w = img.shape[:2]
    ph, pw = int(h * np.sqrt(frac)), int(w * np.sqrt(frac))
    y = rng.integers(0, max(1, h - ph))
    x = rng.integers(0, max(1, w - pw))
    out[y:y + ph, x:x + pw] = 0
    return out


VISUAL_CORRUPTIONS = {
    "motion_blur": motion_blur,
    "brightness_gamma": brightness_gamma,
    "sensor_noise_jpeg": sensor_noise_jpeg,
    "occlusion": occlusion,
}
