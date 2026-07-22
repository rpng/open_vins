#!/usr/bin/env python3
"""
gate3_python_frontend_crosscheck.py  --  Gate 3: cross-check against a ~200-line Python front-end.

Companion to: Section 19.3 (Gate 3).

SCHEDULE NOTE (Section 19.3): under the compressed ICRA timeline this gate is DEMOTED from
scheduled work to a CONTINGENCY -- run it only if track counts or residual scales look suspicious
after Gates 1 and 2 pass. It is documented here so it is ready to deploy on a bad day.

WHY THIS GATE EXISTS (Section 19.3): independently reimplement a MINIMAL feature tracker
(cv2.goodFeaturesToTrack + cv2.calcOpticalFlowPyrLK) and compare track counts and residual
magnitudes against the logged OpenVINS diagnostics (DiagnosticsLogger.hpp output). Two
independent implementations agreeing is meaningful evidence; disagreement LOCALISES a bug (in the
dump, the tracker config, or the image loading).

WHAT TO COMPARE:
    * number of features tracked per frame (this Python tracker vs /frame_diag/num_tracked)
    * KLT forward-backward error distribution (vs the per-feature klt_fwd_bwd_error column)
    * rough residual magnitudes

This is deliberately NOT a re-implementation of the filter -- only the front-end -- so it stays
around 200 lines.

TODO(intern): implement the minimal tracker and the comparison. Skeleton below.
"""

from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np

SHI_TOMASI = dict(maxCorners=200, qualityLevel=0.01, minDistance=8, blockSize=7)
LK = dict(winSize=(21, 21), maxLevel=3,
          criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))


def track_forward_backward(prev_gray: np.ndarray, cur_gray: np.ndarray, pts_prev: np.ndarray):
    """One KLT step with a forward-backward consistency check (matches Section 8.1's signal)."""
    pts_cur, st1, _ = cv2.calcOpticalFlowPyrLK(prev_gray, cur_gray, pts_prev, None, **LK)
    pts_back, st2, _ = cv2.calcOpticalFlowPyrLK(cur_gray, prev_gray, pts_cur, None, **LK)
    fb_err = np.linalg.norm(pts_prev - pts_back, axis=-1).reshape(-1)
    good = (st1.reshape(-1) == 1) & (st2.reshape(-1) == 1)
    return pts_cur, good, fb_err


def run_python_frontend(cam0_dir: str, max_frames: int = 500):
    """Run the minimal tracker over a folder of PNGs; return per-frame track counts + fb-error stats."""
    frames = sorted(Path(cam0_dir).glob("*.png"))[:max_frames]
    counts, fb_means = [], []
    prev_gray, pts = None, None
    for f in frames:
        gray = cv2.imread(str(f), cv2.IMREAD_GRAYSCALE)
        if prev_gray is None:
            pts = cv2.goodFeaturesToTrack(gray, **SHI_TOMASI)
        else:
            pts, good, fb = track_forward_backward(prev_gray, gray, pts)
            pts = pts[good].reshape(-1, 1, 2)
            counts.append(int(good.sum()))
            fb_means.append(float(np.mean(fb[good])) if good.any() else float("nan"))
            if len(pts) < SHI_TOMASI["maxCorners"] // 2:  # replenish
                pts = cv2.goodFeaturesToTrack(gray, **SHI_TOMASI)
        prev_gray = gray
    return np.array(counts), np.array(fb_means)


def crosscheck_against_dump(cam0_dir: str, h5_dump: str) -> bool:
    """Compare Python-front-end track counts / fb-error against the OpenVINS diagnostics dump."""
    raise NotImplementedError(
        "TODO(intern): read /frame_diag/num_tracked and per-feature klt_fwd_bwd_error from h5_dump; "
        "compare distributions to run_python_frontend(cam0_dir); flag large systematic disagreement.")


if __name__ == "__main__":
    print(__doc__.split("TODO")[0])
