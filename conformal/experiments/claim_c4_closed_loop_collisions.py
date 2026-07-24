#!/usr/bin/env python3
"""
claim_c4_closed_loop_collisions.py  --  C4: honest uncertainty produces fewer collisions.

Companion to: Section 13.5 (claim C4).

STATUS: DEFERRED to the journal version (Section 13.5 / Section 23). It raises the ceiling
rather than supporting the core claim, and it costs a full simulator integration. This stub
marks where C4 lives; do NOT spend conference-timeline effort here.

STATEMENT: in closed loop, a planner consuming conformalised covariance collides LESS than one
consuming naive learned covariance.

WHY C4 EXISTS, AND WHY IT CAN BE CUT (Section 13.5): it closes the argument's loop --
overconfidence is dangerous because SOMETHING ACTS ON IT. A planner reading a dishonest, tight P
cuts clearance it cannot afford (Section 3.2). But it is not needed to establish the core result.

ENVIRONMENT: Flightmare (Section 11, "Closed loop -- a simulator where the estimate actually
flies the drone, so uncertainty has consequences").

MEASURED: collision rate / success rate for planners fed {naive learned, conformalised} P, over
many randomized runs.

SUCCESS CRITERION: conformalised P yields a statistically significant reduction in collisions
vs naive learned P at matched planner settings.

TODO(intern, JOURNAL): Flightmare integration; a covariance-aware planner (e.g. chance-constrained
or clearance ∝ sqrt of positional variance); randomized-trial harness with seeds.
"""

from __future__ import annotations

DEFERRED = True  # journal version (Section 13.5)


def run_c4(flightmare_cfg: str, out_csv: str) -> None:
    raise NotImplementedError("DEFERRED to journal (Section 13.5): Flightmare closed-loop collisions.")


if __name__ == "__main__":
    print(__doc__.split("TODO")[0])
