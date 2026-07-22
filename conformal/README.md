# conformal — learned, conformalised Q and R for OpenVINS

This directory is the entire conformal project scaffold. It sits **on top of** an
otherwise byte-for-byte unmodified OpenVINS — keeping `ov_msckf/` and `ov_core/`
untouched is the paper's central defensibility claim (Section 4.4 / 18.3), so all
conformal code lives here in `conformal/`.

conformal makes exactly **five hand-tuned numbers** input-dependent: the 4 IMU noise
densities inside **Q** (Net B) and the 1 pixel scalar inside **R** (Net A). The networks
only write to Q and R — they can never inject a position into the state.

**New here? Read [`../changes.md`](../changes.md) first** — it is the full runbook:
what every file is, how the three stages fit together, the validation gates you must
pass before trusting anything, the two implementation traps, and the week-by-week plan.

Layout:

| Directory | Stage / role | PDF part |
|-----------|--------------|----------|
| `theory/` | numerically verify the theorems | Part I §6 |
| `stage1_dumps/` | C++: run OpenVINS, dump HDF5 training data | Part IV §18 |
| `stage2_train/` | Python: the two heads, the loss, conformal repair | Part II §8–9 |
| `stage3_eval/` | Python: corruptions, metrics, inject σ, money plot | Part III §11–16 |
| `experiments/` | the claims (C0–C4) and ablations (A3–A7) | Part III §13–15 |
| `validation_gates/` | the three non-negotiable gates | Part IV §19 |
| `configs/` | map of the five numbers → OpenVINS keys | Part I §4 |

Almost every `.py`/`.cpp`/`.hpp` here is a **scaffold**: real interfaces, real
references to OpenVINS symbols and config keys, with the science left as clearly
marked `TODO(intern)`. Two files are fully implemented and runnable today:
`theory/verify_theory.py` and the loss/conformal `__main__` self-tests.
