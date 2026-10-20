# Causal RGB radiance-grid development qualification

> Execute inline under the existing autonomous research authorization. No additional feasibility approval or new worktree is required; the original project remains read-only.

**Goal:** Test one fixed, causal, per-eye RGB inverse-rendering distance estimator on all 110 existing E0 groups before considering an RGB semantic source or navigation.

**Architecture:** Four separate explicit metric density/RGB grids. Each eye updates its grid from its own current/past10s RGB and causal estimated camera matrices. Differentiable emission/absorption rendering couples ray depths through a shared 3D representation and models transmittance/occlusion. No pretrained network, point correspondence, measured-depth initialization, scene mesh or known obstacle dimensions. This is a small original implementation inspired by DVGO's coarse density/color representation, not a claim to reproduce official DVGO accuracy.

**Spec:** `experiments/causal_radiance_grid_20261016_v1/protocol.json` freezes all values and failure handling before dispatch.

**Novelty:** Directly compared the closed 5x5 ZNCC per-pixel plane-sweep report and v98. Neither jointly estimates a density/radiance scene using a rendered image loss. Classical space carving was also considered: its maximal photo hull alone is not a certified obstacle surface; no camera-tube free-space premise is invented here.

**Files:** `field.py` owns rendering/scene parameter updates; `run.py` owns sanitized input preparation and causal generation; `score.py` owns post-seal GT-only evaluation/plots; `launch.py` owns the registered durable run. Reuse v76 projection and v95 targets without editing either.

## Frozen steps

- [x] Register one compute run, one administrative recovery at most, 1800s generation timeout and physical/nav0 in the initial stage. Record all source and capture identities.
- [x] Prepare all110 RGB/estimated-pose/CAD-only input packets; check original capture SHA, times, dimensions and nonempty RGB supervision for each eye. No depth, object identities, render pose or target labels in the fitting packets. CAD masks are inherited self-geometry qualification aids and do not establish a deployable mask source.
- [x] Run numerical renderer checks, then one fixed32-update-per-eye-per-frame online scene optimization schedule. All future packets stay unloaded until their turn; sampling uses only the last10s and the same eye. Scene fitting is real optimization and must not be called fit0.
- [x] Predict every eye at full resolution. Frozen opacity/concentration/photometric rules and the existing .1m baseline guard may abstain; missing output stays unknown. Seal all440 outputs before reading GT labels.
- [x] Evaluate actual RGB-derived distance with ideal offline classes using the unchanged semantic consumer and all11652 original targets, including suspended cells. Report depth error/coverage, per-eye and union maps, per-frame failure, synchronization/latency and actual terminal status. This necessary hybrid is not online semantic performance or navigation admission.
- [x] On any original accuracy/coverage/per-frame or .5s necessary latency gate failure, close the candidate and its remaining budget without tuning iterations, grid, losses or thresholds. Preserve implementation failures separately. If it passes, only independent data and actual RGB category-source qualification may follow; do not navigate automatically.

**Review focus:** USD-optical axes and metric recentering; future-frame exclusion; density transmittance vs axial distance; all-target denominator; real scene fitting/latency disclosure. Renderer numerical checks and sealed input/source identities precede actual generation; complete saved predictions are scored afterwards. No old experiment is rerun.
