# Current four-camera weak classical baseline

Implemented and actually executed 2026-10-17. This is a conditional, controlled-unknown simulation baseline, not an admitted safe RGB source or reliable navigation result. No learned model/fit, camera/lighting changes, original-project edits or Git publication.

## Data flow

Four synchronized native RGBs + measured three joints → same archived OpenCV SGBM448/block5/LR1.5 parameters, current F=294.770878px/B=.65012m, CCW-rectified frontL/frontR and rearR/rearL → both signed depth query directions → CAD/range validity and native estimated camera transforms → explicit .05–1.5m world-height risk proxy (NOT RGB semantics) → existing B RepeatedExposureMemoryGrid → existing local grid/kinodynamic planner and fixed tracker wrappers → roll PID → actual requests/physics. Missing is unknown; no observed-free footprint or GT decision geometry.

Privileged initial gauge, fixed .4m base-height/rolling radius, CAD/nominal calibration, static task-ground/safe-start priors and ideal lowlevel bank feedback are disclosed. The task goal is known; no full GT route/map enters the policy. GT geometry is stored offline; true goal/contact enters terminal evaluation only. Computation freezes simulation time, so90s simulation is not90s wall time.

## This run's actual entry points

Use `/home/an/miniconda3/envs/ball_wm/bin/python`. Frozen `protocol.json`, `navigation_protocol.json` and `evaluation_protocol.json` contain identities and independent budgets. Actual stages:

1. `interface_check.py`: signed pair, pinhole units, validity/CAD interface.
2. `supervise.py offline`: full110-group existing E0, predictions sealed before teacher evaluation.
3. `score.py`: complete E0 targets, categories/unknown not reduced; `overlap_diagnosis.py` is separately bounded postseal pair-support diagnosis.
4. `supervise.py navigation`: ONE actual E1 90s B-memory baseline. Durable PID/log/timeout, cap1/recovery≤1.
5. `evaluate_sealed_run.py`: verifies frozen source/input identities, seals all action/map/trajectory/video evidence, evaluates, verifies bytes again.
6. `supported_memory_diagnostic.py`: observed-source-supported blind denominator, distinct from all-visible coverage gaps.
7. `diagnose_no_route.py`: one predefined first>=5s sustained no-route event; GT counterfactual only, not an online clearing rule.

The existing output/ledger guards intentionally reject relaunch. Reproduction requires a separately prepared copy and registered budget; do not erase a closed ledger or overwrite this evidence. Current local environment/absolute asset dependencies are pinned; cross-machine execution has NOT been verified.

## Evidence

- `source_snapshot/`, `source_snapshot_index.json` — 142 actual local source/config byte copies, with external USD/SDK dependencies still pinned separately.
- `e0_predictions/`, `prediction_seal.json`, `e0_result.json` — actual E0 RGB output and full-domain teacher score.
- `e1_run/` — 181 raw four-camera captures and predictions, four videos, 900 maps/plans and requests, 4501 native packets/physics, action journal, contacts, episode/stage/source identities.
- `e1_run/artifact_manifest_post_close.json`, `evaluation_receipt.json`, `author_audit.json` — complete bytes, causal clocks, requests/video checks.
- `e1_result.json`, `supported_memory_result.json`, `no_route_diagnosis.json` — results and once-bounded failure explanation.
- `navigation_status.json`, `navigation_ledger.json` — normal process exit0 versus scientific timeout, separate from environment failure.
- `../view_supported_memory_20261017_v1/` — same-input A/B/C replay only; fixed C rejected, not actual three-arm navigation.

Read the linked report in `reports/current_four_camera_weak_baseline_results_20261017.md`. Old strict source/nav results and closed budgets stay unchanged. Whole research/learning increment is not complete.
