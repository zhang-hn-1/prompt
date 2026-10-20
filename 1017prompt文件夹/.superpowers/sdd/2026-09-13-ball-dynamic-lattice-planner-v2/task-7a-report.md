# Task 7A: Cache Drive Interpolation Tables

## Status

**IMPLEMENTED, CORRECTNESS VERIFIED; Task 7 performance gate remains blocked.**

`DriveModel` now builds its forward and reverse sorted steering groups once in
`__post_init__` and stores them as tuples.  `speed()` and `curvature()` select
the existing direction table instead of filtering and sorting steering samples
for every query.  The cache is bounded to the two immutable drive directions;
it is not keyed by continuous main-command values.  Planner search order,
costs, heuristics, maps, state quantization, primitive integration, and result
schema were not modified.

## TDD evidence

Added `test_repeated_speed_and_curvature_queries_reuse_directional_interpolation_tables`.
It constructs the real model, then replaces the module-level `sorted` lookup
with a counter before repeated forward/reverse `speed()` and `curvature()`
queries.  It also asserts the frozen interpolation values in both directions.

RED command:

```text
PYTHONPATH=research/ball_wm /home/an/miniconda3/envs/ball_wm/bin/python -m pytest research/ball_wm/tests/test_drive_model.py -q
```

Before the implementation, this failed at `assert sort_calls == 0` with
`sort_calls == 12`: four queries each rebuilt and sorted three direction
groups.  This is the expected missing-cache failure, not a test setup error.

GREEN commands:

```text
PYTHONPATH=research/ball_wm /home/an/miniconda3/envs/ball_wm/bin/python -m pytest research/ball_wm/tests/test_drive_model.py -q
# 13 passed in 0.22s

PYTHONPATH=research/ball_wm /home/an/miniconda3/envs/ball_wm/bin/python -m pytest research/ball_wm/tests/test_dynamic_lattice_planning.py -q
# 32 passed in 11.55s
```

## Frozen detour reproducibility

The exact Task 7 single-obstacle request was measured twice in one process
with a fresh 31x31 `uint8` grid per request, `grid[10:12, 15] = 1`, 0.1 m
resolution, origin `(0.0, 0.0)`, goal `(1.7, 1.5)`, and the frozen stopped
initial state `(0.5, 1.5, 0, 0, 0, 0, elapsed=7.0, dwell=0.5)`.

| Run | Wall time (s) | Status | Expanded | Duration (s) | Waypoints | Controls | SHA-256 of `repr(result)` |
|---|---:|---|---:|---:|---:|---:|---|
| 1 | 6.794629 | success | 4677 | 5.0 | `(251, 7)` | `(5, 2)` | `a20937a68f75e49fe2d08aeaaaf69817899d8d6318757fc92fb9bf3167e4a353` |
| 2 | 6.635293 | success | 4677 | 5.0 | `(251, 7)` | `(5, 2)` | `a20937a68f75e49fe2d08aeaaaf69817899d8d6318757fc92fb9bf3167e4a353` |

The two complete `DynamicPlanResult` instances compare equal.  Their control
and waypoint matrices compare equal with `numpy.array_equal`; scalar fields
also match exactly: `status=success`, `expanded_nodes=4677`,
`path_length_m=1.0246359573463306`, `duration_s=5.0`,
`direction_changes=0`, and `total_cost=2.2065726886241666`.  The hash is
identical to the frozen Task 7 verification report.

Both runs remain above the required strictly-less-than-5.0-second limit.  The
cache removes the repeated steering-table reconstruction cost but does not
remove the remaining collision/cost edge-evaluation work, so Task 7B is still
required.

## Full regression

```text
PYTHONPATH=research/ball_wm /home/an/miniconda3/envs/ball_wm/bin/python -m pytest research/ball_wm/tests -q
# 581 passed, 3 warnings in 22.64s
```

The three warnings are pre-existing third-party deprecations reported through
wandb, sentry_sdk, and pkg_resources.

`git diff --check` was also run before commit.

## Review round 1: table-identity regression

The initial regression counted module-level `sorted` calls.  Review correctly
identified that this was coupled to a particular implementation detail: a
future regression could rebuild the same groups with `list.sort` or
`builtins.sorted` and still pass.  The test now wraps the real
`DriveModel._steering_groups` method, retains each returned direction-table
object from the actual `speed()` and `curvature()` calls, and asserts that the
two forward calls reuse one object, the two reverse calls reuse one different
object, and all frozen forward/reverse numeric interpolation values are
unchanged.

RED evidence: with only `_steering_groups` temporarily restored to its legacy
per-query filtering/sorting implementation, the new test failed at the
forward-table identity assertion because it received two distinct tables.
The approved fixed-table implementation was restored before final testing.

```text
PYTHONPATH=research/ball_wm /home/an/miniconda3/envs/ball_wm/bin/python -m pytest research/ball_wm/tests/test_drive_model.py research/ball_wm/tests/test_dynamic_lattice_planning.py -q
# 45 passed in 11.44s
```
