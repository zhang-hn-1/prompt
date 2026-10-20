# Dynamic Oracle V2 pure-code verification

## Status

**PASS** — all six cold-process fixed-request measurements completed strictly
below the required 5.0-second limit, and each pair's complete planning result
was exactly deterministic.

## Environment and commands

- Worktree: `/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark`
- Branch: `codex/e1-detour-benchmark`
- Drive model: `research/ball_wm/configs/benchmark_v1/drive_dynamics_v1.json`
- Full test command:
  `PYTHONPATH=research/ball_wm /home/an/miniconda3/envs/ball_wm/bin/python -m pytest research/ball_wm/tests -q`
- Performance timing: `time.perf_counter()` immediately around
  `plan_dynamic_lattice(request, model)`.

`git diff --check` completed without output.  The full suite completed with
**586 passed, 3 warnings in 20.67 s**.  The warnings are the existing
third-party `sentry_sdk.Hub` and `pkg_resources` deprecations reported through
wandb/setuptools.

## Fixed-request reproducibility and performance

All maps use a 31 by 31 empty `uint8` occupancy grid at 0.1 m resolution and
origin `(0.0, 0.0)`, except that the detour map sets `grid[10:12, 15] = 1`.
The open and detour starts are `(0.5, 1.5, 0 yaw, 0 main, 0 auxiliary, 0
speed, elapsed 7.0 s, stopped, dwell 0.5 s)`.  The reverse start is identical
except `elapsed=0.0` and `dwell=0.0`, so its first required control is the
stationary dwell before reverse becomes legal.

Each repetition ran in a separate fresh Python process.  That child created a
new `DriveModel` from the JSON profile immediately before timing, so neither a
previous `DriveModel` nor a previous process cache was reused.  The frozen
default 500,000-node / 60-second planner limits were retained.

| Fixed request | Goal | Status (run 1 / run 2) | Wall time s (run 1 / run 2) | Expanded | Plan duration s | Waypoint array | Control array | Exact equality | Result SHA-256 |
|---|---|---|---:|---:|---:|---:|---:|---|---|
| Open | `(1.0, 1.5)` | success / success | 0.511898 / 0.527357 | 237 | 3.0 | `(151, 7)` | `(3, 2)` | complete result equal | `a1d26fe5c58216865f6711d7176f9641d69977b3e3ccc25ac6a648dbf2c5b886` |
| Single-obstacle detour | `(1.7, 1.5)` | success / success | 4.796252 / 4.771158 | 4677 | 5.0 | `(251, 7)` | `(5, 2)` | complete result equal | `a20937a68f75e49fe2d08aeaaaf69817899d8d6318757fc92fb9bf3167e4a353` |
| Required reverse | `(0.0, 1.5)` | success / success | 0.633073 / 0.609976 | 348 | 4.0 | `(201, 7)` | `(4, 2)` | complete result equal | `a37472b0a3308a6b8878e0851e8eb617e5018c07e951830ace439328d92071e1` |

For each pair, the verifier compared all of the following with exact equality:

- status;
- the full `float64` waypoint byte sequence in field order `(x_m, y_m,
  yaw_rad, main_command_rad_s, auxiliary_angle_rad, speed_m_s,
  time_from_start_s)`;
- the full `float64` control byte sequence in `(main_command_rad_s,
  auxiliary_angle_rad)` order;
- `expanded_nodes`, `path_length_m`, `duration_s`, `direction_changes`, and
  `total_cost` through the complete frozen `DynamicPlanResult` representation.

The matching SHA-256 values are hashes of that complete `DynamicPlanResult`
representation.  They are unchanged from the pre-optimization plan results,
confirming that the performance changes did not alter the selected plans.
