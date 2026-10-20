# Ball Kinodynamic Oracle Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build and certify a privileged bidirectional kinodynamic Oracle that can execute the frozen seven-task `hard_0` PointGoal benchmark without changing the robot PID, action protocol, task poses, or formal map-free policy inputs.

**Architecture:** Extend the Isaac calibration runner to measure reverse steering/braking, export a compact versioned drive profile, and consume it through a pure-compute drive-model API. A bidirectional Hybrid A* plans collision-checked motion primitives over `(row, column, yaw_bin, drive_direction)`; the privileged Oracle tracks those primitives with an explicit stop-before-direction-change state machine. Runtime and artifact layers expose the kinodynamic reference only across the existing privilege boundary and preserve geometric A* as the SPL denominator.

**Tech Stack:** Python 3, NumPy, pytest, OpenCV, Isaac Sim/Isaac Lab runtime, JSON benchmark artifacts

**Spec:** `docs/superpowers/specs/2026-09-12-ball-kinodynamic-oracle-design.md`

## Global Constraints

- Keep action protocol `ball_action_v2_aux30`; execution outputs remain only `main_joint_velocity_cmd` and `auxiliary_joint_angle_cmd`.
- `drive_direction` is the sign of the planned main-axis command, not a physical gear, sensor value, or learned-model output.
- Keep the accepted main-axis acceleration limit `0.5 rad/s²`, auxiliary target-rate limit `30°/s`, existing PID gains, and auxiliary bound `±30°` unchanged.
- Keep `ball_radius=0.4 m`, `safety_margin=0.1 m`, occupancy resolution `0.1 m`, and collision sampling interval at most `0.05 m`.
- Keep frozen `hard_0` task start, goal, yaw, 20-second budget, collision rule, and goal tolerance unchanged.
- Keep geometric A* length as the SPL reference; the Hybrid A* reference is privileged audit/control information only.
- Do not add occupancy maps, A* paths, Hybrid A* paths, or motion primitives to `PolicyObservation`.
- Preserve uncommitted first-generation curvature-controller changes in `research/ball_wm/benchmark/policies.py` and `research/ball_wm/tests/test_benchmark_policies.py` until superseded deliberately by Task 5.

---

### Task 1: Complete reverse dynamics calibration and export

**Files:**
- Modify: `research/ball_wm/scripts/probe_drive_dynamics.py`
- Modify: `research/ball_wm/tests/test_probe_drive_dynamics.py`
- Create after measurement: `research/ball_wm/configs/benchmark_v1/drive_dynamics_v1.json`

**Interfaces:**
- Consumes: `calibration_cases_from_contract(contract) -> tuple[CalibrationCase, ...]` and existing Isaac trace summarization.
- Produces: `compact_drive_profile(report: Mapping[str, object]) -> dict[str, object]` with signed main-command steering samples and signed-direction braking samples.

- [x] **Step 1: Write failing matrix tests**

Add assertions that steady-turn cases cover the Cartesian product of main commands `(-1.5, -1.0, -0.5, 0.5, 1.0, 1.5)` and auxiliary commands `(-30, -20, -10, 10, 20, 30)°`, and braking cases cover all six signed main commands. Assert case IDs remain unique.

- [x] **Step 2: Run the focused tests and confirm the old 33-case matrix fails**

Run: `pytest -q research/ball_wm/tests/test_probe_drive_dynamics.py`

Expected: FAIL because reverse steady-turn and reverse braking cases are absent.

- [x] **Step 3: Expand the deterministic calibration matrix**

Set `TURN_MAIN_COMMANDS_RAD_S` and `BRAKING_MAIN_COMMANDS_RAD_S` to the six signed commands while leaving steering-step transients forward-only. Update the documented case count and plot grouping so forward and reverse curves are visibly distinct.

- [x] **Step 4: Write failing compact-profile tests**

Build a synthetic report containing all signed cases and assert:

```python
profile = compact_drive_profile(report)
assert profile["schema_version"] == "drive_dynamics_v1"
assert profile["action_protocol_id"] == "ball_action_v2_aux30"
assert {row["drive_direction"] for row in profile["steering_samples"]} == {
    "forward", "reverse"
}
assert all(row["speed_m_s"] >= 0.0 for row in profile["braking_samples"])
```

Also assert missing signed cases, non-finite measurements, duplicate operating points, and incompatible PID/action metadata raise `ValueError`.

- [x] **Step 5: Implement deterministic profile export**

Implement `compact_drive_profile()` and `write_compact_drive_profile()` so the tracked JSON contains schema/action identifiers, measurement timestamp, controller limits, PID metadata, steering rows with signed `main_command_rad_s`, `drive_direction`, auxiliary angle, curvature and radius, plus braking rows with direction, speed and stopping distance. Sort rows deterministically and reject incomplete reports instead of synthesizing values.

- [x] **Step 6: Run CPU tests**

Run: `pytest -q research/ball_wm/tests/test_probe_drive_dynamics.py`

Expected: all tests PASS.

- [x] **Step 7: Run Isaac calibration and export the measured profile**

Run the calibration script in headless mode against the same robot USD and accepted controller settings used by `drive_dynamics_calibration_20260912`. Save raw traces under a new ignored run directory and export the compact profile to `research/ball_wm/configs/benchmark_v1/drive_dynamics_v1.json`.

Expected: every signed operating point completes, the report contains no runtime errors, and the compact profile is generated only from measured rows.

- [x] **Step 8: Commit the calibration deliverable**

Commit only the script, tests, compact profile, updated spec terminology, and this plan; do not stage the existing `policies.py` experiment.

---

### Task 2: Add the versioned drive-model API

**Files:**
- Create: `research/ball_wm/benchmark/drive_model.py`
- Create: `research/ball_wm/tests/test_drive_model.py`

**Interfaces:**
- Consumes: `drive_dynamics_v1.json` from Task 1.
- Produces: `DriveModel.from_json(path: str | Path) -> DriveModel`, `curvature(main_command_rad_s: float, auxiliary_angle_rad: float) -> float`, `auxiliary_for_curvature(main_command_rad_s: float, desired_curvature_1_m: float) -> float`, and `stopping_distance(speed_m_s: float, drive_direction: str) -> float`.

- [x] **Step 1: Write failing schema and interpolation tests**

Test exact metadata validation, forward/reverse separation, zero auxiliary preserving its measured residual curvature, interpolation at measured points, bounded interpolation between adjacent points, auxiliary saturation at `±30°`, braking interpolation, conservative high-speed extrapolation, and rejection of `main_command_rad_s == 0` for steering lookup.

- [x] **Step 2: Run tests and confirm the module is missing**

Run: `pytest -q research/ball_wm/tests/test_drive_model.py`

Expected: FAIL during import of `benchmark.drive_model`.

- [x] **Step 3: Implement immutable validated profile loading**

Use frozen dataclasses for steering and braking samples. Validate sorted finite values, both directions, all three magnitudes, all seven auxiliary operating points including zero, compatible control metadata, and no duplicate `(main_command, auxiliary)` samples.

- [x] **Step 4: Implement direction-aware interpolation**

Interpolate within one signed main-command direction only. For curvature inversion, sort the measured curvature/auxiliary pairs for the selected signed speed and clamp the requested curvature to the measured attainable interval before `np.interp`.

- [x] **Step 5: Run focused and benchmark CPU tests**

Run: `pytest -q research/ball_wm/tests/test_drive_model.py research/ball_wm/tests/test_probe_drive_dynamics.py`

Expected: all tests PASS.

- [x] **Step 6: Commit the drive model**

Commit `drive_model.py` and its tests as an independently usable pure-compute component.

---

### Task 3: Implement bidirectional Hybrid A*

**Files:**
- Create: `research/ball_wm/benchmark/kinodynamic_planning.py`
- Create: `research/ball_wm/tests/test_kinodynamic_planning.py`

**Interfaces:**
- Consumes: `DriveModel.curvature()` from Task 2, inflated occupancy grid, map resolution/origin, start world pose, and goal world point.
- Produces: `plan_kinodynamic_path(request: KinodynamicPlanRequest, drive_model: DriveModel) -> KinodynamicPlanResult`; a successful result contains immutable `KinodynamicWaypoint(x_m, y_m, yaw_rad, drive_direction, main_command_rad_s, auxiliary_angle_rad)` entries and diagnostics.

- [x] **Step 1: Write failing geometry and primitive tests**

Test world/grid conversion, yaw normalization to 72 five-degree bins, forward straight integration, reverse straight integration, measured-sign curved integration, sample spacing `<=0.05 m`, and rejection when any sampled ball center intersects inflated occupancy.

- [x] **Step 2: Write failing search tests on small synthetic maps**

Cover unobstructed forward motion, an obstacle detour, a start pose that requires reverse motion, a path containing exactly one direction change, blocked start/goal, unreachable map, and deterministic termination at a small node budget.

- [x] **Step 3: Run tests and confirm the planner module is missing**

Run: `pytest -q research/ball_wm/tests/test_kinodynamic_planning.py`

Expected: FAIL during import of `benchmark.kinodynamic_planning`.

- [x] **Step 4: Implement motion primitives**

Integrate constant-curvature arcs of target length `0.3 m` for main commands `±0.5/±1.0/±1.5 rad/s` and auxiliary angles `-30/-20/-10/0/10/20/30°`. Store `drive_direction` from the main-command sign; no separate execution command is introduced.

- [x] **Step 5: Implement deterministic A* search and reconstruction**

Use state key `(row, column, yaw_bin, drive_direction)`, Euclidean heuristic, reverse cost multiplier `1.25`, direction-change cost `0.5 m`, stable heap tie breaking, `250_000` default expansion limit, goal radius `0.2 m`, and unconstrained goal yaw. Return explicit statuses `success`, `invalid_start`, `invalid_goal`, `unreachable`, or `node_budget_exhausted` with expansion count.

- [x] **Step 6: Run planner tests and full CPU benchmark suite**

Run: `pytest -q research/ball_wm/tests/test_kinodynamic_planning.py research/ball_wm/tests/test_benchmark_task_generation.py research/ball_wm/tests/test_benchmark_manifest.py`

Expected: all tests PASS.

- [x] **Step 7: Commit the planner**

Commit the pure NumPy planner and synthetic-map tests.

---

### Task 4: Preserve the privilege boundary while delivering the reference

**Files:**
- Modify: `research/ball_wm/benchmark/policies.py`
- Modify: `research/ball_wm/benchmark/runner.py`
- Modify: `research/ball_wm/benchmark/isaac_runtime.py`
- Modify: `research/ball_wm/tests/test_benchmark_policies.py`
- Modify: `research/ball_wm/tests/test_benchmark_runner.py`
- Create or modify: `research/ball_wm/tests/test_isaac_runtime_contract.py`

**Interfaces:**
- Consumes: `KinodynamicPlanResult` from Task 3.
- Produces: privileged `EpisodeContext.kinodynamic_reference`; runtime method `kinodynamic_reference(episode) -> tuple[KinodynamicWaypoint, ...]`; unchanged `PolicyObservation` fields.

- [x] **Step 1: Write failing privilege-boundary tests**

Assert non-privileged contexts reject the kinodynamic reference, privileged Oracle reset requires it, arrays/waypoints are immutable, and `PolicyObservation` still exposes only stereo RGB, proprio/action history, perfect position/yaw. Assert non-Oracle policies never receive reference data.

- [x] **Step 2: Run tests and confirm missing context support**

Run: `pytest -q research/ball_wm/tests/test_benchmark_policies.py research/ball_wm/tests/test_benchmark_runner.py`

Expected: FAIL on the new kinodynamic-reference assertions.

- [x] **Step 3: Extend the privileged context and runner plumbing**

Add the typed reference and planning diagnostics to `EpisodeContext`, obtain them from runtime only for privileged policies, and leave `PolicyObservation` unchanged. Retain geometric `oracle_path_xy` because it supplies the SPL reference and blue audit layer.

- [x] **Step 4: Plan during Isaac episode reset**

After loading and validating the inflated occupancy map and geometric A*, call `plan_kinodynamic_path()` using the episode start pose and goal. Cache the reference per reset, fail with an explicit planning status when no path exists, and do not mutate the manifest.

- [x] **Step 5: Run privilege and runtime-contract tests**

Run: `pytest -q research/ball_wm/tests/test_benchmark_policies.py research/ball_wm/tests/test_benchmark_runner.py research/ball_wm/tests/test_isaac_runtime_contract.py`

Expected: all tests PASS and the formal observation schema is byte-for-byte unchanged.

- [x] **Step 6: Commit context/runtime integration**

Commit the privilege-boundary changes separately from controller behavior.

---

### Task 5: Replace the failed curvature experiment with reference tracking

**Files:**
- Modify: `research/ball_wm/benchmark/policies.py`
- Modify: `research/ball_wm/tests/test_benchmark_policies.py`

**Interfaces:**
- Consumes: privileged kinodynamic waypoints and `DriveModel` APIs.
- Produces: `OraclePathPolicy.act()` action blocks plus read-only diagnostics containing reference index, cross-track error, desired/actual direction, braking state, and `failure_subreason`.

- [x] **Step 1: Write failing tracking-index and steering tests**

Assert projection index is monotonic, lookahead lies within `0.45–0.9 m`, signed main commands follow waypoint `drive_direction`, calibrated feedforward uses the same direction, yaw-rate feedback is bounded, curvature reduces speed, and every returned action satisfies `ball_action_v2_aux30`.

- [x] **Step 2: Write failing direction-change state-machine tests**

Exercise `forward -> braking -> stopped -> reverse`: a requested direction reversal must command zero main speed until measured planar speed is below `0.05 m/s` for five consecutive 10 Hz cycles. Verify speed rebound resets the counter and direction hysteresis prevents oscillation.

- [x] **Step 3: Write failing infeasible-tracking tests**

Assert cross-track distance above `1.0 m` for ten cycles latches a safe stop and reports `tracking_infeasible`; shorter excursions recover. Top-level termination remains `timeout`.

- [x] **Step 4: Run tests and verify expected failures**

Run: `pytest -q research/ball_wm/tests/test_benchmark_policies.py`

Expected: FAIL because the current controller tracks only an XY path and has no direction-change state machine.

- [x] **Step 5: Implement the tracker and retire embedded guessed tables**

Load the versioned `DriveModel`, follow waypoint actions with speed-adaptive lookahead, use bounded yaw-rate correction, invoke calibrated stopping distance before bends/goals/direction changes, and keep all outputs behind the existing `ActionSmoother`. Remove the hard-coded forward-only calibration constants introduced by the failed curvature experiment once equivalent profile-backed behavior is tested.

- [x] **Step 6: Run policy and action-protocol regression tests**

Run: `pytest -q research/ball_wm/tests/test_benchmark_policies.py research/ball_wm/tests/test_action_smoothing.py research/ball_wm/tests/test_action_protocol.py`

Expected: all tests PASS.

- [x] **Step 7: Commit the tracker**

Commit the policy replacement and record `benchmark_hard_0_oracle_curvature_v1` as the retained failed comparison, not the certified baseline.

---

### Task 6: Add certification diagnostics and three-layer audit artifacts

**Files:**
- Modify: `research/ball_wm/benchmark/artifacts.py`
- Modify: `research/ball_wm/scripts/run_pointgoal_benchmark.py`
- Modify: `research/ball_wm/scripts/trajectory_map.py`
- Modify: `research/ball_wm/tests/test_benchmark_artifacts.py`
- Modify: `research/ball_wm/tests/test_trajectory_map.py`

**Interfaces:**
- Consumes: geometric A* path, kinodynamic reference, actual trajectory, planning status, and policy diagnostics.
- Produces: validated per-episode JSON/NPZ and overlay with blue geometric A*, purple kinodynamic reference, and orange actual trajectory.

- [x] **Step 1: Write failing artifact and rendering tests**

Require `kinodynamic_reference` in privileged audit bundles, reject incomplete/modified layers, and use a small known image to verify all three path colors plus start/goal markers. Assert certification JSON includes planning status, node expansions, direction-change count, minimum clearance, maximum/mean cross-track error, and optional `failure_subreason`.

- [x] **Step 2: Run tests and verify missing artifact support**

Run: `pytest -q research/ball_wm/tests/test_benchmark_artifacts.py research/ball_wm/tests/test_trajectory_map.py`

Expected: FAIL on the new required layer and diagnostic fields.

- [x] **Step 3: Implement serialization and visualization**

Serialize waypoint pose/direction/commands without object arrays, render the purple reference independently of blue A*, and retain atomic artifact writes plus SHA-256 protection. Keep formal success/SPL calculations unchanged.

- [x] **Step 4: Run artifact, runner, and rendering regression tests**

Run: `pytest -q research/ball_wm/tests/test_benchmark_artifacts.py research/ball_wm/tests/test_trajectory_map.py research/ball_wm/tests/test_benchmark_runner.py`

Expected: all tests PASS.

- [x] **Step 5: Commit certification artifacts**

Commit the audit schema and renderer changes.

---

### Task 7: Certify the frozen seven-task benchmark

**Files:**
- Do not modify: frozen `hard_0` manifest and task poses.
- Generate ignored outputs: `research/ball_wm/runs/benchmark_hard_0_kinodynamic_oracle_v1/`
- Update after evidence exists: `docs/superpowers/specs/2026-09-08-ball-pointgoal-benchmark-v1-design.md`

**Interfaces:**
- Consumes: integrated planner, tracker, runtime, and artifact pipeline.
- Produces: seven complete episode audit bundles, aggregate summary, one GUI-reviewed E1 recording, and an evidence-backed certification decision.

- [x] **Step 1: Run the complete CPU regression suite**

Run: `pytest -q research/ball_wm/tests`

Expected: all tests PASS before launching Isaac.

- [x] **Step 2: Run a one-episode headless smoke test**

Use one E0 task and verify simulator initialization, plan generation, action execution, clean shutdown, and a complete artifact bundle.

- [ ] **Step 3: Run each frozen episode in an isolated Isaac process**

Execute all seven tasks with fixed seeds and the existing 20-second budget, one process per episode to avoid the previously observed repeated-startup segmentation fault contaminating later episodes. Aggregate only successfully finalized, hash-valid bundles.

- [ ] **Step 4: Evaluate the certification gate**

Require E0 `3/3`, E1 `4/4`, total collisions `0`, runtime errors `0`, and complete required videos/plots. A failed physical episode remains evidence; do not alter task pose or threshold. If planning exhausts 250,000 nodes, record `kinematically_infeasible` and propose a new manifest version as a separate major decision.

- [ ] **Step 5: Reproduce one E1 episode in GUI mode**

Display robot, obstacles, start/goal, blue geometric path, purple kinodynamic reference, and orange executed path. Confirm visually that the robot performs an actual detour and any reverse/forward transition stops before changing command sign.

- [ ] **Step 6: Document measured results**

Add the exact run directory, aggregate metrics, failure breakdown, calibration profile hash, and human GUI review outcome to the benchmark design document. State clearly whether the Oracle is certified; do not claim certification unless every gate passes.

- [ ] **Step 7: Run verification and commit the evidence summary**

Re-run the focused benchmark/artifact tests and verify every finalized audit bundle hash before committing the documentation update.
