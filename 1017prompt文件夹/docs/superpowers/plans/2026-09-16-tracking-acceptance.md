# Tracking Acceptance Implementation Plan

## 执行状态（2026-09-16）

以下保留最初任务拆解，实时证据与后续调整以 `.superpowers/sdd/2026-09-16-tracking-acceptance/ledger.md` 为准，不应依据未更新的逐项复选框重复实施。

- 几何参考、真实时钟/roll 记录、诊断 harness、测试及独立审查已完成。
- 开放平面 0.5/1.0/1.5 m 前视距离诊断已完成；所有跟踪验收门尚未通过，未替换正式默认参数。
- 显式调整了实验顺序：先保持规划器不变，以 1.0 m 参数做正式任务影响测试，而不是声称通过全部跟踪门后再进入障碍场景。真实 35 s 的 7 任务结果为 2 成功、4 栅格碰撞、1 超时，原有效对照为 0/7。
- 全局动态版本仅做单任务诊断，未替代正式基线；结果见 ledger。地图、安全裕度、PID、35 s 和成功标准均未放宽。

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在不改变正式 35 秒 benchmark、PID、地图或现有仿真适配器的前提下，独立验收球形机器人 `OraclePathPolicy` 的几何跟踪与停稳行为，并用同 case A/B 为后续 planner 收紧提供证据。

**Architecture:** 用 Isaac-free geometry module 生成现有 `KinodynamicWaypoint` 参考；薄 CLI 复用 `IsaacBenchmarkRuntime`、`run_episode`、`write_rgb_video` 和轨迹 renderer。runtime/runner 只补最小的 state-aligned World clock/roll 与默认关闭的 diagnostic settling，diagnostic budget 与 formal `35.0 s` 永远分开记录；总控按 case 启动独立子进程，避免在同一 Isaac 进程中重建 `SimulationApp`。

**Tech Stack:** Python 3、NumPy、pytest、现有 Isaac Sim 4.5 runtime、OpenCV MP4、JSON/NPZ、USD fixture。

**Spec:** `docs/superpowers/specs/2026-09-16-tracking-acceptance-design.md`

## Global Constraints

- 保持 action protocol `ball_action_v2_aux30`、每 block 5 个 `(main, auxiliary)` 控制和 `±30°` auxiliary bounds。
- 保持 main acceleration `0.5 rad/s²`、main `D=80`、auxiliary `Kp=300/Kd=60`、auxiliary target rate `30°/s`；不使用 experimental transient tau。
- 保持 `ball_radius=0.4 m`、`safety_margin=0.1 m`、goal tolerance `0.2 m`、formal episode budget `35.0 s`；诊断只使用独立命名的 `diagnostic_budget_s`。
- runtime 每个 action block 必须执行恰好 `5 * world.step(render=False)` 后调用一次 `world.render()`；渲染不能推进 physics clock。
- 每个 state 必须记录由 `world.current_time`/`world.current_time_step_index` 读取的 raw 值和减去 reset 基线后的 `simulation_time_s`/`physics_step_index`；reset 相对值为 `(0.0, 0)`，相邻 action 后 state 为 `(+0.1 s, +5)`。
- ground collider 保留在物理世界，但不进入 obstacle contact/collision 判定；root roll 只作诊断。open-plane USD 不得再定义 ground；显式 diagnostic open-plane 模式才允许空外部 collision paths，以复用唯一默认 ground。
- 使用 raw54 校准生成的 `drive_dynamics_v1.json`；不改 `hard_0` manifest/map，现有 dirty changes 必须保留且本任务不 commit。
- 不把 cross-track 直接称为安全裕量；不后验调 acceptance threshold；后续 Hybrid A* 收紧和 7-task 复测只作为 gate 依赖。

---

### Task 1: Add pure geometric references and the thin diagnostic harness

**Files:**
- Create: `research/ball_wm/benchmark/tracking_acceptance.py`
- Create: `research/ball_wm/scripts/run_tracking_acceptance.py`
- Create: `research/ball_wm/tests/test_tracking_acceptance.py`
- Create: `research/ball_wm/tests/test_tracking_acceptance_harness.py`
- Create if no equivalent scene exists: `research/ball_wm/data/benchmark_v1/scenes/tracking_open_plane.usda` (仅 Xform，无 ground/collider)
- Create if no equivalent occupancy exists: `research/ball_wm/data/benchmark_v1/occupancy/tracking_open_plane_free.npz`
- Create: `research/ball_wm/configs/benchmark_v1/tracking_acceptance.json`
- Modify: `research/ball_wm/benchmark/runner.py`
- Modify: `research/ball_wm/benchmark/isaac_runtime.py`
- Modify: `research/ball_wm/tests/test_benchmark_runner.py`
- Modify: `research/ball_wm/tests/test_isaac_runtime_contract.py`

**Interfaces:**
- `TrackingCase(case_id: str, kind: str, main_command_rad_s: float, budget_s: float, parameters: Mapping[str, float], probe: bool = False)` is an immutable case description.
- `TrackingSegment(segment_id: str, start_index: int, end_index: int, drive_direction: Literal[\"forward\", \"reverse\"], main_command_rad_s: float)` and `TrackingReference(case: TrackingCase, points_xy: np.ndarray, yaw_rad: np.ndarray, waypoints: tuple[KinodynamicWaypoint, ...], segments: tuple[TrackingSegment, ...], direction_boundaries: tuple[int, ...])` own dense finite geometry, actual planner-style cusp encoding, and separate stop/boundary metadata.
- `load_tracking_cases(path: str | Path) -> tuple[TrackingCase, ...]` reads the fixed eight-case JSON order; `build_tracking_reference(case: TrackingCase) -> TrackingReference` implements forward/reverse line, left/right radius-3 m arcs, radius-1.5 m probe, smooth S, and direction-sequence geometry.
- `polyline_cross_track_errors(points_xy: object, reference_xy: object) -> np.ndarray` computes point-to-segment distances; `heading_errors_rad(actual_yaw_rad: object, reference_yaw_rad: object) -> np.ndarray` returns normalized signed errors.
- `summarize_tracking_trace(...) -> dict[str, object]` returns descriptive cross-track/heading distributions, goal errors, and aligned speed/main/aux/roll traces only. It must not infer acceptance from arbitrary masks or reference indices. The harness alone verifies actual clock, post-goal zero-command settling, ordered cusp visits and real direction transitions. Reference yaw is explicitly reference-point-aligned, never inferred from accidental array-length equality.
- `RuntimeStep` adds relative `simulation_time_s: float`, `physics_step_index: int`, `roll_world_rad: float` and raw `raw_simulation_time_s: float`, `raw_physics_step_index: int`; `EpisodeRun` exposes state-aligned arrays with those names. `IsaacBenchmarkRuntime._capture()` supplies the raw fields directly from `world.current_time`/`world.current_time_step_index` after the reset baseline.
- The current `run_episode` call shape is exactly `run_episode(runtime, policy, episode, *, action_protocol, config, oracle_path_xy=None, kinodynamic_plan=None, dynamic_plan=None)`; do not document a nonexistent hold keyword. Extend `RunnerConfig` with default-off diagnostic fields and keep this `run_episode` signature. A positive diagnostic settling budget makes runner append zero-command blocks after first goal entry; formal default remains unchanged.
- `run_tracking_case(case: TrackingCase, *, harness_config: Mapping[str, object], output_root: str | Path, gui: bool, policy_variant: str = "baseline") -> dict[str, object]` creates the existing `IsaacBenchmarkRuntime`, injects `oracle_path_xy` and a `KinodynamicPlanResult` built from the geometry, writes the per-case artifacts, and closes the runtime.

- [ ] **Step 1: Write failing pure geometry tests (geometry owner)**

Add tests that assert the exact endpoint/yaw for a straight line, both left/right `r=3 m` 90° arcs, and reverse line; monotonic arc length and sample spacing `<=0.05 m`; smooth S curvature sign change; mixed-direction cusp encoding with no synthetic zero-direction marker and separate boundary metadata; and eight stable case IDs in the JSON config.

Add tests for `polyline_cross_track_errors()` using a point projected onto a segment (the expected value must not depend on nearest vertex), heading wrap at `±π`, P95/max calculation, and rejection of non-finite or malformed arrays.

```python
reference = build_tracking_reference(load_tracking_cases(config)[2])
assert reference.case.case_id == "arc_left_r3_90_0p5"
assert reference.points_xy[-1] == pytest.approx((3.0, 3.0), abs=1e-6)
assert reference.yaw_rad[-1] == pytest.approx(math.pi / 2.0, abs=1e-6)
assert max(np.linalg.norm(np.diff(reference.points_xy, axis=0), axis=1)) <= 0.05 + 1e-12
```

- [ ] **Step 2: Run CPU tests to verify the new module is absent**

Run: `PYTHONPATH=research/ball_wm pytest -q research/ball_wm/tests/test_tracking_acceptance.py`

Expected: FAIL during import of `benchmark.tracking_acceptance`.

- [ ] **Step 3: Implement the Isaac-free reference and metric module**

Keep all arrays finite, copied, and read-only where they cross the dataclass boundary. Generate geometry analytically, sample at or below `0.05 m`, compute tangent yaw, and map each sample to an existing `KinodynamicWaypoint`. Use `main_command_rad_s=0.0` only for the final stop marker; use `drive_direction="forward"` or `"reverse"` exactly as required by `OraclePathPolicy`. For the mixed-direction case, encode adjacent nonzero forward/reverse waypoints like the actual `KinodynamicPlanResult`; do not insert a same-coordinate zero waypoint or a synthetic `"stopped"` direction because `_project_to_reference()` takes the first equal-distance point and can deadlock at that cusp. Record stop/cusp requirements in `TrackingSegment`/`direction_boundaries` metadata and evaluate them from actual states and policy diagnostics.

Implement point-to-segment projection and normalized angle arithmetic in NumPy/Python only. The module must not import Isaac, `isaacsim`, `omni`, or a planner.

- [ ] **Step 4: Add the deterministic config and optional open-plane fixture (geometry/fixture owner)**

Write `tracking_acceptance.json` with schema `tracking_acceptance_v1`, fixed eight-case order, `goal_tolerance_m=0.2`, `stop_speed_m_s=0.05`, `stop_hold_s=0.5`, `settling_budget_s=3.0`, `replan_hz=10`, and explicit per-case `diagnostic_budget_s`: line forward 0.5 = 20.0 s, line forward 1.0 = 20.0 s, each radius-3 arc = 30.0 s, radius-1.5 probe = 30.0 s, S curve = 30.0 s, reverse line = 20.0 s, mixed direction = 35.0 s. Include `formal_budget_s=35.0` as metadata only. Mark `arc_left_r1p5_90_probe` with `probe=true`; required-case pass/fail must not silently absorb it.

If the shared scene has no reusable open plane, add a minimal USD with only an Xform referenced at `/World/Scene` (no ground and no collision), plus deterministic all-free `tracking_open_plane_free.npz` containing `grid` (uint8 zeros), `global_origin_xy`, and scalar `resolution_m` over all case extents. Extend `IsaacRuntimeConfig` with explicit `diagnostic_open_plane`; only that mode may accept zero external collision paths after `_load_collection_scene()` adds the sole default ground. Do not edit `hard_0` USD, manifest, or occupancy.

- [ ] **Step 5: Write failing clock/runner/harness tests (clock owner then harness owner)**

Extend the existing fake-world test so one `execute()` call observes `[False, False, False, False, False]`, exactly five physics ticks, exactly one render, raw `world.current_time`/`world.current_time_step_index` increasing by `0.1`/`5`, and relative `simulation_time_s=0.1`/`physics_step_index=5` after the initial `(0.0, 0)` reset state. Add a runner test asserting state arrays have `actions + 1` rows, `np.diff(physics_step_index)==5`, `np.diff(simulation_time_s)==0.1`, and `raw_simulation_time_s`/`raw_physics_step_index` equal the captured World values before baseline subtraction. Do not alter formal `elapsed_s`/`time_to_goal_s` semantics in this telemetry task; diagnostic reports may add actual clock duration separately. Add a settling test where the first goal state is still moving, zero-command settling lasts only until a later valid `.5 s` window, and no more than `3.0 s` is consumed.

Use a fake `RuntimeStep`/runtime to test harness serialization without Isaac: `states.npz` must contain the timing, pose/yaw/roll, velocity, joint, action, contact/collision and metric fields; `commands.npz` must contain block and 5x2 controls; config hash verification must fail if `config.json` changes.

- [ ] **Step 6: Implement the minimum runtime/runner plumbing (clock owner; no policy change)**

Reset the runtime clock baseline after warmup/prime and before the first scored `_capture()`. In every `_capture()`, read raw `self._world.current_time` and `self._world.current_time_step_index`, validate finite/nondecreasing values, subtract the saved reset values, and expose both raw and relative fields. Do not increment a local time counter, derive time from `action_blocks/replan_hz`, or infer roll/time from camera frames. Extract roll from the same normalized world quaternion already read by `_capture()`.

Keep the current `run_episode` keyword signature shown in Interfaces. Add to `RunnerConfig` default-off fields `diagnostic_settling_budget_s=0.0`, `diagnostic_stop_hold_s=0.5`, and `diagnostic_stop_speed_m_s=0.05`. With a positive settling budget, preserve the first goal entry as the formal `success` event but continue recording zero-command blocks for at most `3.0 s` (30 blocks at 10 Hz), stopping earlier only after five consecutive post-entry 10 Hz state endpoints spanning `.5 s` that each remain within goal `.2 m` and have planar speed `<.05 m/s`. The first goal state need not be stationary. Report `stop_hold_passed` independently; preserve collision precedence and existing formal termination names. The formal default remains `diagnostic_settling_budget_s=0.0`.

- [ ] **Step 7: Implement the thin CLI and artifact writer (harness owner)**

For one case per child process, construct a `BenchmarkEpisode` with the fixture scene hash, deterministic all-free occupancy NPZ hash, case start/goal pose, and geometry path length. Use the currently verified `run_episode` signature; configure diagnostic settling through `RunnerConfig` rather than adding an unverified keyword:

```python
result = run_episode(
    runtime,
    OraclePathPolicy(protocol, drive_model=DriveModel.from_json(profile)),
    episode,
    action_protocol=protocol,
    config=RunnerConfig(
        episode_budget_s=case.budget_s,
        replan_hz=10,
        goal_tolerance_m=0.2,
        diagnostic_settling_budget_s=3.0,
        diagnostic_stop_hold_s=0.5,
        diagnostic_stop_speed_m_s=0.05,
    ),
    oracle_path_xy=reference.points_xy,
    kinodynamic_plan=KinodynamicPlanResult(
        status="success",
        waypoints=reference.waypoints,
        expanded_nodes=0,
        path_length_m=float(
            np.linalg.norm(np.diff(reference.points_xy, axis=0), axis=1).sum()
        ),
        direction_changes=sum(
            left.drive_direction != right.drive_direction
            for left, right in zip(
                reference.waypoints, reference.waypoints[1:]
            )
            if left.main_command_rad_s != 0.0 and right.main_command_rad_s != 0.0
        ),
    ),
)
```

Use actual result positions/yaw and reference geometry for all metrics. Write `states.npz`, `commands.npz`, `trajectory.png`, and `third_person.mp4` with the exact fields in the design; write `config.json` canonically and `config.sha256` from its bytes. Include fixture/contract/profile hashes and both budget names. Reuse `runtime.captured_frames["third_person"]` and `write_rgb_video(..., fps=10.0)`; do not call `world.step()` from the writer.

- [ ] **Step 8: Run all first-stage CPU tests**

Run: `PYTHONPATH=research/ball_wm pytest -q research/ball_wm/tests/test_tracking_acceptance.py research/ball_wm/tests/test_tracking_acceptance_harness.py research/ball_wm/tests/test_benchmark_runner.py research/ball_wm/tests/test_isaac_runtime_contract.py`

Expected: PASS, with existing dirty tests preserved. A failure in the timing assertions blocks Isaac execution.

### Task 2: Execute the unmodified-policy diagnostic baseline

**Files:**
- Read: `research/ball_wm/configs/benchmark_v1/tracking_acceptance.json`
- Read: `research/ball_wm/configs/data_contract.yaml`
- Read: `research/ball_wm/configs/benchmark_v1/drive_dynamics_v1.json`
- Generate ignored outputs: `research/ball_wm/runs/tracking_acceptance_baseline/`
- Do not modify: `research/ball_wm/data/benchmark_v1/occupancy/cluttered_hard_0_bodyband.npz`, any `hard_0` scene/manifest, or policy/PID files

**Interfaces:**
- Consumes: Task 1 `run_tracking_acceptance.py`, fixed eight `TrackingCase`s, current `OraclePathPolicy`, accepted DriveModel and dedicated open-plane fixture.
- Produces: one complete per-case directory, `summary.json` with raw metrics and pass/fail, and a machine-readable failure stage (`planning`, `runtime`, `goal`, `stop_hold`, or `metric`).

- [ ] **Step 1: Run the harness in headless mode with the diagnostic budget**

Run:

```bash
PYTHONPATH=research/ball_wm /home/an/miniconda3/envs/ball_wm/bin/python \
  research/ball_wm/scripts/run_tracking_acceptance.py \
  --config research/ball_wm/configs/benchmark_v1/tracking_acceptance.json \
  --scene-root research/ball_wm/data/benchmark_v1/scenes \
  --run-root research/ball_wm/runs/tracking_acceptance_baseline \
  --headless --policy-variant baseline
```

The coordinator must launch one fresh child process per case; each child creates and closes exactly one `SimulationApp`/`IsaacBenchmarkRuntime`, retains all raw traces, and exits. Report diagnostic budget separately from formal 35 seconds. Do not run the 7-task benchmark as a substitute.

- [ ] **Step 2: Verify timing and artifact integrity before interpreting motion**

For every case, assert `states.npz["physics_step_index"][0] == 0`, `states.npz["simulation_time_s"][0] == 0.0`, `states.npz["raw_physics_step_index"]`/`states.npz["raw_simulation_time_s"]` equal the captured World values, state/action lengths differ by one, state step deltas are five per block, state time deltas are `0.1 s`, all arrays are finite, the video is non-empty, and `config.sha256` matches `config.json`. If any assertion fails, classify the case as `runtime` and stop; do not tune the tracker.

- [ ] **Step 3: Summarize baseline without threshold fitting**

Report each case’s cross-track mean/P95/max, heading mean/P95/max, final goal error, stop residual/hold result, actual main/aux traces, roll trace, collision/contact and termination. Keep the radius-1.5 m probe separate. Cross-track is evidence for a later margin decision only; it is not a safety claim.

### Task 3: Evidence-gated minimal fix and same-case A/B

**Files:**
- Modify only if Task 2 evidence identifies a concrete defect: `research/ball_wm/benchmark/policies.py` or the minimum directly involved tracking module.
- Modify corresponding tests: `research/ball_wm/tests/test_benchmark_policies.py` or `research/ball_wm/tests/test_tracking_acceptance.py`.
- Generate ignored outputs: `research/ball_wm/runs/tracking_acceptance_fix/` and `research/ball_wm/runs/tracking_acceptance_ab/`.
- Update only after evidence: `docs/superpowers/specs/2026-09-16-tracking-acceptance-design.md` with measured baseline/A/B facts.

**Interfaces:**
- Consumes: baseline per-case raw NPZ/MP4/metrics and the exact same config, fixture, seed, DriveModel, clock assertions and thresholds.
- Produces: either (a) one minimal tested policy change plus paired baseline/fix metrics, or (b) an explicit no-fix result when evidence does not isolate a change.

- [ ] **Step 1: Select one failure hypothesis from traces, without guessing PID**

Use the state/command alignment to identify the first divergence: reference index versus actual pose, main command versus actual main velocity, auxiliary target versus actual auxiliary angle, direction-change stop counter, goal braking, or roll/contact anomaly. Do not infer a cause from final distance alone. If the evidence only shows planner/reference mismatch, do not change the tracker in this task; record `no-fix` and leave Hybrid A* for the next phase.

- [ ] **Step 2: Add a regression test for the observed defect**

Write the smallest deterministic test that fails on the current policy and uses actual recorded values/geometry from the baseline (for example, a direction-boundary or lookahead case). Keep action protocol, DriveModel, PID and thresholds fixed in the test.

- [ ] **Step 3: Implement one minimal fix and run focused CPU tests**

Change only the code path named by Step 1. Run the new regression test plus `test_benchmark_policies.py`, `test_benchmark_runner.py`, `test_isaac_runtime_contract.py`, and the full tracking harness tests. If the change requires PID, threshold, map, or planner edits, stop and record that it is outside this first-stage plan.

- [ ] **Step 4: Re-run exact same cases for A/B**

Run baseline and fix in equivalent fresh processes with the same config, fixture, seeds and diagnostic budgets. Compare per-case success/hold status, cross-track distributions, heading error, goal error, stop residual, actual main/aux, roll, collision and runtime errors; additionally compare the left/right `r=3 m` pair under mirrored lateral sign for response symmetry. Preserve both raw output trees; do not select a better run post hoc or retune acceptance thresholds.

- [ ] **Step 5: Apply the tracking gate and hand off the dependency**

Mark the first-stage gate only when required cases pass goal `≤0.2 m` and the pre-fixed settling rule: first goal entry may still be moving, followed by at most `3.0 s` zero-command settling containing a valid continuous `.5 s` window with speed `<0.05 m/s` and goal error `≤0.2 m`; clock assertions pass, every mixed-direction boundary is actually reached and switched, no runtime/collision artifact error is present, and A/B improvement is evidenced without a required-case regression. The `1.5 m` case remains a separately labelled capability probe. If the gate fails, stop before planner tightening. If it passes, hand off the exact measured cross-track values and config/profile hashes for the later Hybrid A* range review; do not run or claim the 7-task formal 35-second re-evaluation here.

## Self-review checklist

- `TBD`, `TODO` and unbounded “tune until pass” instructions are absent; all thresholds are fixed above.
- No task modifies `hard_0`, formal 35-second defaults, PID, DriveModel source, or obstacle semantics.
- Every timing assertion uses runtime state clock, not wall time or action count alone.
- The only new simulator-facing code is a thin caller of existing `IsaacBenchmarkRuntime`; all geometry and metric logic is CPU-testable.
