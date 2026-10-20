# Delay-Aware Trajectory Tracker V2 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build and gate an independent `oracle_trajectory_v2` policy that blends the frozen Pure Pursuit stabilizer with a locally phase-compensated curvature feedforward while staying inside calibrated drive regimes.

**Architecture:** Add pure local-curvature query/blending helpers and immutable diagnostics to the existing spatial-reference module, then add a new policy without changing either legacy policy. Freeze a 1.0 s / 0.30 m phase-lead hypothesis before GPU work, validate reverse signs separately, and expose V2 to the formal E1 runner only if the frozen six-case gate passes.

**Tech Stack:** Python 3.11, NumPy, pytest, Isaac Sim GPU runtime, existing `ActionProtocol`, `DriveModel`, `SpatialReferenceProfile`, and benchmark artifact validators.

**Spec:** `docs/superpowers/specs/2026-09-17-delay-aware-trajectory-tracker-v2-design.md`

## Global Constraints

- Do not modify Hybrid A*, occupancy maps, task poses, action protocol, joint PD, 0.2 m goal tolerance, 0.05 m/s stop threshold, 0.5 s hold, or 35 s formal budget.
- Freeze `OraclePathPolicy`, `TrajectoryOraclePolicy`, and all V1 artifacts.
- `oracle_trajectory_v2` consumes the existing privileged `kinodynamic_reference`; it must reject `dynamic_reference` and must not expose new inputs through `PolicyObservation`.
- Delay tuning uses only open-plane cases; do not tune from E1 outcomes.
- Formal E1 dispatch is forbidden unless the full open-plane gate passes.
- Preserve deterministic reset and the existing single runner loop.

---

### Task 1: Pure local phase-lead and curvature-blending math

**Files:**
- Modify: `research/ball_wm/benchmark/trajectory_tracking.py`
- Modify: `research/ball_wm/tests/test_trajectory_tracking.py`

**Interfaces:**
- Consumes: `SpatialReferenceProfile.s_m`, `segment_id`, `curvature_1_m`, and current segment/index.
- Produces: `DelayAwareReferenceQuery(index: int, projection_s_m: float, delay_distance_m: float, query_s_m: float, curvature_1_m: float)`; `query_delay_aware_reference(...)`; `curvature_blend_weight(...) -> float`; `blend_tracking_curvature(...) -> float`.

- [ ] **Step 1: Write failing tests for same-segment interpolation and cusp clipping**

```python
def test_delay_query_interpolates_and_never_crosses_direction_cusp():
    profile = make_forward_then_reverse_profile()
    query = query_delay_aware_reference(
        profile,
        current_index=1,
        speed_m_s=0.4,
        auxiliary_latency_s=1.0,
        profile_window_m=0.30,
        maximum_delay_distance_m=0.30,
    )
    assert query.delay_distance_m == pytest.approx(0.30)
    assert query.index <= last_forward_index(profile)
    assert query.query_s_m <= profile.s_m[last_forward_index(profile)]
```

- [ ] **Step 2: Run the focused test and verify RED**

Run: `pytest -q research/ball_wm/tests/test_trajectory_tracking.py -k 'delay_query'`

Expected: FAIL because `query_delay_aware_reference` and `DelayAwareReferenceQuery` do not exist.

- [ ] **Step 3: Implement immutable query result and bounded interpolation**

```python
@dataclass(frozen=True)
class DelayAwareReferenceQuery:
    index: int
    projection_s_m: float
    delay_distance_m: float
    query_s_m: float
    curvature_1_m: float

def query_delay_aware_reference(
    profile: SpatialReferenceProfile,
    *,
    current_index: int,
    speed_m_s: float,
    auxiliary_latency_s: float,
    profile_window_m: float,
    maximum_delay_distance_m: float,
) -> DelayAwareReferenceQuery:
    values = np.asarray(
        [speed_m_s, auxiliary_latency_s, profile_window_m, maximum_delay_distance_m],
        dtype=np.float64,
    )
    if not np.all(np.isfinite(values)) or np.any(values[1:] < 0.0):
        raise ValueError("delay-query inputs must be finite and nonnegative")
    segment = int(profile.segment_id[current_index])
    same_segment = np.flatnonzero(profile.segment_id == segment)
    segment_end = int(same_segment[-1])
    projection_s = float(profile.s_m[current_index])
    delay_distance = min(abs(float(speed_m_s)) * auxiliary_latency_s,
                         maximum_delay_distance_m)
    query_s = min(projection_s + delay_distance,
                  float(profile.s_m[segment_end]))
    local_s = profile.s_m[current_index:segment_end + 1]
    local_curvature = profile.curvature_1_m[current_index:segment_end + 1]
    half_window = 0.5 * profile_window_m
    support = (local_s >= query_s - half_window) & (local_s <= query_s + half_window)
    curvature = float(
        np.mean(local_curvature[support])
        if np.any(support)
        else np.interp(query_s, local_s, local_curvature)
    )
    index = current_index + int(np.searchsorted(local_s, query_s, side="left"))
    index = min(index, segment_end)
    return DelayAwareReferenceQuery(index, projection_s, delay_distance,
                                    query_s, curvature)
```

- [ ] **Step 4: Write failing tests for curvature blending**

```python
@pytest.mark.parametrize(
    ("profile_curvature", "expected_weight"),
    [(0.0, 0.0), (0.05, 0.0), (0.125, 0.5), (0.20, 1.0), (-0.20, 1.0)],
)
def test_curvature_blend_weight_is_symmetric_and_bounded(
    profile_curvature, expected_weight
):
    assert curvature_blend_weight(
        profile_curvature,
        blend_start_1_m=0.05,
        blend_full_1_m=0.20,
    ) == pytest.approx(expected_weight)
```

Also assert finite validation and that zero profile curvature returns the Pure Pursuit curvature exactly, while full blend returns `profile + feedback` exactly.

- [ ] **Step 5: Implement the minimal pure blend**

```python
def curvature_blend_weight(curvature_1_m, *, blend_start_1_m, blend_full_1_m):
    magnitude = abs(_finite_float(curvature_1_m, name="curvature_1_m"))
    if blend_full_1_m <= blend_start_1_m:
        raise ValueError("blend_full_1_m must exceed blend_start_1_m")
    return float(np.clip(
        (magnitude - blend_start_1_m) / (blend_full_1_m - blend_start_1_m),
        0.0,
        1.0,
    ))

def blend_tracking_curvature(pure_pursuit, profile, feedback, *, weight):
    values = np.asarray([pure_pursuit, profile, feedback, weight], dtype=np.float64)
    if not np.all(np.isfinite(values)) or not 0.0 <= weight <= 1.0:
        raise ValueError("curvature blend inputs must be finite and bounded")
    return float((1.0 - weight) * pure_pursuit + weight * (profile + feedback))
```

Do not add a V2 speed-envelope helper in this task. The first V2 experiment uses only the calibrated 0.50 rad/s moving command; remaining-time feasibility is diagnostic-only.

- [ ] **Step 6: Run focused and module tests**

Run: `pytest -q research/ball_wm/tests/test_trajectory_tracking.py`

Expected: PASS.

- [ ] **Step 7: Commit the pure numerical layer**

```bash
git add research/ball_wm/benchmark/trajectory_tracking.py research/ball_wm/tests/test_trajectory_tracking.py
git commit -m "feat: add delay-aware trajectory reference math"
```

### Task 2: Independent `oracle_trajectory_v2` policy

**Files:**
- Modify: `research/ball_wm/benchmark/policies.py`
- Modify: `research/ball_wm/tests/test_benchmark_policies.py`

**Interfaces:**
- Consumes: Task 1 local-query/blending helpers, existing `OraclePathPolicy` geometry formulas copied without shared mutable state, `DriveModel`, `EpisodeContext.kinodynamic_reference`, position/yaw/proprioception, and the fixed episode budget.
- Produces: `DelayAwareTrajectoryOraclePolicy`, `policy_name = "oracle_trajectory_v2"`, and finite diagnostics keys `projection_s_m`, `delay_distance_m`, `query_s_m`, `pure_pursuit_curvature_1_m`, `profile_curvature_1_m`, `feedback_curvature_1_m`, `blend_weight`, `command_curvature_1_m`, `budget_infeasible`, and terminal timing events.

- [ ] **Step 1: Write RED contract tests**

```python
def test_v2_is_independent_and_rejects_dynamic_reference(protocol, context):
    policy = DelayAwareTrajectoryOraclePolicy(protocol)
    assert policy.policy_name == "oracle_trajectory_v2"
    assert not isinstance(policy, TrajectoryOraclePolicy)
    with pytest.raises(ValueError, match="dynamic_reference"):
        policy.reset(replace(context, dynamic_reference=make_dynamic_reference()))
```

Add tests for deterministic reset, forward/reverse curvature sign, delay query not crossing a cusp, zero action on goal, five-cycle stop-before-direction-change, and unchanged legacy-policy golden actions.

- [ ] **Step 2: Verify RED**

Run: `pytest -q research/ball_wm/tests/test_benchmark_policies.py -k 'trajectory_v2 or legacy_policy_golden'`

Expected: FAIL only on missing V2 symbols/behavior; legacy golden test remains PASS.

- [ ] **Step 3: Implement the minimal V2 class**

Create a separate class rather than subclassing either legacy tracker. Reuse validation utilities and immutable profile construction, but own all mutable tracker state. Default preregistered parameters:

```python
auxiliary_latency_s = 1.0
maximum_delay_distance_m = 0.30
profile_window_m = 0.30
profile_blend_start_1_m = 0.05
profile_blend_full_1_m = 0.20
feedback_lookahead_m = 1.0
maximum_feedback_curvature_1_m = 0.20
supported_main_command_rad_s = 0.50
episode_budget_s = 35.0
```

Compute the frozen Pure Pursuit curvature, local profile curvature, bounded feedback, and blend weight separately. Map the blended curvature through `DriveModel` at exactly ±0.50 rad/s. Fail closed if a moving command is not a calibrated magnitude. Preserve the existing stop-before-direction-change and goal-braking logic, and record terminal brake/entry/stop/hold times.

- [ ] **Step 4: Add fail-closed diagnostic validation**

All V2 diagnostic scalars must be finite, query arc length must remain within the current segment, and invalid reference/parameters must raise before returning an action.

- [ ] **Step 5: Run policy and regression tests**

Run: `pytest -q research/ball_wm/tests/test_benchmark_policies.py research/ball_wm/tests/test_trajectory_tracking.py`

Expected: PASS with unchanged V1/baseline tests.

- [ ] **Step 6: Commit V2 policy**

```bash
git add research/ball_wm/benchmark/policies.py research/ball_wm/tests/test_benchmark_policies.py
git commit -m "feat: add delay-aware trajectory tracker v2"
```

### Task 3: Tracking-acceptance harness and provenance

**Files:**
- Modify: `research/ball_wm/scripts/run_tracking_acceptance.py`
- Modify: `research/ball_wm/configs/benchmark_v1/tracking_acceptance.json`
- Modify: `research/ball_wm/tests/test_tracking_acceptance_harness.py`
- Modify: `research/ball_wm/tests/test_tracking_acceptance.py`

**Interfaces:**
- Consumes: `DelayAwareTrajectoryOraclePolicy` and its constructor parameters.
- Produces: CLI `--policy-variant oracle_trajectory_v2`; V2 config/provenance in child commands, `config.json`, `metrics.json`, and `summary.json`; unchanged default `baseline`.

- [ ] **Step 1: Add RED CLI/child-command tests**

Assert exact propagation of `auxiliary_latency_s`, `maximum_delay_distance_m`, `profile_window_m`, blend thresholds, feedback limit, calibrated main command, and episode budget; assert malformed/missing V2 diagnostics fail closed.

- [ ] **Step 2: Verify RED**

Run: `pytest -q research/ball_wm/tests/test_tracking_acceptance_harness.py research/ball_wm/tests/test_tracking_acceptance.py -k trajectory_v2`

Expected: FAIL because V2 is not a valid harness policy variant.

- [ ] **Step 3: Implement V2 dispatch and provenance**

Add a distinct constant `TRAJECTORY_POLICY_V2_VARIANT = "oracle_trajectory_v2"`; do not rename V1. Ensure one-case child commands carry every V2 parameter and that hashes bind the selected configuration and relevant source files.

- [ ] **Step 4: Run integration tests and the existing focused suite**

Run: `pytest -q research/ball_wm/tests/test_tracking_acceptance_harness.py research/ball_wm/tests/test_tracking_acceptance.py research/ball_wm/tests/test_benchmark_policies.py research/ball_wm/tests/test_trajectory_tracking.py`

Expected: PASS.

- [ ] **Step 5: Commit harness integration**

```bash
git add research/ball_wm/scripts/run_tracking_acceptance.py research/ball_wm/configs/benchmark_v1/tracking_acceptance.json research/ball_wm/tests/test_tracking_acceptance_harness.py research/ball_wm/tests/test_tracking_acceptance.py
git commit -m "test: integrate trajectory tracker v2 acceptance"
```

### Task 4: Frozen reverse-sign and minimal GPU probe

**Files:**
- Create ignored artifacts under: `research/ball_wm/runs/tracking_v2_minimal_probe_20260917/`
- Create review record: `.superpowers/sdd/2026-09-17-delay-aware-trajectory-tracker-v2/task-4-minimal-probe-report.md`

**Interfaces:**
- Consumes: frozen Task 3 source/config hashes and the existing deterministic open-plane fixtures.
- Produces: a pass/fail decision for the preregistered `auxiliary_latency_s=1.0`, `maximum_delay_distance_m=0.30` configuration using `line_forward_0p5`, `line_reverse_0p5`, and `s_curve_0p5`.

- [ ] **Step 1: Record source/input hashes and verify GPU visibility**

Run `nvidia-smi`, record the exact git revision, dirty diff hash, fixture hashes, drive-profile hash, and V2 configuration before launching Isaac.

- [ ] **Step 2: Verify reverse signs on CPU before Isaac**

Construct positive and negative lateral/yaw perturbations for forward and reverse motion. Assert that the V2 correction reduces the signed error under the same kinematic convention used by the frozen baseline. Assert `blend_weight == 0` on both straight references.

- [ ] **Step 3: Run three fresh GPU processes**

Run forward line, reverse line, and S curve once with seed 3072, no retry, headless GPU mode, video enabled, and existing diagnostic budgets. Reject V2 if either straight case worsens maximum cross-track by more than 0.02 m, if S fails arrival, if lead distance leaves 0–0.30 m, or if any moving main command differs from ±0.50 rad/s after ramp-up.

- [ ] **Step 4: Persist the evidence report**

The report must list all three exit codes, state-0 equality, arrival/hold, cross-track, roll, speed, lead-distance range, blend weights, moving command magnitudes, terminal timing, configuration/hash provenance, and rejection reasons. A missing or invalid run blocks Task 5; no parameter retuning is permitted inside this probe.

### Task 5: Frozen six-case open-plane gate

**Files:**
- Create ignored artifacts under: `research/ball_wm/runs/tracking_v2_gate_20260917/`
- Create review record: `.superpowers/sdd/2026-09-17-delay-aware-trajectory-tracker-v2/task-5-gate-report.md`

**Interfaces:**
- Consumes: the exact preregistered V2 configuration accepted by Task 4.
- Produces: `open_plane_gate_pass: bool` and `enter_formal_e1: bool` using the spec's fixed criteria.

- [ ] **Step 1: Freeze and hash baseline/V1/V2 inputs**

Record exact source, config, fixture, occupancy, contract, and drive-profile hashes. The runner must reject drift before launch.

- [ ] **Step 2: Run baseline and V2 in matched fresh processes**

Run the six cases: forward line, left R3, right R3, S curve, reverse line, and mixed forward-stop-reverse-stop-forward. Use the same seed, budgets, record-video setting, and deterministic reset contract as V1.

- [ ] **Step 3: Evaluate the fixed gate without manual exceptions**

Require straight/reverse maximum-error worsening no greater than 0.02 m; at least two of three curved cases improve maximum cross-track by 20% without new arrival/hold failure; mixed completes two direction changes, arrival, and final hold; all runs have no collision/runtime/clock error.

- [ ] **Step 4: Stop or proceed**

If false, write a bounded failure report and do not change formal CLI. If true, set `enter_formal_e1=true` and continue to Task 6.

### Task 6: Conditional formal E1 integration and evaluation

**Files:**
- Modify only if Task 5 passes: `research/ball_wm/scripts/run_pointgoal_benchmark.py`
- Modify only if Task 5 passes: `research/ball_wm/tests/test_benchmark_artifacts.py`
- Create ignored artifacts only if Task 5 passes: `research/ball_wm/runs/benchmark_hard_0_trajectory_v2_35s/`

**Interfaces:**
- Consumes: gate-passing frozen V2 configuration.
- Produces: formal policy choice `oracle_trajectory_v2`, complete provenance, and results on exactly four frozen E1 episodes.

- [ ] **Step 1: Write RED formal dispatch/provenance tests after gate PASS**

Assert V2 receives `kinodynamic_reference`, cannot access `dynamic_reference`, serializes all V2 parameters, and leaves baseline dispatch unchanged.

- [ ] **Step 2: Implement minimal formal dispatch**

Add only the new policy choice and parameter plumbing. Do not change formal runner semantics, tasks, map, planner, budget, or metrics.

- [ ] **Step 3: Run CPU regression and verification**

Run the focused benchmark, runner, policy, artifact, acceptance, and trajectory tests; run `py_compile` on changed production files and `git diff --check`.

- [ ] **Step 4: Run the four frozen E1 episodes once**

Use 35 s, no retries, fresh Isaac process per episode, seed from the frozen manifest, GPU headless mode, and video/artifact recording.

- [ ] **Step 5: Report without post-hoc tuning**

Compare V2 against the old 2/4 (50%) Oracle baseline on success, collision, SPL, minimum clearance, maximum/mean cross-track, time-to-goal, and termination reason. Do not tune V2 after observing E1.
