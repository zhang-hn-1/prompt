# Dynamic Oracle V2 Guided Lattice Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make privileged `oracle_v2` find a deterministic dynamics-feasible E1 route by using its frozen static A* route as a soft search priority, without exposing global-map information to the local navigation model.

**Architecture:** Runtime preserves the deterministic A* cells already derived from the frozen inflated map and passes them only to `DynamicPlanRequest`. The planner derives an EDT-backed remaining-route field and uses it only for heap ordering; state cost, collision checking, rate limits, result schema, 32 s horizon and 60 s timeout remain unchanged.

**Tech Stack:** Python 3.10, NumPy, SciPy `distance_transform_edt`, pytest, Isaac Sim 4.5.

**Spec:** `docs/superpowers/specs/2026-09-14-dynamic-oracle-v2-guided-lattice-design.md`

## Global Constraints

- Scope is privileged `oracle_v2` only. Never expose guidance, occupancy or distance fields through `PolicyObservation`, training data or world-model inputs.
- Use `guidance_weight = 0.35` and `guidance_mode = "static_astar_soft_priority"` exactly.
- Do not corridor-prune nodes. Preserve all 50 Hz collision, risk, stopped-reversal and dynamics checks.
- Preserve accumulated cost, `DynamicPlanResult.total_cost`, controls and result schema. Guidance changes heap priority only.
- No-guidance planning remains byte-for-byte legacy behavior. V1 `oracle` remains unchanged.
- Preserve 1.0 s primitive, 0.02 s integration, main limit `0.5 rad/s²`, auxiliary limit `30°/s`, 32 s trajectory, 500,000 nodes, 60 s planner and 35 s episode budgets.
- No V1/GoalGreedy/straight-line fallback, and no changes to frozen maps, manifest, safety margin or success criteria.

---

## File map

| File | Responsibility |
|---|---|
| `research/ball_wm/benchmark/dynamic_lattice_planning.py` | Validate guidance cells, derive priority field, select guided or legacy priority. |
| `research/ball_wm/benchmark/isaac_runtime.py` | Preserve static A* cells and pass them only to dynamic Oracle planning. |
| `research/ball_wm/benchmark/artifacts.py` | Validate guided V2 audit metadata without affecting V1. |
| `research/ball_wm/scripts/run_pointgoal_benchmark.py` | Write guided planner metadata into environment and certification. |
| `research/ball_wm/scripts/preflight_dynamic_oracle.py` | Run hash-verified frozen-map planning without Isaac. |
| `research/ball_wm/tests/test_dynamic_lattice_planning.py` | Guidance validation, soft priority, determinism and legacy regression. |
| `research/ball_wm/tests/test_isaac_runtime_contract.py` | Runtime guidance isolation contract. |
| `research/ball_wm/tests/test_benchmark_artifacts.py` | Guided V2 metadata publishing contract. |
| `research/ball_wm/tests/test_preflight_dynamic_oracle.py` | No-Isaac preflight contract. |

---

### Task 1: Guidance request contract and field

**Files:**
- Modify: `research/ball_wm/benchmark/dynamic_lattice_planning.py`
- Modify: `research/ball_wm/tests/test_dynamic_lattice_planning.py`

**Interfaces:**
- Produce `DynamicPlanRequest(guidance_cells: np.ndarray | None = None, guidance_weight: float = 0.35)`.
- Produce `_guided_remaining_field(guidance_cells, grid_shape, resolution_m) -> np.ndarray`.

- [ ] **Step 1: Write failing request tests**

```python
def test_guidance_cells_are_frozen_free_integer_path_ending_at_goal():
    request = _request(guidance_cells=np.asarray(((1, 1), (1, 2), (2, 2)), dtype=np.int64))
    assert request.guidance_cells is not None
    assert not request.guidance_cells.flags.writeable

@pytest.mark.parametrize("bad", [np.asarray((1, 2)), np.asarray(((1.5, 1.0),)), np.asarray(((99, 99)))])
def test_guidance_cells_reject_invalid_arrays(bad):
    with pytest.raises(ValueError):
        _request(guidance_cells=bad)
```

Add occupied-cell, wrong-final-goal-cell and non-finite/non-positive-weight cases. Run the dynamic planner tests and verify they fail because the request has no guidance contract.

- [ ] **Step 2: Implement the contract and field**

Validate shape `(K, 2)`, integer dtype, `K >= 1`, bounds, free cells, final cell equal to `world_to_cell(goal_xy, ...)`, and finite positive weight; copy and mark the cells read-only. Implement the field with a Boolean path mask, `distance_transform_edt(~mask, return_indices=True)`, reverse cumulative 8-neighbor cell lengths, and nearest-path indexing. Keep `None` as explicit legacy mode.

- [ ] **Step 3: Run focused verification**

```bash
PYTHONPATH=research/ball_wm /home/an/miniconda3/envs/ball_wm/bin/python -m pytest research/ball_wm/tests/test_dynamic_lattice_planning.py -q
```

Verify no-guidance request construction remains valid and frozen arrays cannot be mutated.

- [ ] **Step 4: Commit**

```bash
git add research/ball_wm/benchmark/dynamic_lattice_planning.py research/ball_wm/tests/test_dynamic_lattice_planning.py
git commit -m "feat: validate dynamic lattice guidance paths"
```

### Task 2: Guided soft-priority search

**Files:**
- Modify: `research/ball_wm/benchmark/dynamic_lattice_planning.py`
- Modify: `research/ball_wm/tests/test_dynamic_lattice_planning.py`

**Interfaces:**
- Consume Task 1 guidance cells.
- Produce guided heap priority `cost + remaining + 0.35 * lateral_distance`; legacy priority remains `cost + _grid_distance_to_goal(...)`.

- [ ] **Step 1: Write failing guided-search tests**

```python
def test_guided_search_does_not_build_legacy_full_grid_distance(monkeypatch):
    request = _request(guidance_cells=_detour_reference_cells())
    monkeypatch.setattr(dynamic, "_grid_distance_to_goal", lambda **_: pytest.fail("legacy field"))
    assert dynamic.plan_dynamic_lattice(request, _model()).status == "success"

def test_guidance_is_soft_and_keeps_a_feasible_wider_dynamic_detour():
    result = dynamic.plan_dynamic_lattice(_request(guidance_cells=_tight_reference_cells()), _model())
    assert result.status == "success"
    assert any(_distance_from_reference(waypoint) > 0.1 for waypoint in result.waypoints)

def test_guided_search_is_deterministic():
    request = _request(guidance_cells=_detour_reference_cells())
    assert dynamic.plan_dynamic_lattice(request, _model()) == dynamic.plan_dynamic_lattice(request, _model())
```

Run these tests; they should fail because the current planner builds the legacy field for every request.

- [ ] **Step 2: Implement exactly one priority branch**

For `guidance_cells is not None`, construct the field once after start/goal validation, do not call `_grid_distance_to_goal`, and push `(node.cost + guided_field[state_cell], tie, index)`. Keep best-label comparison on real accumulated cost. Keep all successor, collision, risk and reconstruction code unchanged. Leave the no-guidance branch byte-for-byte equivalent to existing behavior.

- [ ] **Step 3: Add frozen E1 static regression**

Load `cluttered_hard_0_bodyband.npz`, inflate by `0.5 m`, derive `astar_path`, and build a stopped initial state from the E1 manifest start/yaw. Assert guided planning returns `success`, finite waypoints and `duration_s <= 32.0`. Do not make a machine-sensitive wall-time assertion in pytest.

- [ ] **Step 4: Verify and commit**

```bash
PYTHONPATH=research/ball_wm /home/an/miniconda3/envs/ball_wm/bin/python -m pytest research/ball_wm/tests/test_dynamic_lattice_planning.py -q
git add research/ball_wm/benchmark/dynamic_lattice_planning.py research/ball_wm/tests/test_dynamic_lattice_planning.py
git commit -m "feat: guide dynamic lattice search with static A star"
```

### Task 3: Privileged runtime handoff

**Files:**
- Modify: `research/ball_wm/benchmark/isaac_runtime.py`
- Modify: `research/ball_wm/tests/test_isaac_runtime_contract.py`

**Interfaces:**
- Consume existing `_initialize()` `path_cells`.
- Produce immutable `self._oracle_path_cells`, `oracle_path_cells(episode) -> np.ndarray`, and guided `DynamicPlanRequest`; V1 kinodynamic interfaces stay unchanged.

- [ ] **Step 1: Write failing runtime tests**

```python
def test_dynamic_plan_receives_frozen_static_astar_cells(tmp_path, monkeypatch):
    runtime, episode = _runtime_with_reset_and_oracle_cells(tmp_path, monkeypatch)
    captured = {}
    monkeypatch.setattr(isaac_runtime, "plan_dynamic_lattice", lambda request, _: captured.setdefault("request", request) or _plan())
    runtime.dynamic_plan(episode)
    np.testing.assert_array_equal(captured["request"].guidance_cells, runtime._oracle_path_cells)
    assert not captured["request"].guidance_cells.flags.writeable
```

Also verify `kinodynamic_plan()` does not read or require `_oracle_path_cells`. Run the runtime contract file; expected failure is missing guidance handoff.

- [ ] **Step 2: Implement handoff**

In `_initialize()`, copy `path_cells` into read-only `_oracle_path_cells` before converting it to the existing world-space overlay. Add `oracle_path_cells(episode)` with the same active-episode guard and copy-on-return semantics as `oracle_path_xy()`. In `dynamic_plan()`, pass private frozen cells and weight `0.35`. Do not add them to observations, histories or policy inputs.

- [ ] **Step 3: Verify and commit**

```bash
PYTHONPATH=research/ball_wm /home/an/miniconda3/envs/ball_wm/bin/python -m pytest research/ball_wm/tests/test_isaac_runtime_contract.py research/ball_wm/tests/test_benchmark_policies.py -q
git add research/ball_wm/benchmark/isaac_runtime.py research/ball_wm/tests/test_isaac_runtime_contract.py
git commit -m "feat: provide static guidance to dynamic oracle"
```

### Task 4: Guided audit metadata

**Files:**
- Modify: `research/ball_wm/scripts/run_pointgoal_benchmark.py`
- Modify: `research/ball_wm/benchmark/artifacts.py`
- Modify: `research/ball_wm/tests/test_benchmark_artifacts.py`

**Interfaces:**
- Produce `planner_version="dynamic_lattice_v2_guided"`, mode, weight, path length, cell count and cell SHA-256 in V2 environment and certification.

- [ ] **Step 1: Write failing bundle tests**

```python
def test_guided_v2_bundle_requires_complete_guidance_metadata(tmp_path):
    writer = _guided_dynamic_writer(tmp_path)
    writer.write_json("environment.json", _guided_environment_without("guidance_cells_sha256"))
    with pytest.raises(ValueError, match="guidance_cells_sha256"):
        writer.publish()

def test_v1_bundle_does_not_require_guidance_metadata(tmp_path):
    writer = _v1_writer(tmp_path)
    _write_complete_v1_bundle(writer)
    assert writer.publish().is_dir()
```

Also reject wrong V2 version, mode and weight. Run the artifact tests; expected failure is missing guided schema.

- [ ] **Step 2: Implement writer and validator**

Write six guidance fields under `dynamic_planner` and certification. Obtain a defensive copy through `runtime.oracle_path_cells(episode)`, compute path length from adjacent `int64` cells at map resolution and SHA-256 from contiguous cell bytes. Require exact version/mode/weight and complete metadata only when `environment.policy == "oracle_v2"`; leave V1 validation unchanged.

- [ ] **Step 3: Verify and commit**

```bash
PYTHONPATH=research/ball_wm /home/an/miniconda3/envs/ball_wm/bin/python -m pytest research/ball_wm/tests/test_benchmark_artifacts.py research/ball_wm/tests/test_benchmark_runner.py -q
git add research/ball_wm/scripts/run_pointgoal_benchmark.py research/ball_wm/benchmark/artifacts.py research/ball_wm/tests/test_benchmark_artifacts.py
git commit -m "feat: audit guided dynamic oracle plans"
```

### Task 5: No-Isaac frozen E1 preflight

**Files:**
- Create: `research/ball_wm/scripts/preflight_dynamic_oracle.py`
- Create: `research/ball_wm/tests/test_preflight_dynamic_oracle.py`
- Create: `docs/superpowers/specs/2026-09-14-dynamic-oracle-v2-guided-preflight.md`

**Interfaces:**
- Consume hash-verified manifest occupancy, 0.5 m inflation, static A* guidance and drive profile.
- Produce sorted JSON with hashes, guidance metadata, planner status, expansions, duration, total cost and wall time.

- [ ] **Step 1: Write failing preflight tests**

```python
def test_preflight_prints_verified_guided_success_json(tmp_path, capsys):
    assert preflight.main(_fixture_args(tmp_path)) == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["guidance_mode"] == "static_astar_soft_priority"
    assert payload["planner_status"] == "success"

def test_preflight_rejects_occupancy_hash_mismatch(tmp_path):
    with pytest.raises(ValueError, match="occupancy SHA256 mismatch"):
        preflight.main(_mismatched_occupancy_args(tmp_path))
```

Run the file; it should fail because no preflight module exists.

- [ ] **Step 2: Implement pure preflight**

Load the manifest/episode, resolve occupancy only under `--data-root`, verify its hash, map free labels to zero, inflate by `0.5 m`, call `astar_path`, build stopped initial state from manifest start/yaw, and call guided planning. Print sorted JSON. Return 0 only for plan success, otherwise print failure evidence and return 1. Do not import Isaac, `omni`, video or policy code.

- [ ] **Step 3: Measure E1 and write evidence**

```bash
PYTHONPATH=research/ball_wm /home/an/miniconda3/envs/ball_wm/bin/python research/ball_wm/scripts/preflight_dynamic_oracle.py --manifest research/ball_wm/configs/benchmark_v1/hard_0_dev_manifest.json --episode-id hard_0_e1_0002 --data-root research/ball_wm/data --drive-profile research/ball_wm/configs/benchmark_v1/drive_dynamics_v1.json
```

Write command and exact JSON to the preflight report. If planner status is non-success, commit the report and stop before Isaac.

- [ ] **Step 4: Verify and commit**

```bash
PYTHONPATH=research/ball_wm /home/an/miniconda3/envs/ball_wm/bin/python -m pytest research/ball_wm/tests/test_preflight_dynamic_oracle.py research/ball_wm/tests/test_dynamic_lattice_planning.py -q
git add research/ball_wm/scripts/preflight_dynamic_oracle.py research/ball_wm/tests/test_preflight_dynamic_oracle.py docs/superpowers/specs/2026-09-14-dynamic-oracle-v2-guided-preflight.md
git commit -m "test: preflight guided dynamic oracle"
```

### Task 6: E1 Isaac rerun and decision gate

**Files:**
- Create: `docs/superpowers/specs/2026-09-14-dynamic-oracle-v2-guided-e1-gate.md`
- Generated, ignored: `research/ball_wm/runs/e1_0002_dynamic_oracle_v2_guided_gate/`

**Interfaces:**
- Consume passing Task 5 preflight, frozen E1 manifest and guided `oracle_v2`.
- Produce a report that either stops or explicitly authorizes resuming the existing seven-task protocol.

- [ ] **Step 1: Run frozen E1 with a fresh run root**

```bash
/home/an/miniconda3/envs/ball_wm/bin/python research/ball_wm/scripts/run_pointgoal_benchmark.py --manifest research/ball_wm/configs/benchmark_v1/hard_0_dev_manifest.json --split dev --policy oracle_v2 --scene-root /home/an/project/NavDP/assets/scenes --data-root research/ball_wm/data --run-root research/ball_wm/runs/e1_0002_dynamic_oracle_v2_guided_gate --headless --record-video --episode-id hard_0_e1_0002
```

- [ ] **Step 2: Audit the complete published bundle**

Verify manifest, scene and occupancy hashes; plan success; zero collision; execution `<=35 s`; cross-track `<0.2 m`; main rate `<=0.5`; auxiliary rate `<=30°/s`; all media/NPZ/review files; and environment/certification guidance hash, count and path length consistency.

- [ ] **Step 3: Write branch decision and commit**

If any check fails, record planner status or first execution deviation and stop. If all pass, state that the existing seven-task Task 9 protocol may resume; do not run it in this task.

```bash
git add docs/superpowers/specs/2026-09-14-dynamic-oracle-v2-guided-e1-gate.md
git commit -m "test: evaluate guided dynamic oracle E1 gate"
```

## Plan self-review

- Tasks 1–2 implement soft non-pruning guidance; Task 3 confines it to the privileged runtime; Task 4 makes non-optimal guided behavior auditable; Tasks 5–6 retain the preflight and E1 stop gates.
- The plan contains no deferred requirements or unspecified parameters. Interface names introduced in Task 1 are consistently consumed by Tasks 2–5.
