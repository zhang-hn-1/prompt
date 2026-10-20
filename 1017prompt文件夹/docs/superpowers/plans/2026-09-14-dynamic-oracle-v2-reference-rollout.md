# Dynamic Oracle V2 Reference Rollout Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make privileged `oracle_v2` return a deterministic, collision-free, dynamics-executable reference plan for frozen E1 before an Isaac Sim run.

**Architecture:** Replace the uncommitted static-A* soft-priority experiment with a V3 reference rollout. A deterministic width-8 beam evaluates existing one-second 50 Hz motion primitives by their actual rolled-out state relative to static A*; a valid stopped-goal chain returns immediately, while safe nonterminal endpoints seed the unrestricted lattice. Runtime passes a static-A* copy only to Oracle V2 and artifacts audit that privilege.

**Tech Stack:** Python, NumPy, SciPy EDT, pytest, existing `DriveModel`; Isaac Sim only after pure-Python preflight.

**Spec:** `docs/superpowers/specs/2026-09-14-dynamic-oracle-v2-reference-rollout-design.md`

## Global Constraints

- Use worktree `/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark`, branch `codex/e1-detour-benchmark`.
- Keep `DynamicPlanRequest` positional compatibility: guidance fields stay after `occupied`. Replace the current uncommitted soft experiment with patches, never `git reset` or `git checkout`.
- Frozen E1 facts: occupancy SHA-256 `547dd73c52b45f9cc1aa8dca561c779349391f6ecef06ef59026921284b467da`; 620×620 grid, 0.1 m resolution, 0.5 m inflation; start `(-9.85, 4.05)`; goal `(-7.75, -4.15)`; yaw `-1.3200870673734872` rad.
- Keep 1.0 s primitives, 0.02 s samples/collision checks, 0.5 rad/s² main acceleration, 30°/s auxiliary rate, ±30° range, stopped reversal, 32 s horizon, 500,000 nodes and 60 s ceiling.
- Return only `propagate_dynamic_primitive` samples. No direct A* interpolation, straight-line fallback, relaxed collision/goal rule, altered map or obstacle inflation.
- Guidance, its hash and provenance remain Oracle-only: never PolicyObservation, training data, world-model target/loss or V1 schema.
- Do not start Isaac until Tasks 1–5 pass and the user explicitly approves Task 6.

---

### Task 1: Reconcile soft-priority code into shared expansion primitives

**Files:**
- Modify: `research/ball_wm/benchmark/dynamic_lattice_planning.py`
- Modify: `research/ball_wm/tests/test_dynamic_lattice_planning.py`

**Interfaces:**
- Produces `_iter_valid_successors(node, *, request, drive_model, clearance, templates)` and `_reference_progress_fields(request) -> tuple[np.ndarray, np.ndarray] | None`.
- Preserves exact no-guidance result, existing `_grid_distance_to_goal` legacy priority and status literals.

- [ ] **Step 1: Write failing tests**

Add `test_reference_progress_fields_are_metric_without_creating_a_corridor`: a guidance route `[[1,1],[1,2],[2,3]]` has zero remaining at `[2,3]`, positive lateral distance at `[4,4]`, and finite remaining there. Add `test_shared_successor_foundation_keeps_no_guidance_result_deterministic`: two `plan_dynamic_lattice(_request(planning_timeout_s=5.0), _model())` results compare equal.

- [ ] **Step 2: Confirm RED**

Run: `pytest -q research/ball_wm/tests/test_dynamic_lattice_planning.py -k 'reference_progress_fields or shared_successor_foundation'`

Expected: FAIL because the named interfaces do not yet exist. Replace the intentionally red old E1 soft-priority test; retain its diagnostic report in ignored SDD state and create the V3 E1 gate in Task 3.

- [ ] **Step 3: Implement minimal foundation**

Extract the existing successor loop unchanged into the iterator: retain quantized ±0.5 rad/s and ±30° target gates, primitive cache, frame transformation, 50 Hz all-sample collision checks, current edge cost and last-direction calculation. Add defaulted internal fields `reference_index: int | None = None` and `source: Literal["root", "reference_rollout", "lattice"] = "root"` after current `_SearchNode` fields. Derive path remaining and metric EDT lateral fields only from validated immutable cells; return `None` without guidance. Remove the unfinished guided heap-priority branch while preserving legacy Dijkstra for no-guidance requests.

- [ ] **Step 4: Confirm GREEN**

Run: `pytest -q research/ball_wm/tests/test_dynamic_lattice_planning.py -k 'reference_progress_fields or shared_successor_foundation or no_guidance or deterministic'`; then `git diff --check`.

Expected: PASS and silent diff check.

- [ ] **Step 5: Commit**

Run: `git add research/ball_wm/benchmark/dynamic_lattice_planning.py research/ball_wm/tests/test_dynamic_lattice_planning.py && git commit -m "refactor: share dynamic lattice primitive expansion"`.

### Task 2: Build deterministic, dynamics-feasible reference rollout

**Files:**
- Modify: `research/ball_wm/benchmark/dynamic_lattice_planning.py`
- Modify: `research/ball_wm/tests/test_dynamic_lattice_planning.py`

**Interfaces:**
- Produces `_ReferenceRollout(nodes, seed_indices, goal_index, expanded_nodes)` and `_build_reference_rollout(request, drive_model, clearance)`.
- Consumes Task 1 iterator/fields and `_reconstruct_dynamic`.

- [ ] **Step 1: Write failing beam tests**

Add a tiny turning-map fixture and `test_reference_rollout_reaches_stopped_goal_from_misaligned_yaw`: build with `initial_yaw_rad=pi/2`, assert `goal_index is not None`, reconstruct, assert duration ≤32 s and final direction is stopped. Add `test_reference_rollout_rejects_colliding_high_progress_primitive`, asserting every retained primitive passes `dynamic_primitive_is_collision_free`. Add `test_reference_rollout_is_deterministic_and_has_safe_off_reference_seed`, comparing two rollouts and asserting a retained `source == "reference_rollout"` node.

- [ ] **Step 2: Confirm RED**

Run: `pytest -q research/ball_wm/tests/test_dynamic_lattice_planning.py -k reference_rollout`

Expected: FAIL because `_build_reference_rollout` is absent.

- [ ] **Step 3: Implement beam**

For guidance requests, create a local node graph rooted at exact start state. At every one-second layer call Task 1’s iterator. Set child progress to the maximum of its parent index and nearest reference index. Rank candidates by `(remaining_m[progress], lateral_m_at_end, child.cost, target.main_command_rad_s, target.auxiliary_angle_rad, creation_index)`. After dominance filtering, reserve the best available endpoint in each non-empty `forward`, `reverse`, and `stopped` direction class, then fill remaining slots in the original global rank; retain at most 8. Lateral distance ranks only: a free child is never rejected merely because it leaves the reference. Preserve local parent links and real primitive chains, mark survivors `source="reference_rollout"`, stop branches at 32 s, and choose a stopped-goal candidate by the same stable rank. `expanded_nodes` counts considered beam children, not samples.

- [ ] **Step 4: Confirm GREEN**

Run: `pytest -q research/ball_wm/tests/test_dynamic_lattice_planning.py -k reference_rollout`; then `git diff --check`.

Expected: PASS and silent diff check.

- [ ] **Step 5: Commit**

Run: `git add research/ball_wm/benchmark/dynamic_lattice_planning.py research/ball_wm/tests/test_dynamic_lattice_planning.py && git commit -m "feat: add dynamics-feasible reference rollout"`.

### Task 3: Integrate warm-start and pass the frozen E1 Python gate

**Files:**
- Modify: `research/ball_wm/benchmark/dynamic_lattice_planning.py`
- Modify: `research/ball_wm/tests/test_dynamic_lattice_planning.py`

**Interfaces:**
- Guided call returns the beam’s executable goal chain immediately, otherwise seeds root plus safe beam endpoints into existing unrestricted lattice.
- No-guidance behavior remains legacy-identical.

- [ ] **Step 1: Write failing E1 test**

Add `test_reference_rollout_solves_hash_verified_hard_0_e1_0002_static_input`. Its helper reads repository manifest and occupancy, asserts the frozen SHA, applies exactly 0.5 m inflation and the manifest start/goal/yaw. Plan twice and assert both are `success`, controls/waypoints are equal, duration is `(0, 32]`, waypoint values are finite, and every sample clears the frozen grid.

- [ ] **Step 2: Confirm RED without Isaac**

Run: `pytest -q research/ball_wm/tests/test_dynamic_lattice_planning.py -k 'reference_rollout_solves_hash_verified_hard_0_e1_0002_static_input'`

Expected: FAIL until the planner invokes the beam. Do not start Isaac.

- [ ] **Step 3: Integrate safely**

After existing start/goal checks and clearance construction, call the beam for guidance. If `goal_index` exists, return `_reconstruct_dynamic` at once. Otherwise remap local rollout parents and seed root plus legal endpoints into `nodes`, `best` and `frontier`, keeping each actual cost/time/primitive. Normal expansion must not test path-cell membership. No valid beam child falls back to original root lattice; existing `timeout`, `unreachable` and `node_budget_exhausted` semantics remain.

- [ ] **Step 4: Confirm E1 and planner regressions GREEN**

Run: `pytest -q research/ball_wm/tests/test_dynamic_lattice_planning.py` and `pytest -q research/ball_wm/tests/test_kinodynamic_planning.py`.

Expected: PASS. Record E1 status, wall time, duration, path length, direction changes and expanded nodes. Required result: deterministic, finite, nonempty, zero frozen-grid collision samples, duration ≤32 s.

- [ ] **Step 5: Commit**

Run: `git add research/ball_wm/benchmark/dynamic_lattice_planning.py research/ball_wm/tests/test_dynamic_lattice_planning.py && git commit -m "feat: warm start dynamic oracle with reference rollout"`.

### Task 4: Provide Oracle-only cells and audit V3 provenance

**Files:**
- Modify: `research/ball_wm/benchmark/isaac_runtime.py`
- Modify: `research/ball_wm/scripts/run_pointgoal_benchmark.py`
- Modify: `research/ball_wm/benchmark/artifacts.py`
- Modify: `research/ball_wm/tests/test_isaac_runtime_contract.py`
- Modify: `research/ball_wm/tests/test_benchmark_runner.py`
- Modify: `research/ball_wm/tests/test_benchmark_artifacts.py`

**Interfaces:**
- Produces `IsaacBenchmarkRuntime.oracle_path_cells(episode) -> np.ndarray`, active-episode scoped and defensive-copying, plus strict V3 audit validation.
- V1 does not consume/reference these fields.

- [ ] **Step 1: Write failing contract/audit tests**

Add `test_oracle_path_cells_are_episode_scoped_and_defensive`: reset runtime, mutate returned cells, assert a fresh call is untouched, and assert another episode raises `ValueError("active episode")`. Add `test_oracle_v2_v3_bundle_requires_reference_provenance`: V3 bundle publishes with `reference_rollout_status="goal_incumbent"`, then fails if `reference_cell_sha256` is omitted.

- [ ] **Step 2: Confirm RED**

Run: `pytest -q research/ball_wm/tests/test_isaac_runtime_contract.py research/ball_wm/tests/test_benchmark_runner.py research/ball_wm/tests/test_benchmark_artifacts.py -k 'oracle_path_cells or reference_provenance'`

Expected: FAIL because V3 access/schema is absent.

- [ ] **Step 3: Implement V3 wiring**

Keep reset-time `path_cells` as immutable `_oracle_path_cells` and derive the existing XY display curve from those cells. In `dynamic_plan`, pass only a defensive copied array and fixed `guidance_weight=.35`. Producer and validator require exact V3 fields: planner version `dynamic_lattice_v3_reference_rollout`, reference mode `static_astar_dynamics_beam_warm_start`, beam width `8`, cell count, metric length, SHA-256, rollout status (`goal_incumbent`/`seeded_search`/`no_valid_seed`), rollout expansion count and plan source (`reference_rollout`/`lattice`). Add public result provenance only as defaulted trailing fields if it is needed. Do not require them for V1.

- [ ] **Step 4: Confirm GREEN**

Run: `pytest -q research/ball_wm/tests/test_isaac_runtime_contract.py research/ball_wm/tests/test_benchmark_runner.py research/ball_wm/tests/test_benchmark_artifacts.py`; then `git diff --check`.

Expected: PASS without launching Isaac and silent diff check.

- [ ] **Step 5: Commit**

Run: `git add research/ball_wm/benchmark/isaac_runtime.py research/ball_wm/scripts/run_pointgoal_benchmark.py research/ball_wm/benchmark/artifacts.py research/ball_wm/tests/test_isaac_runtime_contract.py research/ball_wm/tests/test_benchmark_runner.py research/ball_wm/tests/test_benchmark_artifacts.py && git commit -m "feat: audit oracle reference rollout provenance"`.

### Task 5: Complete non-GPU verification and record evidence

**Files:**
- Create: ignored `.superpowers/sdd/2026-09-15-dynamic-oracle-v2-reference-rollout/task-5-preflight-report.md`
- Modify: no source unless a separate red/green defect task becomes necessary.

**Interfaces:**
- Produces exact commands/results, frozen E1 statistics and git state; it never claims an Isaac result.
- Gates Task 6.

- [ ] **Step 1: Run frozen E1 twice**

Run Task 3’s exact frozen E1 pytest command twice. Both must pass and test assertions prove identical plans; record command wall times without asserting a machine-specific limit.

- [ ] **Step 2: Run complete non-GPU suite**

Run: `pytest -q research/ball_wm/tests && git diff --check && git status --short`.

Expected: all tests PASS except documented pre-existing third-party deprecation warnings; silent diff check and no unintended files.

- [ ] **Step 3: Write report**

Record literal map hash, inflation, episode/start/goal/yaw, static A* cells/path length, every command/result, plan statistics and explicit `Isaac not launched`. A failed assertion blocks Task 6 and starts systematic debugging; it never authorizes benchmark changes.

### Task 6: Run the one-task Isaac E1 Gate after fresh user approval

**Files:**
- Create: unique generated bundle under `research/ball_wm/runs/` per repository ignore policy
- Create: ignored `.superpowers/sdd/2026-09-15-dynamic-oracle-v2-reference-rollout/task-6-isaac-gate-report.md`
- Modify: no source during Gate.

**Interfaces:**
- Consumes Task 5 success and existing `oracle_v2` CLI.
- Produces one audited E1 V3 bundle or a blocked report; never starts the seven-task evaluation.

- [ ] **Step 1: Ask permission**

Report Task 5 evidence and request approval for a real single-task GPU/Isaac Sim Gate. Stop until approval.

- [ ] **Step 2: Run frozen task after approval**

Use frozen `hard_0` manifest, exact filter `hard_0_e1_0002`, `oracle_v2`, real data root/occupancy, unique run path and current user-selected GUI/headless setting. Never use a synthetic scene.

- [ ] **Step 3: Audit and report**

Audit the bundle. Require planning success, zero collisions, ≤35 s execution, ≤32 s predicted duration, `<0.2 m` maximum cross-track error, V3 planner/reference fields, and all video/state/action/metrics files. On pass link bundle, overlay and video and claim only privileged E1 success. On failure record exact stage/status, stop and do not alter task inputs or run additional cards.

## Plan Self-Review

- Tasks 1–4 implement every architecture, safety, determinism, Oracle-boundary and audit requirement in the spec.
- Task 3 is the frozen pure-Python E1 gate; Task 5 requires full evidence before GPU; Task 6 requires a separate consent.
- This supersedes the retained soft-priority experiment without destructive git operations. No placeholder terms or deferred implementation steps remain.
