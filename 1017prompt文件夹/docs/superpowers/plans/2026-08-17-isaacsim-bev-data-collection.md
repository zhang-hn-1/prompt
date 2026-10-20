# Isaac Sim BEV Data Collection Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Upgrade the Dubins collector to record contract-compliant synchronized stereo, proprioception, 2D actions, timestamps, Isaac Sim geometry BEV truth, stereo visibility masks, and auditable previews.

**Architecture:** Keep Isaac Sim lifecycle and control in `collect_data_dubins.py`; move deterministic scheduling, coordinate transforms, serialization, and auditing into importable modules that run without Isaac Sim. Wrap Isaac Sim 4.5 Occupancy Map and camera depth access behind a small adapter so simulator-specific failures are explicit while mathematical behavior remains unit tested.

**Tech Stack:** Python 3.10, NumPy, Pillow, PyYAML, pytest, Isaac Sim 4.5 (`isaacsim.asset.gen.omap`, `omni.isaac.sensor.Camera`), USD/PhysX.

## Global Constraints

- Source contract: `research/ball_wm/configs/data_contract.yaml`, schema `1.1.0`.
- Physics/state/control/stereo rates are exactly 50/50/50/10 Hz.
- Physics dt is 0.02 s and stereo period is 0.1 s; each visual interval contains exactly 5 state/action samples.
- The 0.5 s history contains 5 stereo frames, 25 state samples, and 25 action samples; the 1.0 s prediction horizon contains 10 stereo steps.
- Action is float32 `[main_joint_velocity_cmd, auxiliary_joint_angle_cmd]`; no third dimension.
- Base frame is `+X forward, +Y left, +Z up`; BEV image up is base `+X`, image left is base `+Y`.
- Local BEV is 8.0 m × 8.0 m at 0.1 m resolution with shape `80 × 80` and uint8 classes `0 free, 1 occupied, 2 unknown`.
- RGB is deployment input; depth, geometry BEV, world pose, and visibility masks are training-only supervision.
- RGB dimensions are fixed at `640 × 480`; `EpisodeWriter` validates every pair against the declared metadata dimensions.
- Only the explicit main-loop `world.step()` advances physics; getters never advance simulation.
- Static geometry occupancy is generated once per loaded scene and excludes `/World/BallRobot`.
- Existing collected datasets are never rewritten or deleted.

---

### Task 1: Deterministic multi-rate schedule and episode schema

**Files:**
- Create: `research/ball_wm/scripts/collection_schema.py`
- Create: `research/ball_wm/tests/test_collection_schema.py`

**Interfaces:**
- Produces: `RateSchedule(physics_hz, control_hz, stereo_hz, state_hz)`, `RateSchedule.is_control_step(step)`, `RateSchedule.is_stereo_step(step)`, `validate_action(action)`, `EpisodeBuffer.append_state(...)`, `EpisodeBuffer.append_stereo(...)`, and `EpisodeBuffer.as_arrays()`.
- Consumes: no Isaac Sim imports; only NumPy and standard library.

- [ ] **Step 1: Write failing schedule and action tests**

```python
def test_schedule_emits_exact_50_50_10_boundaries():
    schedule = RateSchedule(physics_hz=50, control_hz=50, stereo_hz=10, state_hz=50)
    assert [s for s in range(10) if schedule.is_control_step(s)] == list(range(10))
    assert [s for s in range(10) if schedule.is_stereo_step(s)] == [0, 5]

def test_validate_action_rejects_legacy_three_dimensions():
    with pytest.raises(ValueError, match="exactly 2"):
        validate_action(np.zeros(3, dtype=np.float32))

def test_validate_action_returns_float32_copy():
    result = validate_action([1.0, -0.2])
    assert result.dtype == np.float32
    np.testing.assert_array_equal(result, np.array([1.0, -0.2], dtype=np.float32))
```

- [ ] **Step 2: Run tests and verify RED**

Run: `pytest -q research/ball_wm/tests/test_collection_schema.py`

Expected: collection fails with `ModuleNotFoundError: No module named 'collection_schema'`.

- [ ] **Step 3: Implement schedule validation and 2D action normalization**

```python
@dataclass(frozen=True)
class RateSchedule:
    physics_hz: int = 50
    control_hz: int = 50
    stereo_hz: int = 10
    state_hz: int = 50

    def __post_init__(self):
        if (self.physics_hz, self.control_hz, self.state_hz, self.stereo_hz) != (50, 50, 50, 10):
            raise ValueError("rates must be exactly 50/50/50/10 Hz")

    def is_control_step(self, step: int) -> bool:
        return step % (self.physics_hz // self.control_hz) == 0

    def is_stereo_step(self, step: int) -> bool:
        return step % (self.physics_hz // self.stereo_hz) == 0

def validate_action(action) -> np.ndarray:
    result = np.asarray(action, dtype=np.float32)
    if result.shape != (2,):
        raise ValueError(f"action must have exactly 2 elements, got {result.shape}")
    return result.copy()
```

- [ ] **Step 4: Add failing buffer invariants tests**

```python
def test_episode_buffer_preserves_state_and_stereo_time_axes():
    buf = EpisodeBuffer()
    buf.append_state(make_state(step=0, time=0.00))
    buf.append_state(make_state(step=1, time=0.02))
    buf.append_stereo(make_stereo(frame_id=0, step=0, time=0.00))
    arrays = buf.as_arrays()
    assert arrays["physics_step"].shape == (2,)
    assert arrays["applied_action"].shape == (2, 2)
    assert arrays["stereo_frame_id"].shape == (1,)
    assert arrays["cam_left_pose"].shape == (1, 4, 4)

def test_episode_buffer_rejects_nonmonotonic_state_time():
    buf = EpisodeBuffer()
    buf.append_state(make_state(step=0, time=0.00))
    with pytest.raises(ValueError, match="strictly increasing"):
        buf.append_state(make_state(step=1, time=0.00))
```

- [ ] **Step 5: Implement typed state/stereo records and buffer conversion**

Use dataclasses `StateRecord` and `StereoRecord`. `StateRecord` contains the contract state arrays, both actions, and three action/state timestamps. `StereoRecord` contains frame ID, physics step, left/right timestamps, both camera matrices, filenames, collision, and minimum obstacle distance. Validate shapes at append time and `np.stack` only non-empty lists in `as_arrays()`.

- [ ] **Step 6: Run Task 1 tests and commit**

Run: `pytest -q research/ball_wm/tests/test_collection_schema.py`

Expected: all Task 1 tests pass.

```bash
git add research/ball_wm/scripts/collection_schema.py research/ball_wm/tests/test_collection_schema.py
git commit -m "feat: add synchronized collection schema"
```

---

### Task 2: Pure BEV coordinates, local crop, and obstacle metrics

**Files:**
- Create: `research/ball_wm/scripts/bev_ground_truth.py`
- Create: `research/ball_wm/tests/test_bev_ground_truth.py`

**Interfaces:**
- Produces: `BEVConfig`, `world_points_to_local_bev(points_world, base_position, base_yaw, config)`, `crop_global_occupancy(global_map, global_origin_xy, base_position, base_yaw, config)`, `obstacle_clearance(local_bev, ball_radius, config)`, and `warp_history_mask(previous_mask, previous_pose, current_pose, config)`.
- Consumes: NumPy only for these functions; Isaac imports remain outside module top level.

- [ ] **Step 1: Write failing coordinate convention tests**

```python
def test_base_forward_maps_up_and_base_left_maps_image_left():
    cfg = BEVConfig(extent_m=8.0, resolution_m=0.1)
    points = np.array([[1.0, 0.0, 0.2], [0.0, 1.0, 0.2]])
    rows, cols = world_points_to_local_bev(points, np.zeros(3), 0.0, cfg)
    assert (rows[0], cols[0]) == (30, 40)
    assert (rows[1], cols[1]) == (40, 30)

def test_world_forward_rotates_with_base_yaw():
    cfg = BEVConfig(extent_m=8.0, resolution_m=0.1)
    rows, cols = world_points_to_local_bev(
        np.array([[0.0, 1.0, 0.2]]), np.zeros(3), np.pi / 2, cfg
    )
    assert (rows[0], cols[0]) == (30, 40)
```

The literal pixel values follow `floor(center - coordinate / 0.1)` with center pixel 40; they do not reuse production helpers.

- [ ] **Step 2: Run coordinate tests and verify RED**

Run: `pytest -q research/ball_wm/tests/test_bev_ground_truth.py`

Expected: import fails because `bev_ground_truth.py` does not exist.

- [ ] **Step 3: Implement BEV configuration and coordinate transforms**

```python
@dataclass(frozen=True)
class BEVConfig:
    extent_m: float = 8.0
    resolution_m: float = 0.1
    free: int = 0
    occupied: int = 1
    unknown: int = 2

    @property
    def shape(self) -> tuple[int, int]:
        side = int(round(self.extent_m / self.resolution_m))
        return side, side
```

Transform world deltas with `R_world_to_base = [[cos(yaw), sin(yaw)], [-sin(yaw), cos(yaw)]]`; map base X to decreasing row and base Y to decreasing column.

- [ ] **Step 4: Write failing crop, clearance, and history-warp tests**

```python
def test_clearance_subtracts_ball_radius():
    bev = np.zeros((80, 80), dtype=np.uint8)
    bev[40, 50] = 1
    clearance = obstacle_clearance(bev, ball_radius=0.4, config=BEVConfig())
    assert clearance == pytest.approx(0.6)

def test_history_mask_moves_backward_when_robot_moves_forward():
    previous = np.zeros((80, 80), dtype=np.uint8)
    previous[30, 40] = 1
    warped = warp_history_mask(previous, (0.0, 0.0, 0.0), (0.5, 0.0, 0.0), BEVConfig())
    assert warped[35, 40] == 1
    assert warped[30, 40] == 0
```

- [ ] **Step 5: Implement nearest-neighbor crop/warp and clearance**

Generate the target local grid centers, transform them into the source coordinate system, sample nearest-neighbor labels, and fill out-of-bounds cells with unknown. Compute clearance from occupied cell-center coordinates, subtract `ball_radius`, return `np.inf` if no occupied cells, and define collision as `clearance <= 0.0`.

- [ ] **Step 6: Run Task 2 tests and commit**

Run: `pytest -q research/ball_wm/tests/test_bev_ground_truth.py`

Expected: all Task 2 tests pass.

```bash
git add research/ball_wm/scripts/bev_ground_truth.py research/ball_wm/tests/test_bev_ground_truth.py
git commit -m "feat: add local BEV geometry transforms"
```

---

### Task 3: Isaac Sim 4.5 occupancy and stereo-depth adapters

**Files:**
- Modify: `research/ball_wm/scripts/bev_ground_truth.py`
- Modify: `research/ball_wm/tests/test_bev_ground_truth.py`

**Interfaces:**
- Produces: `IsaacOccupancyProvider(stage, scene_root, robot_root, config)`, `IsaacOccupancyProvider.build_global_map() -> GlobalOccupancy`, `enable_camera_depth(camera)`, `read_camera_depth(camera) -> np.ndarray`, and `depth_to_visible_mask(depth, intrinsics, camera_to_base, config)`.
- Consumes: Isaac Sim imports loaded lazily inside adapter methods; pure projection consumes NumPy arrays.

- [ ] **Step 1: Write failing depth projection tests with hand-derived pixels**

```python
def test_center_depth_marks_forward_ray_visible():
    depth = np.array([[2.0]], dtype=np.float32)
    intrinsics = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
    camera_to_base = np.array([
        [0.0, 0.0, 1.0, 0.0],
        [-1.0, 0.0, 0.0, 0.0],
        [0.0, -1.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])
    mask = depth_to_visible_mask(depth, intrinsics, camera_to_base, BEVConfig())
    assert mask[40, 40] == 1
    assert mask[20, 40] == 1

def test_invalid_depth_does_not_mark_visible_cells():
    depth = np.array([[np.inf, np.nan, 0.0]], dtype=np.float32)
    camera_to_base = np.array([
        [0.0, 0.0, 1.0, 0.0],
        [-1.0, 0.0, 0.0, 0.0],
        [0.0, -1.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])
    mask = depth_to_visible_mask(depth, np.eye(3), camera_to_base, BEVConfig())
    assert np.count_nonzero(mask) == 0
```

Camera-frame to base-frame axis conversion must be explicit in `camera_to_base`; the projection function makes no undocumented Isaac camera-axis assumption.

- [ ] **Step 2: Run projection tests and verify RED**

Run: `pytest -q research/ball_wm/tests/test_bev_ground_truth.py -k depth`

Expected: tests fail because `depth_to_visible_mask` is absent.

- [ ] **Step 3: Implement ray rasterization and stereo union**

Back-project valid depth pixels, transform endpoints to base, sample each origin-to-endpoint segment at half-cell spacing, and set traversed in-range cells to 1. Return a binary uint8 visibility mask, not occupancy labels.

- [ ] **Step 4: Implement the lazy Isaac adapters**

At runtime enable `isaacsim.asset.gen.omap`, import `from isaacsim.asset.gen.omap import _omap`, construct `_omap.Generator(omni.physx.acquire_physx_interface(), stage_id)`, configure 0.1 m values `(1, 0, 2)`, and call `generate2d()`. Temporarily disable collision on prims under `/World/BallRobot` while generating, restore every modified attribute in `finally`, and reject an all-free map when a non-empty scene root was supplied.

For each `Camera`, call `add_distance_to_image_plane_to_frame()` after initialization. `read_camera_depth` requires a finite-shape `distance_to_image_plane` entry and raises `RuntimeError` with camera path and available frame keys when absent.

- [ ] **Step 5: Add adapter error tests without mocking framework behavior**

Test only local boundary behavior: a small fake frame dictionary passed to `_extract_depth_from_frame(frame, camera_path)` returns a float32 2D array; missing key and wrong shape raise explicit errors. Do not assert that Isaac methods were called.

- [ ] **Step 6: Run pure tests and commit**

Run: `pytest -q research/ball_wm/tests/test_bev_ground_truth.py`

Expected: all tests pass without launching Isaac Sim.

```bash
git add research/ball_wm/scripts/bev_ground_truth.py research/ball_wm/tests/test_bev_ground_truth.py
git commit -m "feat: add Isaac occupancy and visibility adapters"
```

---

### Task 4: Dataset writer, preview, and offline audit

**Files:**
- Modify: `research/ball_wm/scripts/collection_schema.py`
- Create: `research/ball_wm/scripts/audit_dataset.py`
- Modify: `research/ball_wm/tests/test_collection_schema.py`
- Create: `research/ball_wm/tests/test_audit_dataset.py`

**Interfaces:**
- Produces: `EpisodeWriter(output_dir, metadata, save_depth, save_preview)`, `EpisodeWriter.write_visual_sample(...)`, `EpisodeWriter.finalize(buffer)`, `audit_episode(path) -> list[str]`, and CLI exit code 0 only when all episodes pass.
- Consumes: Task 1 records and Task 2 BEV arrays.

- [ ] **Step 1: Write failing writer behavior test**

```python
def test_writer_uses_lossless_paired_stereo_and_contract_arrays(tmp_path):
    writer = EpisodeWriter(tmp_path, metadata=minimal_metadata(), save_depth=True, save_preview=False)
    writer.write_visual_sample(frame_id=0, left_rgb=rgb(), right_rgb=rgb(),
                               left_depth=depth(), right_depth=depth(),
                               geometry_bev=bev(), current_visible=mask(), history_observed=mask())
    writer.finalize(minimal_buffer())
    assert (tmp_path / "left_000000.png").exists()
    assert (tmp_path / "right_000000.png").exists()
    with np.load(tmp_path / "states.npz") as data:
        assert data["applied_action"].shape[1] == 2
```

- [ ] **Step 2: Run writer test and verify RED**

Run: `pytest -q research/ball_wm/tests/test_collection_schema.py -k writer`

Expected: import fails because `EpisodeWriter` is absent.

- [ ] **Step 3: Implement atomic episode writing and preview mosaic**

Write into `epXXXX.partial/`; rename to `epXXXX/` only after `states.npz` and `metadata.json` succeed. Each visual sample uses `left_XXXXXX.png`, `right_XXXXXX.png`, optional float32 `left_depth_XXXXXX.npy` / `right_depth_XXXXXX.npy`, and uint8 PNG labels `geometry_bev_XXXXXX.png`, `current_visible_XXXXXX.png`, and `history_observed_XXXXXX.png`. The preview is a horizontal PNG mosaic containing left RGB, right RGB, colorized geometry BEV, current visibility, and history visibility. Existing final episode directories cause `FileExistsError`; never overwrite them.

- [ ] **Step 4: Write failing audit tests using real temporary files**

```python
def test_audit_rejects_three_dimensional_action(tmp_path):
    episode = write_valid_fixture(tmp_path)
    replace_npz_field(episode / "states.npz", "applied_action", np.zeros((10, 3), np.float32))
    errors = audit_episode(episode)
    assert any("applied_action must have shape (N, 2)" in error for error in errors)

def test_audit_rejects_unpaired_stereo_file(tmp_path):
    episode = write_valid_fixture(tmp_path)
    (episode / "right_000000.png").unlink()
    assert any("missing right_000000.png" in error for error in audit_episode(episode))
```

- [ ] **Step 5: Implement audit checks and CLI**

Check required NPZ fields, dtypes/shapes, finite numeric state/action/timestamp/pose values, the documented `+inf` no-obstacle exception, collision/clearance agreement, strictly increasing state time, equal left/right timestamp and stereo step arrays, expected 50/10 sample ratio within final partial interval, consecutive frame IDs, paired file existence, fixed RGB dimensions, BEV/mask shape, and allowed values. Print one line per error and return process exit code 1 if any error exists.

- [ ] **Step 6: Run Task 4 tests and commit**

Run: `pytest -q research/ball_wm/tests/test_collection_schema.py research/ball_wm/tests/test_audit_dataset.py`

Expected: all tests pass.

```bash
git add research/ball_wm/scripts/collection_schema.py research/ball_wm/scripts/audit_dataset.py research/ball_wm/tests
git commit -m "feat: write and audit BEV collection episodes"
```

---

### Task 5: Integrate the formal Dubins collection loop

**Files:**
- Modify: `research/ball_wm/scripts/collect_data_dubins.py`
- Modify: `research/ball_wm/configs/data_contract.yaml`

**Interfaces:**
- Consumes: `RateSchedule`, records, `EpisodeWriter`, `IsaacOccupancyProvider`, BEV crop/projection/history functions, and existing Dubins controller.
- Produces: executable formal collection CLI and schema `1.1.0` metadata including new obstacle fields.

- [ ] **Step 1: Add a failing contract integration test**

Extend `test_collection_schema.py` to load `data_contract.yaml` and build `RateSchedule` plus `BEVConfig` through `load_contract(path)`. Assert returned action names, rates, BEV shape, and class codes as observable parsed configuration. Run it before adding `load_contract` and confirm the missing-symbol failure.

- [ ] **Step 2: Implement contract loading and CLI defaults**

Add `--contract`, `--physics_hz`, `--control_hz`, `--stereo_hz`, `--bev_extent_m`, `--bev_resolution_m`, `--no_save_depth`, `--save_preview`, and `--audit_after_episode`. Frequency defaults are fixed at 50/50/50/10 Hz; explicit CLI frequency values are allowed only when they equal those fixed values and otherwise fail validation. Validated values and `waypoints_world_xy` are written to `metadata.json`.

- [ ] **Step 3: Replace the loop with one-step ownership**

Set `World(physics_dt=0.02, rendering_dt=0.1)`. Apply a freshly computed clipped action on every control boundary (each physics step), call exactly one `world.step(render=is_stereo_step)`, then record state. On stereo boundaries read both frames and depths from that completed step, compute local geometry BEV and visibility masks, write visual files, and append one `StereoRecord`. Remove the old 3D `action` field and ensure `get_cam_pose()` is a pure USD query.

- [ ] **Step 4: Add sign-probe mode**

Add `--sign_probe {none,main_positive,main_negative,aux_positive,aux_negative}` and `--sign_probe_steps`. In probe mode bypass Dubins waypoints, apply exactly one configured joint command, save the same formal format, and print net base translation/yaw. This closes the unchecked Phase 0 action-direction validation without hard-coding a guessed left/right meaning.

- [ ] **Step 5: Validate syntax and pure suite**

Run:

```bash
python -m py_compile research/ball_wm/scripts/collect_data_dubins.py
pytest -q research/ball_wm/tests
```

Expected: compilation succeeds and all pure tests pass.

- [ ] **Step 6: Commit integration**

```bash
git add research/ball_wm/scripts/collect_data_dubins.py research/ball_wm/configs/data_contract.yaml research/ball_wm/tests/test_collection_schema.py
git commit -m "feat: collect synchronized stereo and BEV truth"
```

---

### Task 6: Documentation, legacy warning, and Isaac Sim smoke test

**Files:**
- Modify: `research/ball_wm/README.md`
- Modify: `research/ball_wm/scripts/collect_data.py`
- Modify: `research/Ball_Nav/PHASE0_RESEARCH_CONTRACT.md`

**Interfaces:**
- Consumes: final CLI and audit command from Tasks 4–5.
- Produces: runnable user instructions and recorded Phase 1 verification status.

- [ ] **Step 1: Document the exact one-episode visual check command**

Update README to describe PointGoal/BEV rather than EV-WM/ImageGoal, mark `collect_data.py` legacy at its module header, document every emitted field, and include:

```bash
python research/ball_wm/scripts/collect_data_dubins.py \
  --scene /absolute/path/to/scene.usd \
  --num_episodes 1 --episode_len 100 \
  --save_preview --audit_after_episode \
  --save_dir /tmp/ball_wm_bev_smoke
```

- [ ] **Step 2: Run repository verification before simulator launch**

Run:

```bash
pytest -q research/ball_wm/tests
python -m py_compile research/ball_wm/scripts/collection_schema.py research/ball_wm/scripts/bev_ground_truth.py research/ball_wm/scripts/audit_dataset.py research/ball_wm/scripts/collect_data_dubins.py
git diff --check
```

Expected: all tests pass, all modules compile, and diff check is clean.

- [ ] **Step 3: Run Isaac Sim one-episode smoke collection**

Use the user's valid scene path, or omit `--scene` for the default ground only compatibility check. Run with `--num_episodes 1 --episode_len 100 --save_preview --audit_after_episode` into a new explicit directory under `/tmp`.

Expected: process exits 0; log reports 100 state samples, 20 stereo samples, action dimension 2, geometry map generation success, and audit PASS.

- [ ] **Step 4: Inspect the generated preview**

Open one preview through the local image viewer. Verify obstacle direction matches RGB/scene, the robot body is absent from geometry BEV, current-visible mask follows the two camera views, and history mask is a superset of current visibility. If using a ground-only scene, geometry BEV must be free rather than fabricated occupied data; repeat with the user's cluttered scene before declaring obstacle labels validated.

- [ ] **Step 5: Update Phase 0 checklist from evidence**

Mark “采集代码符合本契约” complete only if Step 3 passed. Mark “数据审计脚本验证新数据符合本契约” complete only if the generated episode audit passed. Leave action direction unchecked until all four sign-probe runs have been inspected.

- [ ] **Step 6: Commit documentation and verification state**

```bash
git add research/ball_wm/README.md research/ball_wm/scripts/collect_data.py research/Ball_Nav/PHASE0_RESEARCH_CONTRACT.md
git commit -m "docs: describe BEV collection workflow"
```

---

## Final verification

- [ ] Run `pytest -q research/ball_wm/tests` and record the pass count.
- [ ] Run `python research/ball_wm/scripts/audit_dataset.py /tmp/ball_wm_bev_smoke` and require exit code 0.
- [ ] Run `git status --short` and confirm no generated data, previews, or unrelated files are staged.
- [ ] Review all implementation commits against `docs/superpowers/specs/2026-08-17-isaacsim-bev-data-collection-design.md` before presenting the run command to the user.
