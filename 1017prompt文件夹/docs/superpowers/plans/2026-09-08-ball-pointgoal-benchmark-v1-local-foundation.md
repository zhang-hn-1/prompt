# 球形机器人 PointGoal Benchmark V1 本地基础实施计划

> **供执行代理使用：** 必须使用 `superpowers:subagent-driven-development`（推荐）或 `superpowers:executing-plans`，按任务逐项实施，并用 `- [ ]` 跟踪进度。

**目标：** 建立可在本地人工审查的 Benchmark V1 基础，包括 `±30°` 动作协议、冻结任务清单、A* 参考路径、sanity/Oracle 基线、GUI/headless 运行器、完整审核包和 3 个经人工批准的 golden 任务。

**架构：** 任务协议、网格规划、指标和策略接口放在 `research/ball_wm/benchmark/` 中，保持可脱离 Isaac Sim 测试。Isaac 导入和物理执行集中到一个延迟加载适配器。动作数值边界只存在于 `data_contract.yaml`；旧模型和旧数据继续按 legacy 协议处理，禁止暗中扩展到 `±30°`。

**技术栈：** Python 3.10、NumPy、SciPy、PyYAML、Pillow、OpenCV、PyTorch、pytest、Isaac Sim 4.5/OpenUSD。

**设计依据：** `docs/superpowers/specs/2026-09-08-ball-pointgoal-benchmark-v1-design.md`

## 全局约束

- V1 只研究静态平面 PointGoal 导航，不加入动态障碍、复杂地形、长期记忆、拓扑图或在线全局 SLAM 地图。
- 核心策略输入为同步双目 RGB、本体历史、完美仿真位置/yaw、base-frame PointGoal 和历史动作。
- 每个结果写入 `localization_source: perfect_localization`；真机 GPS/里程计/yaw 融合不在本计划内。
- 物理/控制/状态频率为 50 Hz，双目和重规划为 10 Hz；一个 `(10,)` action block 包含 5 组 `(main, auxiliary)`。
- 动作协议为 `ball_action_v2_aux30`；主驱动边界 `[-2.0, 2.0] rad/s`，辅助关节边界 `[-0.5235987756, 0.5235987756] rad`。
- 球半径 `0.4 m`，额外安全裕度 `0.20 m`，栅格分辨率 `0.1 m`。
- episode 时限 20 s，即最多 200 个 action block；目标容差 `0.2 m`。
- 终止优先级为 `collision > success > timeout`；基础设施异常单独记为 `runtime_error`。
- Oracle 地图和 A* 始终标记 `privileged: true`，不能流入非特权策略的 observation。
- 缺少 V2 ID 的旧数据/checkpoint 按 `ball_action_v1_aux16` 处理，V2 评测器默认拒绝。
- 正式路径通过 `SCENE_ROOT`、`DATA_ROOT`、`RUN_ROOT` 解析，提交的 manifest 不包含本机绝对路径。
- 本计划只实现本地阶段 1–5。服务器 parity、stereo-depth+DWA 和外部开源方法分别制定后续计划。

---

## 任务 1：建立 `ball_action_v2_aux30` 单一契约源

**文件：**

- 新建：`research/ball_wm/training/action_protocol.py`
- 新建：`research/ball_wm/tests/test_action_protocol.py`
- 修改：`research/ball_wm/configs/data_contract.yaml`
- 修改：`research/ball_wm/scripts/collection_schema.py`
- 修改：`research/ball_wm/tests/test_collection_schema.py`

**输出接口：** `ActionProtocol.from_contract()`、`component_bounds_numpy()`、`block_bounds_numpy()`、`block_bounds_torch()`、`clip_block()`、`sample_block()`。

- [ ] **1.1 先写失败测试**

```python
def test_v2_contract_declares_full_auxiliary_range():
    contract = load_contract(CONTRACT_PATH)
    assert contract.action_protocol_id == "ball_action_v2_aux30"
    assert contract.controls_per_block == 5
    assert contract.action_minimum == pytest.approx((-2.0, -0.5235987756))
    assert contract.action_maximum == pytest.approx((2.0, 0.5235987756))


def test_action_protocol_expands_five_control_pairs():
    protocol = ActionProtocol.from_contract(load_contract(CONTRACT_PATH))
    bounds = protocol.block_bounds_numpy()
    assert bounds.shape == (10, 2)
    np.testing.assert_allclose(
        bounds,
        np.asarray([[-2.0, 2.0], [-0.5235987756, 0.5235987756]] * 5),
    )
```

- [ ] **1.2 运行测试，确认因缺少 V2 字段而失败**

```bash
PYTHONPATH=research/ball_wm /home/an/miniconda3/envs/ball_wm/bin/python -m pytest -q \
  research/ball_wm/tests/test_action_protocol.py \
  research/ball_wm/tests/test_collection_schema.py
```

- [ ] **1.3 修改 YAML 契约**

```yaml
schema_version: "2.0.0"
action:
  protocol_id: ball_action_v2_aux30
  dimension: 2
  controls_per_block: 5
  fields:
    - name: main_joint_velocity_cmd
      index: 0
      control_mode: velocity
      unit: rad_per_s
      minimum: -2.0
      maximum: 2.0
    - name: auxiliary_joint_angle_cmd
      index: 1
      control_mode: position
      unit: rad
      minimum: -0.5235987756
      maximum: 0.5235987756
```

保留现有正方向和导航效果字段。`CollectionContract` 新增 `action_protocol_id: str` 和 `controls_per_block: int`；拒绝空 ID、非正 block 数、非有限边界及 `minimum >= maximum`。

- [ ] **1.4 实现不可变 ActionProtocol**

```python
@dataclass(frozen=True)
class ActionProtocol:
    protocol_id: str
    minimum: tuple[float, float]
    maximum: tuple[float, float]
    controls_per_block: int

    @classmethod
    def from_contract(cls, contract: CollectionContract) -> "ActionProtocol":
        return cls(
            protocol_id=contract.action_protocol_id,
            minimum=tuple(contract.action_minimum),
            maximum=tuple(contract.action_maximum),
            controls_per_block=contract.controls_per_block,
        )

    @property
    def action_block_dim(self) -> int:
        return self.controls_per_block * len(self.minimum)

    def component_bounds_numpy(self) -> np.ndarray:
        return np.stack((self.minimum, self.maximum), axis=1).astype(np.float32)

    def block_bounds_numpy(self) -> np.ndarray:
        return np.tile(self.component_bounds_numpy(), (self.controls_per_block, 1))
```

`clip_block()` 仅接受有限 `(10,)` 数组；`sample_block()` 仅接受 NumPy `Generator` 并返回 float32 `(10,)`。

- [ ] **1.5 运行聚焦测试并提交**

```bash
git add research/ball_wm/configs/data_contract.yaml \
  research/ball_wm/scripts/collection_schema.py \
  research/ball_wm/training/action_protocol.py \
  research/ball_wm/tests/test_action_protocol.py \
  research/ball_wm/tests/test_collection_schema.py
git commit -m "feat: define aux30 action protocol"
```

---

## 任务 2：传递 V2 边界并防止 legacy/V2 混用

**文件：**

- 修改：`collect_data_dubins.py`、`collect_candidate_rollouts.py`、`audit_dataset.py`
- 修改：`planner.py`、`evaluate_navigation.py`、`evaluate_pointgoal.py`、`evaluate_planner_calibration.py`
- 修改：`train_joint.py`、`ball_wm_dataset.py` 及对应测试。

- [ ] **2.1 先写边界传递和协议不匹配测试**

```python
def test_planner_uses_injected_aux30_bounds():
    protocol = ActionProtocol.from_contract(load_contract(CONTRACT_PATH))
    bounds = planner.action_bounds(protocol)
    assert bounds.shape == (10, 2)
    assert torch.allclose(bounds[1::2, 0], torch.full((5,), -0.5235988))
    assert torch.allclose(bounds[1::2, 1], torch.full((5,), 0.5235988))


def test_v2_evaluator_rejects_unlabelled_checkpoint():
    with pytest.raises(ValueError, match="ball_action_v1_aux16"):
        require_checkpoint_protocol({"model": {}}, "ball_action_v2_aux30")
```

同时覆盖：数据根目录不能混合 V1/V2 episode；V2 审核必须报告 `16.1° < |auxiliary| <= 30°` 的样本数。

- [ ] **2.2 运行相关测试，确认当前硬编码被检出**

```bash
PYTHONPATH=research/ball_wm /home/an/miniconda3/envs/ball_wm/bin/python -m pytest -q \
  research/ball_wm/tests/test_collect_data_dubins.py \
  research/ball_wm/tests/test_collect_candidate_rollouts.py \
  research/ball_wm/tests/test_evaluate_navigation.py \
  research/ball_wm/tests/test_planner.py \
  research/ball_wm/tests/test_train_joint.py \
  research/ball_wm/tests/test_ball_wm_dataset.py \
  research/ball_wm/tests/test_audit_dataset.py
```

- [ ] **2.3 修改规划器接口**

```python
def action_bounds(protocol: ActionProtocol) -> torch.Tensor:
    return torch.from_numpy(protocol.block_bounds_numpy())


def plan(
    model: torch.nn.Module,
    context: dict[str, torch.Tensor],
    *,
    action_protocol: ActionProtocol,
    goal_emb: torch.Tensor | None = None,
    horizon: int = 5,
    samples: int = 300,
    elites: int = 30,
    iterations: int = 10,
    safety_weight: float = 0.1,
    history_size: int = 5,
    sigma: float = 1.0,
    device: torch.device | None = None,
    planner_mode: str = "latent",
    goal_relative_xy: torch.Tensor | None = None,
) -> PlanResult:
    bounds = action_bounds(action_protocol).to(
        device or next(model.parameters()).device
    )
```

保留现有其余 cost 参数。所有调用端只加载一次契约并显式传入 protocol。删除 evaluator 的 `_ACTION_MINIMUM/_ACTION_MAXIMUM`；random、clip 和 calibration primitive 全部调用 protocol。

- [ ] **2.4 版本化数据和 checkpoint**

episode metadata、run manifest 和 checkpoint 都写入 `action_protocol_id` 与 `controls_per_block`。缺少 ID 时只识别为 `ball_action_v1_aux16`。`BallWMDataset` 拒绝混合协议并公开 `dataset.action_protocol_id`。

`audit_dataset.py` 使用 5° 固定区间输出 auxiliary 直方图、`high_angle_sample_count` 和 `high_angle_fraction`；V2 正式采集中 high-angle 样本为 0 时失败。

- [ ] **2.5 扫描重复常量**

```bash
rg -n "0\.2809658115|-0\.2809658115|0\.5235987756|-0\.5235987756" \
  research/ball_wm --glob '*.py' --glob '*.yaml'
```

生产代码中 V2 数值只能出现在 `data_contract.yaml`，测试可保留期望值。

- [ ] **2.6 运行聚焦测试并提交本任务文件**

```bash
git commit -m "refactor: propagate versioned action bounds"
```

---

## 任务 3：建立低速 `±30°` 物理验收门

**文件：**

- 新建：`research/ball_wm/scripts/probe_auxiliary_limits.py`
- 新建：`research/ball_wm/tests/test_probe_auxiliary_limits.py`
- 修改：`research/ball_wm/README.md`

- [ ] **3.1 先写纯 Python 判定测试**

```python
def test_evaluate_probe_accepts_symmetric_aux30_tracking():
    result = evaluate_probe(
        commanded_rad=np.asarray([0.5235988, -0.5235988]),
        measured_rad=np.asarray([0.520, -0.521]),
        yaw_delta_rad=np.asarray([0.18, -0.17]),
        max_abs_roll_rad=np.asarray([0.08, 0.09]),
        collision=np.asarray([False, False]),
    )
    assert result["passed"] is True
```

另写超限、同向转弯、侧倾过大、碰撞和非有限输入测试。通过标准：实测关节 `<=30.5°`，跟踪误差 `<=3°`，左右 yaw 符号相反且幅值均 `>=3°`，机体 roll `<=20°`，无碰撞。

- [ ] **3.2 实现延迟加载 Isaac 的探针**

```python
PROBE_CONTROLS = (
    ("positive", 0.5, math.radians(30.0)),
    ("negative", 0.5, math.radians(-30.0)),
)
SETTLE_STEPS = 50
DRIVE_STEPS = 100
```

每个方向重置到 `[0, 0, 0.45]`，稳定 1 s，50 Hz 低速行驶 2 s，逐步记录 joint2、yaw、roll、接触和轨迹；输出 `steering_probe.json` 与 `steering_probe.png`。

- [ ] **3.3 在用户可见 GUI 终端运行**

```bash
PYTHONPATH=research/ball_wm /home/an/miniconda3/envs/ball_wm/bin/python \
  research/ball_wm/scripts/probe_auxiliary_limits.py \
  --contract research/ball_wm/configs/data_contract.yaml \
  --out "${RUN_ROOT}/benchmark_v1/steering_probe" --gui
```

必须同时满足 JSON `passed: true` 和用户视觉确认；任一失败时停止正式采集。

- [ ] **3.4 运行测试并提交**

```bash
git add research/ball_wm/scripts/probe_auxiliary_limits.py \
  research/ball_wm/tests/test_probe_auxiliary_limits.py \
  research/ball_wm/README.md
git commit -m "test: gate aux30 steering behavior"
```

---

## 任务 4：实现不可静默修改的 benchmark manifest

**文件：**

- 新建：`research/ball_wm/benchmark/__init__.py`
- 新建：`research/ball_wm/benchmark/manifest.py`
- 新建：`research/ball_wm/tests/test_benchmark_manifest.py`
- 新建：`research/ball_wm/configs/benchmark_v1.yaml`

- [ ] **4.1 先写 schema、路径、哈希和审批测试**

```python
def test_manifest_requires_approved_tasks_before_freeze(tmp_path: Path):
    manifest = candidate_manifest(review_status="needs_adjustment")
    with pytest.raises(ValueError, match="approved"):
        freeze_manifest(manifest, tmp_path / "golden_manifest.json")


def test_scene_uri_cannot_escape_scene_root(tmp_path: Path):
    episode = candidate_episode(
        scene_uri="${SCENE_ROOT}/../private.usd"
    )
    with pytest.raises(ValueError, match="escapes SCENE_ROOT"):
        episode.resolve_scene(tmp_path)
```

测试 helper 固定使用一个合法 E1 episode：3 m 起终距离、单位四元数、合法 64 位 SHA256、seed 3072。另覆盖重复 ID、非有限数据、绝对路径、scene 哈希不匹配和 manifest 自哈希不匹配。

- [ ] **4.2 实现类型化记录**

```python
@dataclass(frozen=True)
class HumanReview:
    status: Literal["approved", "rejected", "needs_adjustment"]
    reviewer: str
    notes: str


@dataclass(frozen=True)
class BenchmarkEpisode:
    episode_id: str
    split: Literal["golden", "dev", "test"]
    difficulty: Literal["E0", "E1", "E2"]
    scene_uri: str
    scene_sha256: str
    occupancy_sha256: str
    start_position_world_m: tuple[float, float, float]
    start_quaternion_world_wxyz: tuple[float, float, float, float]
    goal_position_world_m: tuple[float, float, float]
    euclidean_distance_m: float
    oracle_path_length_m: float
    seed: int
    human_review: HumanReview
    classification_evidence: Mapping[str, object]
```

`BenchmarkManifest` 还包含 schema/version、`action_protocol_id`、`localization_source`、episode 列表和 `manifest_sha256`。Canonical JSON 使用 sorted keys、UTF-8、compact separators、`allow_nan=False`，计算时排除自身哈希字段。

- [ ] **4.3 新增固定 benchmark 配置**

```yaml
schema_version: ball_pointgoal_benchmark_config_v1
benchmark_version: v1
action_protocol_id: ball_action_v2_aux30
localization_source: perfect_localization
ball_radius_m: 0.4
safety_margin_m: 0.2
occupancy_resolution_m: 0.1
goal_tolerance_m: 0.2
episode_budget_s: 20.0
physics_hz: 50
control_hz: 50
stereo_hz: 10
replan_hz: 10
initial_goal_distance_m: [1.5, 4.0]
e1_min_detour_ratio: 1.15
e2_extra_width_range_m: [0.0, 0.4]
```

- [ ] **4.4 运行测试并提交**

```bash
PYTHONPATH=research/ball_wm /home/an/miniconda3/envs/ball_wm/bin/python -m pytest -q \
  research/ball_wm/tests/test_benchmark_manifest.py
git commit -m "feat: add immutable benchmark manifests"
```

---

## 任务 5：实现栅格 A* 和 E0/E1/E2 有效性判定

**文件：**

- 新建：`research/ball_wm/benchmark/grid_planning.py`
- 新建：`research/ball_wm/benchmark/task_generation.py`
- 新建：`research/ball_wm/tests/test_grid_planning.py`
- 新建：`research/ball_wm/tests/test_benchmark_task_generation.py`

- [ ] **5.1 先写障碍膨胀和 A* 测试**

```python
def test_astar_detours_without_diagonal_corner_cutting():
    grid = np.zeros((9, 9), dtype=np.uint8)
    grid[4, 2:7] = 1
    path = astar_path(grid, start=(1, 4), goal=(7, 4), occupied=1)
    assert tuple(path[0]) == (1, 4)
    assert tuple(path[-1]) == (7, 4)
    assert not np.any(grid[path[:, 0], path[:, 1]] == 1)
    assert path_length_cells(path) > 6.0


def test_inflation_uses_ball_plus_margin():
    grid = np.zeros((21, 21), dtype=np.uint8)
    grid[10, 10] = 1
    inflated = inflate_obstacles(
        grid, radius_m=0.6, resolution_m=0.1, occupied=1
    )
    assert inflated[16, 10] == 1
    assert inflated[17, 10] == 0
```

- [ ] **5.2 先写任务分类测试**

E0 必须直线可行；E1 必须直线受阻且 A*/欧氏比 `>=1.15`；E2 的 A* 必须经过窄通道 mask。起点/终点碰撞、地图外、corner cutting 和不可达均写独立测试。

- [ ] **5.3 实现确定性 A***

使用 8 邻域，代价 `1`/`sqrt(2)`，欧氏 heuristic，heap 使用单调计数器解决 tie。对角移动时两个相邻正交格均须可通行。障碍膨胀半径固定为 `0.4 + 0.2 = 0.6 m`。

- [ ] **5.4 输出可审计分类证据**

```python
@dataclass(frozen=True)
class TaskEvidence:
    line_is_free: bool
    euclidean_distance_m: float
    oracle_path_length_m: float
    detour_ratio: float
    start_clearance_m: float
    goal_clearance_m: float
    corridor_intersection: bool
    corridor_width_m: float | None
```

`validate_task()` 必须返回该结构，调用端不能绕过验证直接指定难度标签。

- [ ] **5.5 运行测试并提交**

```bash
PYTHONPATH=research/ball_wm /home/an/miniconda3/envs/ball_wm/bin/python -m pytest -q \
  research/ball_wm/tests/test_grid_planning.py \
  research/ball_wm/tests/test_benchmark_task_generation.py
git commit -m "feat: validate benchmark tasks with astar"
```

---

## 任务 6：生成任务卡并建立人工审批流程

**文件：**

- 新建：`research/ball_wm/benchmark/task_cards.py`
- 新建：`research/ball_wm/scripts/generate_benchmark_tasks.py`
- 新建：`research/ball_wm/tests/test_benchmark_task_cards.py`
- 修改：`research/ball_wm/scripts/trajectory_map.py`

- [ ] **6.1 先写图层完整性和 freeze 拒绝测试**

渲染器除输出 `1200x900` PNG 外，还必须返回含以下键的 layer manifest：`obstacles`、`inflated`、`start_heading`、`goal`、`straight_line`、`oracle_path`、`metrics`。存在任何非 `approved` 任务时，freeze 返回码 2 且不产生输出。

- [ ] **6.2 扩展现有轨迹图组件**

颜色固定为：障碍深灰、安全膨胀半透明红、直线虚线琥珀色、A* 蓝色、起点与朝向绿色、目标品红。保持现有 collector 调用签名兼容。

- [ ] **6.3 实现 CLI 子命令**

```text
export-scene --scene-uri URI --scene-root PATH --out OCCUPANCY_NPZ
generate --scene-index JSON --split golden|dev --count-per-class N --seed 3072 --out JSON
render --candidates JSON --cards-dir DIRECTORY
review --candidates JSON --episode-id ID --status approved|rejected|needs_adjustment --reviewer NAME --notes TEXT
freeze --candidates JSON --out MANIFEST_JSON
```

`export-scene` 是唯一启动 Isaac 的子命令。review 原子替换 candidate JSON，并保留所有 rejected 记录。

- [ ] **6.4 运行测试并提交**

```bash
PYTHONPATH=research/ball_wm /home/an/miniconda3/envs/ball_wm/bin/python -m pytest -q \
  research/ball_wm/tests/test_benchmark_task_cards.py \
  research/ball_wm/tests/test_trajectory_map.py
git commit -m "feat: add benchmark task review cards"
```

---

## 任务 7：实现统一策略接口和基础对照组

**文件：**

- 新建：`research/ball_wm/benchmark/policies.py`
- 新建：`research/ball_wm/tests/test_benchmark_policies.py`

- [ ] **7.1 先写权限、shape、边界和行为测试**

```python
def test_oracle_requires_privileged_context():
    policy = OraclePathPolicy(protocol=aux30_protocol())
    with pytest.raises(PermissionError, match="privileged"):
        policy.reset(non_privileged_context())


def test_goal_greedy_changes_turn_sign_with_goal_side():
    policy = GoalGreedyPolicy(protocol=aux30_protocol())
    policy.reset(non_privileged_context())
    left = policy.act(valid_observation(), np.asarray([2.0, 1.0]))
    right = policy.act(valid_observation(), np.asarray([2.0, -1.0]))
    assert np.sign(left[1]) == -np.sign(right[1])
    assert left.shape == right.shape == (10,)
```

测试 fixtures 固定为 V2 契约、640x480 零值 RGB、17 维本体历史、零动作历史和单位朝向位姿。另覆盖 stop 严格为零、random 同 seed 完全可复现、Oracle 路径索引单调前进，以及非 Oracle 无法读取 occupancy/path。

- [ ] **7.2 实现 observation/context 边界**

```python
@dataclass(frozen=True)
class PolicyObservation:
    stereo_left_rgb: np.ndarray
    stereo_right_rgb: np.ndarray
    proprio_history: np.ndarray
    action_history: np.ndarray
    position_world_xy: np.ndarray
    yaw_world_rad: float


@dataclass(frozen=True)
class EpisodeContext:
    episode_id: str
    privileged: bool
    oracle_path_xy: np.ndarray | None
    action_protocol_id: str
```

runner 自己保留 occupancy 与碰撞真值，不把它们放进 `PolicyObservation`。

- [ ] **7.3 实现 Stop/Random/GoalGreedy/OraclePath**

- Stop：重复 5 组 `(0, 0)`。
- Random：从 V2 边界按固定 seed 采样。
- GoalGreedy：由 base-frame PointGoal 朝向误差转向，距离小于 `0.6 m` 时降速。
- OraclePath：使用 A* 路径上 `0.5 m` lookahead，经共享几何 tracker 生成一组控制并重复 5 次。

- [ ] **7.4 运行测试并提交**

```bash
PYTHONPATH=research/ball_wm /home/an/miniconda3/envs/ball_wm/bin/python -m pytest -q \
  research/ball_wm/tests/test_benchmark_policies.py
git commit -m "feat: add benchmark policy adapters"
```

---

## 任务 8：实现纯 episode runner 和 scene-cluster 置信区间

**文件：**

- 新建：`research/ball_wm/benchmark/runner.py`
- 新建：`research/ball_wm/tests/test_benchmark_runner.py`
- 修改：`research/ball_wm/training/navigation_metrics.py`
- 修改：`research/ball_wm/tests/test_navigation_metrics.py`

- [ ] **8.1 先写终止优先级和时限测试**

```python
def test_collision_wins_when_goal_is_reached_on_same_step():
    terminal = episode_termination(
        distance_m=0.1,
        collision=True,
        step=1,
        max_steps=200,
        goal_tolerance_m=0.2,
    )
    assert terminal == "collision"


def test_twenty_seconds_is_two_hundred_action_blocks():
    config = RunnerConfig(
        episode_budget_s=20.0,
        replan_hz=10,
        goal_tolerance_m=0.2,
    )
    assert config.max_action_blocks == 200
```

- [ ] **8.2 先写按场景 bootstrap 测试**

```python
def test_cluster_bootstrap_resamples_scenes():
    episodes = [
        {"scene_id": "a", "success": 1.0},
        {"scene_id": "a", "success": 1.0},
        {"scene_id": "b", "success": 0.0},
    ]
    result = cluster_bootstrap_interval(
        episodes,
        field="success",
        cluster_field="scene_id",
        seed=3072,
        samples=200,
    )
    assert result["num_clusters"] == 2
    assert result["estimate"] == pytest.approx(2.0 / 3.0)
```

- [ ] **8.3 实现固定闭环顺序**

每次循环依次执行：根据完美位姿计算 base-frame PointGoal；调用 policy；校验 V2 shape/边界；执行 5 个物理控制；记录 action/state；按固定优先级判定终止。异常 episode 保留在汇总分母中。

运行时接口提供 `reset(BenchmarkEpisode) -> RuntimeStep`、`execute(action_block) -> RuntimeStep` 和 `close()`。`RuntimeStep` 包含策略 observation、truth clearance、物理 contact、collision 和用于视频的帧。

- [ ] **8.4 实现按 scene ID 重采样的汇总**

每次 bootstrap 先有放回采样 scene ID，再加入该 scene 下全部 episode。直接从原始 episode 计算 Success、Collision、SPL、Timeout、Final Distance 和 Path Length，不对 summary 二次平均。

- [ ] **8.5 运行测试并提交**

```bash
PYTHONPATH=research/ball_wm /home/an/miniconda3/envs/ball_wm/bin/python -m pytest -q \
  research/ball_wm/tests/test_benchmark_runner.py \
  research/ball_wm/tests/test_navigation_metrics.py
git commit -m "feat: add strict benchmark episode runner"
```

---

## 任务 9：实现 Isaac GUI/headless 适配器和完整审核包

**文件：**

- 新建：`research/ball_wm/benchmark/isaac_runtime.py`
- 新建：`research/ball_wm/benchmark/artifacts.py`
- 新建：`research/ball_wm/scripts/run_pointgoal_benchmark.py`
- 新建：`research/ball_wm/scripts/review_benchmark_episode.py`
- 新建：`research/ball_wm/tests/test_benchmark_artifacts.py`

- [ ] **9.1 先写审核包完整性测试**

```python
REQUIRED_FILES = {
    "manifest.json",
    "environment.json",
    "task_card.png",
    "third_person.mp4",
    "stereo_view.mp4",
    "trajectory_overlay.png",
    "actions.npz",
    "states.npz",
    "metrics.json",
    "human_review.json",
}


def test_incomplete_bundle_cannot_publish(tmp_path: Path):
    writer = AuditBundleWriter(tmp_path, "golden_e0_0001")
    writer.partial_dir.mkdir(parents=True)
    (writer.partial_dir / "metrics.json").write_text("{}")
    with pytest.raises(ValueError, match="missing audit files"):
        writer.publish()
    assert not writer.output_dir.exists()
```

另覆盖：不覆盖已有输出、失败后保留 `.partial`、NPZ 数值有限、JSON 不含 NaN、视频非空、轨迹图图层完整，以及行为审核不改变 metrics/actions/states 字节。

- [ ] **9.2 实现 IsaacBenchmarkRuntime**

```python
@dataclass(frozen=True)
class IsaacRuntimeConfig:
    scene_root: Path
    contract_path: Path
    gui: bool
    record_video: bool
    ball_radius_m: float = 0.4
```

根据 `gui` 创建 `SimulationApp`，解析 scene URI，加载现有球形机器人 USD，创建双目和第三人称相机，构建仅供碰撞审计的真值 occupancy，并从 manifest 位姿以零速度重置。策略只获得 RGB、本体、位姿和历史动作。

- [ ] **9.3 实现 GUI 标记和 10 Hz 视频**

创建不参与碰撞的起点、目标和 Oracle 路径标记。第三人称视频与左右双目拼接视频使用 OpenCV `mp4v`、10 FPS；episode 开始前必须确认 `VideoWriter.isOpened()`。正式 golden/dev/test 运行不可关闭录像。

- [ ] **9.4 实现原子审核包**

先写入 `episode_<id>.partial`，验证十个必需文件和哈希后，以 no-replace 方式发布。轨迹图必须包含真值障碍/膨胀区、A*、实际轨迹、起点、目标、终点和碰撞点。

初始行为审核文件为：

```json
{
  "status": "pending",
  "reviewer": "",
  "labels": [],
  "notes": ""
}
```

- [ ] **9.5 实现运行与行为审核 CLI**

运行器参数：

```text
--manifest PATH --split golden|dev|test --policy stop|random|goal_greedy|oracle
--scene-root PATH --run-root PATH --gui|--headless --record-video
--episode-id ID --seed 3072
```

行为审核参数：

```text
--episode-dir PATH --status approved|needs_investigation --reviewer NAME
--labels goal_response|active_avoidance|contact|stuck|oscillation|termination_mismatch|technically_invalid
--notes TEXT
```

行为审核只允许原子替换 `human_review.json`，替换前后核对其他文件 SHA256 未变化。失败策略的 episode 也必须保留。

- [ ] **9.6 运行纯 Python 测试并提交**

```bash
PYTHONPATH=research/ball_wm /home/an/miniconda3/envs/ball_wm/bin/python -m pytest -q \
  research/ball_wm/tests/test_benchmark_artifacts.py \
  research/ball_wm/tests/test_benchmark_runner.py
git commit -m "feat: run auditable Isaac benchmark episodes"
```

---

## 任务 10：生成、审查、冻结并验证本地任务集

**文件：**

- 新建：`research/ball_wm/configs/benchmark_v1/scene_index.json`
- 人工批准后新建：`research/ball_wm/configs/benchmark_v1/golden_manifest.json`
- 新建：`research/ball_wm/configs/benchmark_v1/dev_candidates.json`
- 修改：`research/ball_wm/README.md`

- [ ] **10.1 导出场景 occupancy 并生成固定候选集**

对选中的 `cluttered_easy` 场景执行 `export-scene`，记录 scene/occupancy 哈希和窄通道 mask。使用 seed 3072 生成 3 个 golden 候选和 30 个 dev 候选；scene URI 采用 `${SCENE_ROOT}/cluttered_easy/easy_N/cluttered-N.usd`，occupancy NPZ 不提交 Git。

```bash
PYTHONPATH=research/ball_wm /home/an/miniconda3/envs/ball_wm/bin/python \
  research/ball_wm/scripts/generate_benchmark_tasks.py generate \
  --scene-index research/ball_wm/configs/benchmark_v1/scene_index.json \
  --split golden --count-per-class 1 --seed 3072 \
  --out "${RUN_ROOT}/benchmark_v1/golden_candidates.json"
```

dev 使用 `--split dev --count-per-class 10`，输出到 `research/ball_wm/configs/benchmark_v1/dev_candidates.json`。

- [ ] **10.2 渲染 3 张 golden 和 30 张 dev 任务卡**

```bash
PYTHONPATH=research/ball_wm /home/an/miniconda3/envs/ball_wm/bin/python \
  research/ball_wm/scripts/generate_benchmark_tasks.py render \
  --candidates "${RUN_ROOT}/benchmark_v1/golden_candidates.json" \
  --cards-dir "${RUN_ROOT}/benchmark_v1/golden_cards"
```

- [ ] **10.3 停止并由项目负责人逐张审核 golden 卡片**

检查起点、朝向、目标、障碍、安全膨胀区、直线、A*、分类指标，以及画面是否真正要求对应 E0/E1/E2 行为。拒绝项保留记录；替换任务使用新 episode ID。

- [ ] **10.4 仅冻结已批准的三个 golden 任务**

```bash
PYTHONPATH=research/ball_wm /home/an/miniconda3/envs/ball_wm/bin/python \
  research/ball_wm/scripts/generate_benchmark_tasks.py freeze \
  --candidates "${RUN_ROOT}/benchmark_v1/golden_candidates.json" \
  --out research/ball_wm/configs/benchmark_v1/golden_manifest.json
```

冻结结果必须恰好包含一个 E0、一个 E1、一个 E2，并能通过自身 SHA256 重新加载。

- [ ] **10.5 在 GUI 中运行三个 Oracle golden episode**

```bash
PYTHONPATH=research/ball_wm /home/an/miniconda3/envs/ball_wm/bin/python \
  research/ball_wm/scripts/run_pointgoal_benchmark.py \
  --manifest research/ball_wm/configs/benchmark_v1/golden_manifest.json \
  --split golden --policy oracle \
  --scene-root "${SCENE_ROOT}" \
  --run-root "${RUN_ROOT}/benchmark_v1/golden_oracle" \
  --gui --record-video --seed 3072
```

三个任务必须全部 success、无 collision/runtime error；E1 可见明确绕障，E2 可见穿过指定通道。用户审核双目/第三人称视频和轨迹图，再用行为审核 CLI 记录判断。

- [ ] **10.6 在同一 manifest 上运行 sanity baselines**

依次运行 stop、goal-greedy，以及 random seeds 3072/3073/3074。stop 必须 0 success；random 不能解释 Oracle 成功。任何方法失败都保留原 episode。

- [ ] **10.7 完整回归验证**

```bash
PYTHONPATH=research/ball_wm /home/an/miniconda3/envs/ball_wm/bin/python -m pytest -q research/ball_wm/tests
/home/an/miniconda3/envs/ball_wm/bin/python -m compileall -q research/ball_wm
git diff --check
```

- [ ] **10.8 记录本地工作流并提交小型配置**

README 记录环境变量、转向验收、任务审核、freeze、GUI/headless、行为标签和指标汇总。只提交代码、文档、小型配置及批准的 JSON；不提交 occupancy、视频、数据、runs 或 checkpoint。

```bash
git commit -m "docs: publish local benchmark golden protocol"
```

---

## 本地基础完成门槛

以下条件全部成立后，才能开始服务器 parity 或 stereo-depth+DWA：

1. 低速 `±30°` 仿真探针通过，并经用户目视确认。
2. golden manifest 恰好包含一个批准的 E0/E1/E2，所有哈希可复核。
3. Oracle 在三个任务上全部成功，无碰撞和运行错误。
4. E1 视频显示主动绕障，E2 视频显示通过指定通道。
5. Stop 成功率为 0，Random 未产生误导性结果。
6. 每个 golden episode 均有十个审核文件和完整行为审核。
7. 全量 pytest、compileall 和 `git diff --check` 全部通过。
