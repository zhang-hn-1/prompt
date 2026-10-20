# Dynamics-Aware Trajectory Tracker V1 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 新增不替换旧 Oracle 的动力学适配轨迹跟踪策略，并通过确定性复位、开放平面验收和条件式 formal 回归验证它。

**Architecture:** 先消除相机预热造成的 state-0 差异；再用纯 CPU 空间参考剖面提供分段平滑曲率；独立 `TrajectoryOraclePolicy` 使用曲率前馈、有界误差反馈和曲率前瞻降速。只有开放平面门通过后才接入正式 benchmark。

**Tech Stack:** Python、NumPy、pytest、现有 Isaac Sim runtime/DriveModel/action protocol。

**Spec:** `docs/superpowers/specs/2026-09-17-dynamics-aware-trajectory-tracker-design.md`

## Global Constraints

- 工作树 `.worktrees/e1-detour-benchmark`；保留已有 dirty changes，不提交、推送或清理。
- 所有代码、测试、调试、代码审查和 GPU 实验使用 Luna Max；worker 不派生子代理。
- 旧 `oracle`、Hybrid A*、动作协议、PD、地图、0.1 m 裕度、35 s 预算及评分定义保持不变。
- 新策略名固定为 `oracle_trajectory_v1`；不复用 `oracle_v2` timed dynamic reference。
- 每次 GPU case 独立进程、单 GPU owner、唯一 run root；失败保留 partial，不自动重试。
- 不实现延期项，不自动扫参数，不因诊断结果改写旧 raw artifact。

---

### Task 1: 确定性 reset state 0

**Files:**
- Modify: `research/ball_wm/benchmark/isaac_runtime.py`
- Modify: `research/ball_wm/tests/test_isaac_runtime_contract.py`

**Interfaces:**
- Consumes: 现有 `_prime_stereo_cameras`、episode pose 和 `ActionSmoother`。
- Produces: 相机预热结束后的最终 reset 顺序；不改变 `IsaacRuntimeConfig` 或 observation schema。

- [ ] **Step 1: 写 RED fake-world 测试。** 分别模拟相机在 1 步和 6 步后 ready；断言 reset 返回的 root pose、root velocity、joint position/velocity、smoother state 和 previous action 完全相同，相对 clock/state index 都从 0 开始，且预热后仅有一个固定 non-render physics step。
- [ ] **Step 2: 单独运行新测试并确认因当前“预热后未复位”而失败，不接受依赖或 fixture 错误。**
- [ ] **Step 3: 最小实现预热后的 pose/velocity/joint/smoother/action 复位和固定一步；建立 epoch 后 capture，不再次预热相机。**
- [ ] **Step 4: 运行 `test_isaac_runtime_contract.py`、`test_benchmark_runner.py` 与 PID isolation runtime tests，确认旧 5×physics+1×render action block 合同不变。**
- [ ] **Step 5: 保存 dirty-baseline scoped diff、RED/GREEN 日志与实现报告；独立 Luna Max review GO 后才运行 GPU。**
- [ ] **Step 6: GPU 只跑两个相同 `line_forward_0p5` case，核对 state 0 物理字段逐元素相同。若不同，停止并诊断，不进入 Task 2。**

### Task 2: 纯 CPU 空间参考剖面

**Files:**
- Create: `research/ball_wm/benchmark/trajectory_tracking.py`
- Create: `research/ball_wm/tests/test_trajectory_tracking.py`

**Interfaces:**
- Produces: `SpatialReferenceProfile.from_waypoints(waypoints, smoothing_window_m=0.40)`；只读数组 `xy`, `yaw_unwrapped`, `s_m`, `curvature_1_m`, `segment_id`, `drive_direction`, `main_command_rad_s`；以及 `preview_max_abs_curvature(index, distance_m)`。
- Consumes: `tuple[KinodynamicWaypoint, ...]`，不修改 waypoint schema。

- [ ] **Step 1: 写手算 RED 测试。** 覆盖直线曲率 0、R3 左/右弧约 ±1/3、S 弯换号、±π yaw 展开、重复终点、换向分段不跨界、非有限/退化输入拒绝。
- [ ] **Step 2: 运行新测试确认缺失模块导致预期 RED。**
- [ ] **Step 3: 实现弧长、分段 yaw unwrap、中心/单边差分、重复点继承与 0.40 m 同段加权平滑；数组复制并只读。**
- [ ] **Step 4: 增加未来 0.80 m 最大绝对曲率查询测试与实现，严格停止在方向段边界。**
- [ ] **Step 5: 运行新文件和 kinodynamic/tracking-reference 相关 CPU 回归，保存 scoped diff 与独立 review。**

### Task 3: 独立 `TrajectoryOraclePolicy`

**Files:**
- Modify: `research/ball_wm/benchmark/policies.py`
- Modify: `research/ball_wm/tests/test_benchmark_policies.py`
- Consume unchanged: `research/ball_wm/benchmark/trajectory_tracking.py`

**Interfaces:**
- Produces: `TrajectoryOraclePolicy(protocol, *, lookahead_m=1.0, smoothing_window_m=0.40, curvature_preview_m=0.80, lateral_curvature_gain_1_m2=0.20, heading_curvature_gain_1_m=0.15, max_feedback_curvature_1_m=0.20, yaw_rate_feedback_gain_s=0.15, latency_preview_s=0.0)`。
- Diagnostics: reference index/direction/braking fields plus `reference_curvature_1_m`, `feedback_curvature_1_m`, `command_curvature_1_m`, `lateral_error_m`, `heading_error_rad`, `main_cap_rad_s`, `auxiliary_feedforward_rad`, `auxiliary_feedback_rad`。

- [ ] **Step 1: 写 RED policy 测试。** 用 literal reference/observation 验证直线零前馈、R3 曲率前馈符号、横向/航向反馈方向、±0.20 m⁻¹ 限幅、倒车符号、方向切换停稳门、终点制动、所有参数有限性和 `latency_preview_s=0`。
- [ ] **Step 2: 写速度剖面 RED 测试。** 未来曲率 `<.20/.20–.45/>=.45` 分别产生 `0.50/0.35/0.25 rad/s` 上限；误差门产生 0.25 上限；前瞻能在弯道前降速；正反只改变符号。
- [ ] **Step 3: 运行选择性 policy tests 确认新类缺失 RED，随后实现最小类；复用旧投影/换向逻辑时抽取纯 helper，不改变旧类动作输出。**
- [ ] **Step 4: 添加 mutation-oriented compatibility 测试：旧 `OraclePathPolicy` 在冻结 fixture 的动作逐元素等于改动前 literal，策略名仍为 `oracle`。**
- [ ] **Step 5: 运行 policies、trajectory_tracking、DriveModel 与 action protocol 聚焦回归，保存 scoped diff 与独立 review。**

### Task 4: Tracking acceptance 集成与开放平面门

**Files:**
- Modify: `research/ball_wm/scripts/run_tracking_acceptance.py`
- Modify: `research/ball_wm/benchmark/tracking_acceptance.py`（仅当 diagnostics schema 需要）
- Modify: `research/ball_wm/tests/test_tracking_acceptance_harness.py`
- Modify: `research/ball_wm/tests/test_tracking_acceptance.py`（仅对应新字段）

**Interfaces:**
- Consumes: `TrajectoryOraclePolicy` 与 spec 中固定参数。
- Produces: `--policy-variant oracle_trajectory_v1`、child propagation、hashed config/metrics/summary provenance；默认 variant 仍为 baseline。

- [ ] **Step 1: 写 RED CLI/child/config tests，验证每个 V1 参数只从 CLI 解析一次并原样进入 policy 与 case config；非法值失败；baseline 不携带隐式 V1 行为。**
- [ ] **Step 2: 最小集成 policy builder 和 diagnostics artifact；不得复制 run loop。**
- [ ] **Step 3: 运行 tracking acceptance/harness/artifact/runner 聚焦回归，独立 review GO 后冻结源码哈希。**
- [ ] **Step 4: 单 GPU owner 串行运行旧 baseline 与 V1：直线 0.5、左右 R3、S 弯、倒车、mixed；每对先审计 state 0，任何不一致对作废且停止该对解释。**
- [ ] **Step 5: 生成有界中文报告，分别列 arrival、hold、cross RMS/P95/max、roll、主轴目标/实际速度、副轴目标连续性、方向切换与 clock。按 spec 门决定是否进入 Task 5；不得凭单一曲线通过。**

### Task 5: 条件式 formal benchmark 集成与七任务回归

**Files:**
- Modify only after Task 4 gate: `research/ball_wm/scripts/run_pointgoal_benchmark.py`
- Modify only after Task 4 gate: `research/ball_wm/tests/test_benchmark_runner.py`
- Modify only if validation requires: `research/ball_wm/tests/test_benchmark_artifacts.py`

**Interfaces:**
- Produces: formal `--policy oracle_trajectory_v1`，复用普通 `kinodynamic_plan`；完整参数写入 environment/metrics/summary。

- [ ] **Step 1: 若 Task 4 未过门，记录证据并结束本计划，不修改 formal CLI。**
- [ ] **Step 2: 若过门，写 RED dispatch/provenance tests，证明新策略获得 kinodynamic reference、不会请求 dynamic reference，旧 `oracle` 行为不变。**
- [ ] **Step 3: 最小 formal CLI/policy dispatch 实现并运行 benchmark runner/artifact/policy 聚焦回归；独立 Luna Max review。**
- [ ] **Step 4: 唯一 GPU owner 在新 run root 运行冻结 `hard_0` 七任务；不覆盖 lookahead=1.0 的 2/7 基线。**
- [ ] **Step 5: 审计 source/map/manifest/config hash、35 s World clock、逐任务轨迹/碰撞/超时和视频；按 spec 正式门给出结论，不修改判据或事后重评分。**

### Task 6: 收尾验证与研究记录

**Files:**
- Modify: `.superpowers/sdd/2026-09-17-dynamics-aware-trajectory-tracker/progress.md`
- Create: run-root 中文报告与 bounded machine-readable summary。

- [ ] **Step 1: 对本计划所有增量做 dirty-baseline scoped review；重要问题回原实现者 TDD 修复。**
- [ ] **Step 2: 运行计划列出的完整聚焦测试集合、`py_compile` 和 `git diff --check`；不因已知无关动态搜索超时声称全仓全绿。**
- [ ] **Step 3: 记录完成/未执行任务、原始数据路径、参数、证据边界和下一步；不提交、推送或清理用户改动。**
