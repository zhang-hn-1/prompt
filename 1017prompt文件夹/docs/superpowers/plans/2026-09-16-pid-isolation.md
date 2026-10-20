# PID Isolation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 用固定目标与路径闭环的对照定位副轴 PD、限速和球体/路径反馈对振荡的贡献。

**Architecture:** 复用已校验的 benchmark runtime，新增默认关闭的 50 Hz 被动记录和 diagnostic-only aux gain override；独立 CLI 提供固定时序策略及现有直线路径跟踪，实验与正式评分分离。

**Tech Stack:** Python、NumPy、pytest、现有 Isaac Sim / OpenCV / matplotlib。

**Spec:** `docs/superpowers/specs/2026-09-16-pid-isolation-design.md`

## Global Constraints

- 工作树 `.worktrees/e1-detour-benchmark`，保留既有 dirty changes，不提交、推送或清理。
- 每个 case 新建独立 Isaac 进程，GPU 单一 owner。原始输出不可覆盖，失败保留 partial 与异常信息。
- 动作协议 `ball_action_v2_aux30`；主轴加速度 0.5 rad/s²、主轴 D=80；副轴基线 Kp=300、Kd=60、目标速率 30°/s、上下限 ±30°。
- 每块 5 次 `world.step(render=False)`，之后一次独立 `world.render()`；50 Hz 状态必须来自真实 World 时钟，不用计数器冒充实测时间。
- 不改变正式地图、0.1 m 裕度、0.2 m 到达标准、35 s 正式预算或 DriveModel 文件。PD 实验值不成为默认值，也不得静默重写标定。
- 代码、调试、测试、GPU 实验与代码审查均使用 Luna Max；父任务只做设计、证据审查与文档。不允许 worker 派生子代理。

### Task 1: 被动逐步记录与隔离实验入口

**Files:**
- Modify: `research/ball_wm/benchmark/isaac_runtime.py`
- Create: `research/ball_wm/scripts/run_pid_isolation.py`
- Modify: `research/ball_wm/tests/test_isaac_runtime_contract.py`
- Create: `research/ball_wm/tests/test_pid_isolation.py`
- Reuse unchanged: `scripts/run_tracking_acceptance.py` 中的 geometry/fixture/episode helper、现有 run_episode、write_rgb_video。

**Interfaces:**
- `IsaacRuntimeConfig` 增加 `diagnostic_physics_trace: bool=False`、`diagnostic_auxiliary_kp: float|None=None`、`diagnostic_auxiliary_kd: float|None=None`。
- runtime 暴露复制出的 `physics_diagnostics`（字段名到 ndarray 的字典），以及用于配置记录的关节/增益读回信息；由 reset 清空，execute 每个物理步后追加，不改变默认模式。
- 新入口暴露 `command_for_block(case_id: str, block_index: int) -> tuple[float,float]` 和 `case_duration_s(case_id: str) -> float` 给固定时序案例；未知 case、负索引及超出预算索引拒绝。纯 CPU 函数不导入 Isaac。
- CLI 需要 `--case-id`、`--run-root`、`--aux-kp`、`--aux-kd`（默认基线）、headless 默认/可选 GUI；单次调用只跑一个 case，不自动扫参。closed-loop 两个 case 的前视值由 case 定义，不能另外扫描。
- 使用 spec 的 7 个 case、时序、统计与产物合同。固定时序的 phase 和 requested action 由 schedule 决定，不能来自未来观测。

- [ ] **Step 1: 先写有具体行为断言的失败测试并观察 RED。** 时序边界独立手算，例如：

```python
assert command_for_block("forward_zero", 19) == (0.0, 0.0)
assert command_for_block("forward_zero", 20) == (0.5, 0.0)
assert command_for_block("forward_zero", 220) == (0.0, 0.0)
assert command_for_block("forward_pos10_return", 80) == pytest.approx((0.5, math.pi / 18))
assert command_for_block("forward_neg10_return", 160) == (0.5, 0.0)
assert case_duration_s("hold_zero") == 20.0
```

运行 `PYTHONPATH=research/ball_wm /home/an/miniconda3/envs/ball_wm/bin/python -m pytest -q research/ball_wm/tests/test_pid_isolation.py`。必须因缺失行为失败，而不是路径/依赖错误。

- [ ] **Step 2: 扩展现有 fake-world 的执行测试，确认 5 个真实 post-step 样本、一次 render、请求与应用指令分离、数组复制与 reset 清空。** 使用手算 fake World 初始时刻 8.0 s / step 400、epoch 相同，单块后 physics raw time 应为 `[8.02,8.04,8.06,8.08,8.10]`、relative step `[1,2,3,4,5]`；关节测量来自 fake robot 指定值而非目标。关闭 diagnostics 的旧测试必须仍通过。覆盖诊断外 gain override 拒绝、NaN/inf/0/bool 拒绝，实际控制器接收基线或选择的 gains。

- [ ] **Step 3: 实现最小 runtime 透传。** 执行顺序必须为：

```python
applied = smoother.step(target, minimum, maximum)
controller.apply_action(ArticulationAction(
    joint_velocities=np.asarray([applied[0], np.nan, np.nan], dtype=np.float32),
    joint_positions=np.asarray([np.nan, applied[1], np.nan], dtype=np.float32),
))
world.step(render=False)
# Only with diagnostic_physics_trace: sample actual World/robot getters here.
# After exactly five iterations: world.render(), existing block capture.
```

不复制 execute 到新 runtime、不更改积分/渲染步长。新实验 CLI 只做 schedule/policy/config/artifacts 编排，所有物理执行仍调用现有 runtime。

- [ ] **Step 4: CPU 测试统计和产物路径。** 人工构造恒定目标/实际角相同的数据，应得到误差 RMS=0、末段 peak-to-peak=0；零目标百分比超调必须为 null；故意超调、未稳定信号不能报告 settling=0。测试真实时钟跳步拒绝、已有目录不覆盖、异常在 close 前写入 partial。解析两种路径闭环的 case 定义与固定时序策略不能混淆。

- [ ] **Step 5: 运行聚焦回归并自审，保存 scoped dirty diff。**

```bash
PYTHONPATH=research/ball_wm /home/an/miniconda3/envs/ball_wm/bin/python -m pytest -q research/ball_wm/tests/test_pid_isolation.py research/ball_wm/tests/test_isaac_runtime_contract.py research/ball_wm/tests/test_tracking_acceptance_harness.py research/ball_wm/tests/test_benchmark_runner.py
```

不要运行全仓测试或 GPU；已有全仓动态搜索超时不在本任务范围。记录 RED/GREEN、文件列表、潜在风险，等待独立 spec/quality review。不提交 Git。

### Task 2: 串行基线隔离实验与证据审计

**Files:** 新的 `research/ball_wm/runs/pid_isolation_baseline_20260916/`，不修改源码。

**Interfaces:** 消费 Task 1 CLI / artifact schema；产出逐 case 时钟、参数、响应统计与中文分析。

- [ ] review GO 后，唯一 GPU owner 按 spec 表格顺序串行执行 7 个基线 case，各独立进程，初始全部 aux Kp=300/Kd=60。
- [ ] 每例先报告 exit、实际时长、样本数/时钟、是否有异常，不等待长报告。出错保存 partial，不自动重试或改参。
- [ ] 保存前后源码/config/asset 哈希、reset 状态差异、gain 读回、raw/application/actual 时间曲线、主轴/roll/yaw-rate 对比。
- [ ] 先比较固定指令与路径反馈，区分限速延迟、跟踪延迟与球体振荡。所有推断注明证据边界；不把 roll 平均倾角或恒定转角的 yaw 改变判为失稳。

### Task 3: 证据驱动的有限 PD 对照和收尾

**Files:** 新对照 run root、分析报告；没有证据不改生产默认值。

**Interfaces:** 消费 Task 2 证据；父任务明确选择单个参数与代表 case 后，GPU owner 才能执行。

- [ ] 若固定指令下有关节/球体持续摆振，父任务明确一个 single-variable gain variant；比较相同 case 的应用目标到实际响应、roll 与延迟。不自动调主轴或限速，不超过两种非基线 gain 组合。
- [ ] 若固定指令稳定而路径反馈振荡显著，不强行扫描 PD：报告外环/延迟匹配证据和最小下一步；该分支也属于诊断完成。
- [ ] 候选 PD 如有改善，至少复测受影响固定 case 与闭环直线；标定仍标记待复查，不推广成正式 baseline。
- [ ] 对本任务增量做最终审查与 scoped verification，输出中文报告、曲线和结论，明确未解决事项。原有七任务导航 2/7 结果不因本轮诊断而改写。
