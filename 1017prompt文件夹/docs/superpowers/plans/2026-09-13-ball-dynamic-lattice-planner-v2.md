# 球形机器人动态状态格规划器 V2 实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (- [ ]) syntax for tracking.

**Goal:** 实现显式考虑主轴加速度、副轴变化速率和停稳换向的动态状态格 Oracle V2，并在冻结的 35 秒 Benchmark 上达到可审计的导航门槛。

**Architecture:** 保留 V1 oracle 的完整语义，新建连续动力学传播与状态格搜索模块，并新增 oracle_v2 跟踪策略。搜索枚举离散控制目标，运动原语内部以 50 Hz 连续传播；Isaac 执行层读取带时间参考，使用前馈加误差反馈输出连续动作。

**Tech Stack:** Python 3.10、NumPy、SciPy 距离场、pytest、Isaac Sim 4.5、现有 DriveModel、Ball PointGoal Benchmark V1.1。

**Spec:** docs/superpowers/specs/2026-09-13-ball-dynamic-lattice-planner-v2-design.md

## Global Constraints

- 保持 ball_action_v2_aux30：主轴 [-2, 2] rad/s，副轴 [-30°, 30°]，每个动作块五个控制值对。
- 保持 ball_pointgoal_v1_1_35s、35 秒预算、0.2 m 目标容差和 0.1 m safety margin。
- V1 oracle、KinodynamicPlanResult 和历史产物语义不得改变。
- 非特权策略不得读取 occupancy、truth clearance、碰撞标签或 Oracle 参考。
- V2 单任务规划上限 60 秒，规划时间不计入执行预算但必须进入审计指标。
- 动态参考预测时长不得超过 32 秒。
- 当前未提交的 V1 限速与预瞄实验不得混入 V2 正式提交。
- 每项生产代码变更都先运行对应失败测试，再写最小实现。

---

### Task 1: 清理失败实验并补齐连续速度接口

**Files:**
- Restore: research/ball_wm/benchmark/policies.py
- Restore: research/ball_wm/benchmark/runner.py
- Restore: research/ball_wm/benchmark/artifacts.py
- Restore: research/ball_wm/scripts/run_pointgoal_benchmark.py
- Restore: research/ball_wm/tests/test_benchmark_policies.py
- Modify: research/ball_wm/benchmark/drive_model.py
- Test: research/ball_wm/tests/test_drive_model.py

**Interfaces:**
- Consumes: DriveModel.steering_samples 中完整的带符号主轴/副轴稳态矩阵。
- Produces: DriveModel.speed(main_command_rad_s: float, auxiliary_angle_rad: float) -> float。

- [ ] **Step 1: 删除未通过的实验差异**

仅撤销 steering_transition 编码、0.25 rad/s 限速、转向换向预瞄分段及其测试；保留提交 5df7560 的执行诊断和提交 4cd54bc 的设计。先检查：

    git diff -- research/ball_wm/benchmark/policies.py research/ball_wm/benchmark/runner.py research/ball_wm/benchmark/artifacts.py research/ball_wm/scripts/run_pointgoal_benchmark.py research/ball_wm/tests/test_benchmark_policies.py

清理后以上五个文件不应出现在 git status --short。

- [ ] **Step 2: 写连续速度插值失败测试**

在 test_drive_model.py 增加：

    def test_speed_is_zero_at_zero_main_and_continuous_between_calibrated_commands():
        model = DriveModel.from_json(PROFILE_PATH)
        assert model.speed(0.0, 0.0) == 0.0
        midpoint = model.speed(0.75, 0.0)
        assert model.speed(0.5, 0.0) < midpoint < model.speed(1.0, 0.0)
        assert model.speed(-0.75, 0.0) > 0.0

- [ ] **Step 3: 运行测试并确认因接口缺失而失败**

    PYTHONPATH=research/ball_wm /home/an/miniconda3/envs/ball_wm/bin/python -m pytest research/ball_wm/tests/test_drive_model.py::test_speed_is_zero_at_zero_main_and_continuous_between_calibrated_commands -q

预期：FAIL，DriveModel 没有 speed。

- [ ] **Step 4: 实现连续速度插值**

同一方向先按副轴角度插值 speed_m_s，再按主轴绝对值插值；在 0 与 0.5 rad/s 之间加入原点 (0, 0)。返回非负速度大小，方向由调用方保存。

- [ ] **Step 5: 验证并提交**

    PYTHONPATH=research/ball_wm /home/an/miniconda3/envs/ball_wm/bin/python -m pytest research/ball_wm/tests/test_drive_model.py -q
    PYTHONPATH=research/ball_wm /home/an/miniconda3/envs/ball_wm/bin/python -m pytest research/ball_wm/tests -q
    git add research/ball_wm/benchmark/drive_model.py research/ball_wm/tests/test_drive_model.py
    git commit -m "feat: interpolate continuous ball speed"

---

### Task 2: 连续动态状态与运动原语传播

**Files:**
- Create: research/ball_wm/benchmark/dynamic_lattice_planning.py
- Create: research/ball_wm/tests/test_dynamic_lattice_planning.py

**Interfaces:**
- Consumes: DriveModel.speed(...)、DriveModel.curvature(...)。
- Produces: DynamicState、DynamicControlTarget、DynamicPrimitive、propagate_dynamic_primitive(...)。

- [ ] **Step 1: 写速率限制失败测试**

测试从静止和零副轴开始，以 (1.5 rad/s, 30°) 为目标传播 1 秒。主轴只能到 0.5 rad/s，副轴只能到 30°；51 个状态样本均有限且时间间隔为 0.02 秒。

    primitive = propagate_dynamic_primitive(
        start=DynamicState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "stopped", 0.5),
        target=DynamicControlTarget(1.5, math.radians(30.0)),
        drive_model=model,
    )
    assert primitive.end.main_command_rad_s == pytest.approx(0.5)
    assert primitive.end.auxiliary_angle_rad == pytest.approx(math.radians(30.0))
    assert primitive.samples.shape == (51, 8)

- [ ] **Step 2: 运行测试确认类型和函数不存在**

    PYTHONPATH=research/ball_wm /home/an/miniconda3/envs/ball_wm/bin/python -m pytest research/ball_wm/tests/test_dynamic_lattice_planning.py -q

- [ ] **Step 3: 实现核心数据结构**

    @dataclass(frozen=True)
    class DynamicState:
        x_m: float
        y_m: float
        yaw_rad: float
        main_command_rad_s: float
        auxiliary_angle_rad: float
        speed_m_s: float
        elapsed_s: float
        drive_direction: Literal["forward", "reverse", "stopped"]
        stopped_time_s: float

    @dataclass(frozen=True)
    class DynamicControlTarget:
        main_command_rad_s: float
        auxiliary_angle_rad: float

    @dataclass(frozen=True)
    class DynamicPrimitive:
        target: DynamicControlTarget
        samples: np.ndarray
        end: DynamicState
        path_length_m: float

- [ ] **Step 4: 实现 50 Hz 连续传播**

固定 duration_s=1.0、dt_s=0.02、主轴加速度上限 0.5 rad/s²、副轴速率上限 30°/s。每个子步按更新后的连续主轴、副轴、速度和曲率积分 XY/yaw。

- [ ] **Step 5: 写并通过换向测试**

实现 valid_control_targets(state)。正主轴不能直接选择负目标；必须先传播到零速并累计 stopped_time_s >= 0.5。目标顺序固定以保证确定性。

- [ ] **Step 6: 写并通过连续碰撞测试**

实现 dynamic_primitive_is_collision_free(...)。构造起终点均空闲、但中间样本穿过单栅格障碍的原语，断言返回 False。

- [ ] **Step 7: 验证并提交**

    PYTHONPATH=research/ball_wm /home/an/miniconda3/envs/ball_wm/bin/python -m pytest research/ball_wm/tests/test_dynamic_lattice_planning.py -q
    git add research/ball_wm/benchmark/dynamic_lattice_planning.py research/ball_wm/tests/test_dynamic_lattice_planning.py
    git commit -m "feat: propagate rate-limited ball primitives"

---

### Task 3: 状态量化、风险代价与动态搜索

**Files:**
- Modify: research/ball_wm/benchmark/dynamic_lattice_planning.py
- Modify: research/ball_wm/tests/test_dynamic_lattice_planning.py

**Interfaces:**
- Consumes: Task 2 的 DynamicState、DynamicPrimitive 和 valid_control_targets(...)。
- Produces: DynamicPlanRequest、DynamicWaypoint、DynamicPlanResult、plan_dynamic_lattice(...)。

- [ ] **Step 1: 写状态量化失败测试**

断言位置 0.1 m、yaw 5°、主轴 0.25 rad/s、副轴 5°、速度 0.05 m/s 分别进入独立量化箱；不同制动速度不能得到相同键。

- [ ] **Step 2: 实现 dynamic_state_key(...)**

返回：

    (row, column, yaw_bin, main_bin, auxiliary_bin, speed_bin, drive_direction)

连续 DynamicState 始终作为节点载荷，不用量化中心覆盖。

- [ ] **Step 3: 写风险排序失败测试**

构造长度相同的两条无碰撞原语，一条贴近膨胀障碍，另一条保持更大间距。实现距离场风险积分后断言后者代价更低。

- [ ] **Step 4: 写简单绕障搜索失败测试**

    result = plan_dynamic_lattice(request, model)
    assert result.status == "success"
    assert result.waypoints[0].time_from_start_s == 0.0
    assert result.waypoints[-1].speed_m_s < 0.05
    assert result.duration_s <= 32.0
    assert all(b.time_from_start_s > a.time_from_start_s for a, b in pairwise(result.waypoints))

- [ ] **Step 5: 实现请求、结果和 A* 搜索**

DynamicPlanRequest 验证 occupancy、初始状态、目标、node_budget=500_000、planning_timeout_s=60.0。使用现有二维障碍感知 cost-to-go 启发式，并累计设计中冻结的距离、时间、倒车、换向、控制变化和风险代价。

- [ ] **Step 6: 写失败状态与确定性测试**

覆盖 invalid_start、invalid_goal、unreachable、node_budget_exhausted、timeout；相同请求运行两次必须得到相同状态和控制序列。

- [ ] **Step 7: 验证并提交**

    PYTHONPATH=research/ball_wm /home/an/miniconda3/envs/ball_wm/bin/python -m pytest research/ball_wm/tests/test_dynamic_lattice_planning.py -q
    git add research/ball_wm/benchmark/dynamic_lattice_planning.py research/ball_wm/tests/test_dynamic_lattice_planning.py
    git commit -m "feat: search dynamic ball state lattice"

---

### Task 4: 带时间参考的 DynamicOraclePolicy

**Files:**
- Modify: research/ball_wm/benchmark/policies.py
- Modify: research/ball_wm/benchmark/runner.py
- Modify: research/ball_wm/tests/test_benchmark_policies.py
- Modify: research/ball_wm/tests/test_benchmark_runner.py

**Interfaces:**
- Consumes: DynamicPlanResult.waypoints。
- Produces: DynamicOraclePolicy、EpisodeContext.dynamic_reference，以及参考索引、位置/yaw/副轴误差诊断。

- [ ] **Step 1: 写权限与上下文失败测试**

非特权 EpisodeContext 必须拒绝 dynamic_reference。DynamicOraclePolicy 权限包含动态参考，但不包含 occupancy、truth clearance 或 collision。

- [ ] **Step 2: 实现上下文契约与复位**

V1 继续只使用 kinodynamic_reference；V2 只接受 dynamic_reference，两者不能互相替代。

- [ ] **Step 3: 写时间插值和前馈失败测试**

构造 0.0、0.1、0.2 秒参考，断言 act() 按动作块时钟选择正确前馈主轴和副轴，输出为 10 维连续动作块。

- [ ] **Step 4: 实现前馈加反馈跟踪**

将位置误差投影到参考 yaw 的横向方向，yaw 误差归一化到 [-π, π]，副轴反馈限制为 ±5°。任一误差达到第一档（横向 0.10 m、yaw 10°、副轴 10°）时主轴上限为 0.5 rad/s；达到第二档（0.20 m、20°、20°）时上限为 0.25 rad/s；达到第三档（0.30 m、30°、30°）时输出零主轴。最终动作继续经过 runtime ActionSmoother。

- [ ] **Step 5: 写换向和终点停稳测试**

相反主轴命令前必须等待实际速度小于 0.05 m/s 并持续五个 10 Hz 周期；参考终点要求主轴为零并持续输出停止。

- [ ] **Step 6: 扩展 runner 诊断并验证**

状态数组保持 N+1，策略诊断保持 N。

    PYTHONPATH=research/ball_wm /home/an/miniconda3/envs/ball_wm/bin/python -m pytest research/ball_wm/tests/test_benchmark_policies.py research/ball_wm/tests/test_benchmark_runner.py -q

- [ ] **Step 7: 提交**

    git add research/ball_wm/benchmark/policies.py research/ball_wm/benchmark/runner.py research/ball_wm/tests/test_benchmark_policies.py research/ball_wm/tests/test_benchmark_runner.py
    git commit -m "feat: track timed dynamic oracle references"

---

### Task 5: Isaac runtime 动态计划接入

**Files:**
- Modify: research/ball_wm/benchmark/isaac_runtime.py
- Modify: research/ball_wm/tests/test_isaac_runtime_contract.py

**Interfaces:**
- Consumes: reset observation 的本体状态、runtime occupancy 和 plan_dynamic_lattice(...)。
- Produces: IsaacBenchmarkRuntime.dynamic_plan(episode) -> DynamicPlanResult。

- [ ] **Step 1: 写初始状态来源失败测试**

用非零主轴状态、副轴角度和线速度构造 reset observation，断言 DynamicPlanRequest 来自 observation，而不是硬编码为零。

- [ ] **Step 2: 写缓存和 episode 隔离失败测试**

同一 episode 连续调用两次只规划一次；另一个 episode 必须触发 episode 不匹配错误。

- [ ] **Step 3: 实现请求构造和缓存**

reset 后保存初始 observation；dynamic_plan() 使用冻结 occupancy、起终点和 reset 状态。规划失败不能回退 V1、GoalGreedy 或直线动作。

- [ ] **Step 4: 验证并提交**

    PYTHONPATH=research/ball_wm /home/an/miniconda3/envs/ball_wm/bin/python -m pytest research/ball_wm/tests/test_isaac_runtime_contract.py -q
    git add research/ball_wm/benchmark/isaac_runtime.py research/ball_wm/tests/test_isaac_runtime_contract.py
    git commit -m "feat: provide cached dynamic plans in Isaac runtime"

---

### Task 6: CLI、审计产物和协议隔离

**Files:**
- Modify: research/ball_wm/scripts/run_pointgoal_benchmark.py
- Modify: research/ball_wm/benchmark/artifacts.py
- Modify: research/ball_wm/tests/test_benchmark_artifacts.py
- Modify: research/ball_wm/tests/test_benchmark_runner.py

**Interfaces:**
- Consumes: DynamicOraclePolicy、DynamicPlanResult 和 runner 诊断。
- Produces: --policy oracle_v2、动态参考 NPZ 数组和 V2 认证指标。

- [ ] **Step 1: 写 CLI 隔离失败测试**

_make_policy("oracle", ...) 仍返回 V1；_make_policy("oracle_v2", ...) 返回 DynamicOraclePolicy；未知名称仍报错。

- [ ] **Step 2: 写 V2 审计数组失败测试**

states.npz 必须包含：

    dynamic_reference_time_s
    dynamic_reference_pose
    dynamic_reference_speed_m_s
    dynamic_reference_command
    dynamic_reference_drive_direction
    dynamic_reference_index
    dynamic_position_error_m
    dynamic_yaw_error_rad
    dynamic_auxiliary_error_rad

验证参考长度一致、时间严格递增、方向仅为 -1/0/1，执行诊断长度等于动作数。

- [ ] **Step 3: 写认证指标失败测试**

certification 增加 planner_version、planning_wall_time_s、predicted_duration_s、maximum_main_rate_rad_s2、maximum_auxiliary_rate_rad_s。缺字段或参考超过冻结速率不得发布。

- [ ] **Step 4: 实现 CLI、写盘和发布校验**

environment 明确记录 policy=oracle_v2 和模型参数；V1 产物不被要求包含 V2 数组。

- [ ] **Step 5: 验证并提交**

    PYTHONPATH=research/ball_wm /home/an/miniconda3/envs/ball_wm/bin/python -m pytest research/ball_wm/tests/test_benchmark_artifacts.py research/ball_wm/tests/test_benchmark_runner.py -q
    git add research/ball_wm/scripts/run_pointgoal_benchmark.py research/ball_wm/benchmark/artifacts.py research/ball_wm/tests/test_benchmark_artifacts.py research/ball_wm/tests/test_benchmark_runner.py
    git commit -m "feat: audit dynamic oracle v2 episodes"

---

### Task 7: 全量纯代码验证与规划性能

**Files:**
- Create: docs/superpowers/specs/2026-09-13-dynamic-oracle-v2-unit-verification.md

**Interfaces:**
- Consumes: 完整 V2 单元、契约和产物链路。
- Produces: 可复现的纯代码验证记录。

- [ ] **Step 1: 运行全量测试**

    git diff --check
    PYTHONPATH=research/ball_wm /home/an/miniconda3/envs/ball_wm/bin/python -m pytest research/ball_wm/tests -q

预期：全部通过，只有项目已有的第三方弃用警告。

- [ ] **Step 2: 测量三张合成地图的搜索性能**

运行空旷、单障碍绕行和必须倒车的固定请求。每张重复两次，计划数组必须完全一致；每个合成请求必须在 5 秒内完成。

- [ ] **Step 3: 写实际验证数字并提交**

    git add docs/superpowers/specs/2026-09-13-dynamic-oracle-v2-unit-verification.md
    git commit -m "test: verify dynamic oracle v2 core"

---

### Task 8: hard_0_e1_0002 第一仿真门槛

**Files:**
- Generated, ignored: research/ball_wm/runs/e1_0002_dynamic_oracle_v2_gate/
- Create: docs/superpowers/specs/2026-09-13-dynamic-oracle-v2-e1-gate.md

**Interfaces:**
- Consumes: 冻结 hard_0_dev_manifest.json 和 oracle_v2。
- Produces: 第一门槛证据和继续/停止结论。

- [ ] **Step 1: 运行单任务**

    /home/an/miniconda3/envs/ball_wm/bin/python research/ball_wm/scripts/run_pointgoal_benchmark.py \
      --manifest research/ball_wm/configs/benchmark_v1/hard_0_dev_manifest.json \
      --split dev --policy oracle_v2 \
      --scene-root /home/an/project/NavDP/assets/scenes \
      --data-root research/ball_wm/data \
      --run-root research/ball_wm/runs/e1_0002_dynamic_oracle_v2_gate \
      --headless --record-video --episode-id hard_0_e1_0002

- [ ] **Step 2: 校验产物门槛**

必须同时满足：成功、无碰撞、35 秒内完成、最大横向误差小于 0.2 m、参考主轴速率不超过 0.5 rad/s²、参考副轴速率不超过 30°/s。

- [ ] **Step 3: 根据门槛分支**

若失败：记录首次偏差时刻及参考/实际主轴、副轴、速度、yaw、clearance，停止 Task 9，不改变冻结 Benchmark。若通过：提交证据并继续 Task 9。

- [ ] **Step 4: 提交门槛报告**

    git add docs/superpowers/specs/2026-09-13-dynamic-oracle-v2-e1-gate.md
    git commit -m "test: evaluate dynamic oracle v2 E1 gate"

---

### Task 9: 七任务卡冻结评测

**Files:**
- Generated, ignored: research/ball_wm/runs/benchmark_hard_0_dynamic_oracle_v2_35s/
- Create: docs/superpowers/specs/2026-09-13-dynamic-oracle-v2-benchmark-evaluation.md

**Interfaces:**
- Consumes: 已通过 Task 8 的实现和冻结七任务 manifest。
- Produces: V1/V2 同协议成功率、碰撞率、超时率和 SPL 对比。

- [ ] **Step 1: 运行七任务**

    /home/an/miniconda3/envs/ball_wm/bin/python research/ball_wm/scripts/run_pointgoal_benchmark.py \
      --manifest research/ball_wm/configs/benchmark_v1/hard_0_dev_manifest.json \
      --split dev --policy oracle_v2 \
      --scene-root /home/an/project/NavDP/assets/scenes \
      --data-root research/ball_wm/data \
      --run-root research/ball_wm/runs/benchmark_hard_0_dynamic_oracle_v2_35s \
      --headless --record-video

- [ ] **Step 2: 校验七个审计包**

每项必须包含 manifest、environment、两段视频、任务卡、轨迹图、actions、states、metrics 和 human review；summary.json 必须使用 ball_pointgoal_v1_1_35s。

- [ ] **Step 3: 计算并解释结果**

通过标准：至少 6/7 成功、最多 1/7 碰撞，且 V1 已成功的四项不得退化。报告逐项对比 V1 的 4/7 成功、3/7 碰撞与 V2 结果，不把规划失败计作成功。

- [ ] **Step 4: 提交评测报告并最终验证**

    git add docs/superpowers/specs/2026-09-13-dynamic-oracle-v2-benchmark-evaluation.md
    git commit -m "docs: report dynamic oracle v2 benchmark"
    git diff --check
    PYTHONPATH=research/ball_wm /home/an/miniconda3/envs/ball_wm/bin/python -m pytest research/ball_wm/tests -q
    git status --short

预期：测试全部通过；只允许存在被 .gitignore 忽略的仿真运行产物。
