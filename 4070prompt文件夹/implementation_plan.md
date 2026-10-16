# 球形机器人导航 Baseline 与采集 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task when that skill is available. Steps use checkbox (`- [ ]`) syntax for tracking. 不具备该技能的服务器 AI 按本文件的任务顺序与证据要求直接执行，无需安装技能或创建其他 AI 任务。

**Goal:** 复用现有球形机器人执行链，交付可审计的已知地图数据采集和双路单目深度视觉导航。

**Architecture:** 已知地图教师与视觉策略共用动作执行和记录接口；视觉策略只消费当前允许观测，左右分别估深后融合三态地图。先完成小批采集，再依次验证可见真值深度、DA3 深度和估计定位。

**Tech Stack:** 服务器现有 Python、NumPy、现有规划/控制模块、PyTorch/DA3、Isaac Sim；测试使用项目现有框架，默认 pytest。不强制升级 CUDA、Torch、Isaac 或引入 ROS/Nav2。

**Spec:** 同目录 `design.md`。编码 AI 必须先读设计，再按本计划实施。

## Global Constraints

- 左右图分别估计米制深度，再融合点云或地图。
- 优先复用已有 Oracle 的规划、跟踪、倾侧控制和动作执行链。
- 地图三态编码 `free=0, occupied=1, unknown=2`；适配旧格式而非混用编码。
- 单位 m/s/rad，base 为 x前/y左/z上，光学系为 x右/y下/z前；矩阵为 `T_target_from_source`。
- 保持核验后的 50/50/50/10 Hz 物理/控制/状态/相机合同，深度更新率另测。
- 原始 RGB、真值标签、预测深度、所有动作层与数据划分均按设计记录。
- 不静默修改主表的 35 s、0.20 m、物理接触规则；服务器既有不同协议先报告映射。
- 不把测试 skip、假模型和离线回放当作 GPU/Isaac 闭环通过。
- 不更改已有训练数据、权重和历史结果；不覆盖用户未提交改动。
- 不在本轮训练世界模型、引入行人模块、整体更换低层控制或启动未审计的大规模采集。

## Review Focus

1. 同 shape 的过期/跨 episode 深度被误配到新图像；应拒绝并保留原因。测试归 Task 4/7。
2. 两路相机矛盾、深度断边和自体遮挡制造虚假自由空间；应保留障碍/未知。测试归 Task 5。
3. 中心路径可行但足迹跨越未知/障碍，或足迹被重复膨胀导致启动死锁；测试归 Task 5/6。
4. 录像、额外 render 和推理改变物理步或动作时间归属；测试归 Task 2/7/8。
5. 中断、resume 和缓存混入另一配置或 split，产出看似完整的污染数据；测试归 Task 2/3/8。

## 阅读约定

下列路径均相对服务器实际 `research/ball_wm/` 包。名字属于拟新增默认接口，不能据此声称文件已经存在。Task 0 输出路径映射后，如已有等价模块则复用并记录最终名称；所有后续命令必须替换成真实路径并实际执行。每个功能任务先补能失败的关键测试，再做最小实现并跑对应回归；测试已有覆盖时直接复用，不制造只测实现自身的重复测试。

## Task 0：核对服务器仓库和已跑通执行链

**Files:** 新建 `reports/navigation_baseline/environment_inventory.md`；读取仓库规则、现有配置、入口和测试。此阶段不改生产代码。

**Interfaces:** 产出 `repo_root, package_root, python/launcher, base_commit, dirty_changes, oracle_entry, recorder_entry, contact_entry, model_checkpoint, path_mapping, known_failures`。

- [ ] 读取 AGENTS/README，检查 Git 状态，记录 Python、Isaac、Torch/CUDA、GPU、模型和资产路径；不得输出凭据。
- [ ] 定位 Oracle、相机、位姿、本体、动作协议、PID/smoother、物理接触、writer、audit 的实际入口，列出哪些复用、哪些缺失。
- [ ] 查阅现有最新 Oracle 成功记录，选定可复现版本；没有日志时标注未知，不能将任意名为 Oracle 的类当作已验证版本。
- [ ] 在改代码前跑相关轻量回归，保存命令/退出码/现存失败。真实 Isaac 已有最短 smoke 可运行时执行一次。
- [ ] 生成路径映射和阶段缺依赖表。缺 GPU/权重只阻塞对应真实测试，不阻塞 CPU 合同、记录器或代码审查。

**通过条件:** 报告可区分实际发现、历史文档陈述、待实现接口和未运行项目。

## Task 1：观测、权限、坐标与配置合同

**Files:** Create `navigation_baseline/contracts.py`, `configs/navigation_baseline_v1.yaml`, `tests/navigation_baseline/test_contracts.py`。必要时为旧 runtime 增加很薄的观测适配器。

**Interfaces:** 实现设计中的 `CameraFrame, DepthFrame, PoseSample, OnlineObservation, OfflineTruth, PolicyDecision`；配置加载返回严格校验的 `BaselineConfig`；提供 `make_online_observation(runtime_sample, mode)`，不能把 runtime 对象本身传给策略。

- [ ] 写失败测试：合法单位和 shape 通过；非法 K、非刚体 T、未知 mode、零四元数、过期 frame ID 和缺 source 拒绝。
- [ ] 写来源测试：`dual_mono_est_pose` 无法收到 `source=ground_truth_assisted` 的导航位姿；真值倾侧等低层例外单独记录，不假装估计来源。
- [ ] 写权限测试：用 guard/spy 的 truth provider 包装 full map/path/future access，视觉策略一旦调用即失败。离线记录器仍能读取 truth。
- [ ] 写数组别名测试：策略改输入数组不会修改 runtime/truth；不通过对象共享泄漏真值。
- [ ] 运行 `python -m pytest tests/navigation_baseline/test_contracts.py -q`，确认新行为尚不存在时失败；实现合同和配置后重跑通过。
- [ ] 在不安装 Isaac、无 CUDA 的测试环境导入 contracts，确认没有副作用。

**通过条件:** 三个基础 mode 的输入权限可从配置和类型层同时识别，非法混用失败而非静默纠正。

## Task 2：统一记录器与严格数据审计

**Files:** Create `navigation_baseline/recording.py`, `scripts/audit_navigation_dataset.py`, `tests/navigation_baseline/test_recording.py`, `tests/navigation_baseline/test_dataset_audit.py`；复用/适配 `scripts/collection_schema.py` 和现有 audit，不无理由替换 writer。

**Interfaces:** `EpisodeRecorder.append_transition(step)`, `finalize(summary)->Path`；`audit_dataset(root:Path, strict:bool)->AuditReport`，报告含逐错误代码/文件/原因。

- [ ] 写 3 个观测时刻、10 个 50 Hz 动作的合成 fixture，逐项断言每个 0.1 s 区间恰好关联其后的 5 个动作，终端部分区间有 mask。
- [ ] 写四层不同动作值，验证 raw、protocol、PID/平滑层、applied 不被复用同一数组覆盖，实测关节反馈独立。
- [ ] 写失败测试：时间逆序、重复 frame ID、缺一侧 RGB、坏图、缺引用深度、越界动作、丢 control step、非法 split 失败。
- [ ] 写 crash/resume 测试：写到一半异常，没有 COMPLETE 和正式 manifest 记录；同一完整 episode 重试不重复；配置 hash 改变拒绝复用。
- [ ] 写 manifest 一致性测试：文件存在但内容 hash 被篡改失败；失败/超时的完整 episode 可通过，进程中断不能伪造正常终态。
- [ ] 运行对应两份测试，确认失败后实现 writer bridge 与审计。`--strict` 有任何硬错误必须非零退出，不能只打印 warning。
- [ ] 写明可选资产字段：未启用预测深度时不要求预测文件；启用时逐帧必须完整。不能用“可选”掩盖已经声明采集但缺失的数据。

**通过条件:** 合成正常数据通过 strict audit；每种故意破坏的数据均被准确拒绝，错误信息足以定位。

## Task 3：已知地图导航采集器，形成第一份可用交付

**Files:** Create `scripts/collect_navigation_dataset.py`, `tests/navigation_baseline/test_collection_driver.py`；Modify 现有 runner 的记录订阅接口，避免改变控制语义。

**Interfaces:** 读取冻结 collection manifest；以 `known_map_gt_pose` 调用现有教师；输出 Task 2 的正式 episode；支持以完整 episode 为粒度的 `--resume`。

- [ ] 写 fake runtime 的流程测试：reset、warmup、正式计时、动作执行、终止、finalize 顺序正确；warmup 步不混入 t=0 初态。
- [ ] 写 schema、mode、scene/layout、seed、反馈来源和 commit/hash 进入 metadata 的断言。
- [ ] 写不覆盖、不跨 split 及中断后重采完整 episode 的失败测试；实现采集驱动。
- [ ] 用真实 Isaac 跑一条 5–10 s 短任务，检查图像非重复、时间推进、动作真实施加与记录无额外 physics step。
- [ ] 跑冻结 E0 一条完整教师任务；保留结果，即使未到达也不得删掉。定位新代码引入的回归后再推进。
- [ ] 初次交付 3 条完整小样本并 strict audit、回放；之后试采总计 20–30 条，先完成布局 split 清单，不随机切帧。

**通过条件:** 记录是真实物理 rollout；至少 E0 教师可无碰撞到达；小批数据审计通过。若教师失败，报告其现有缺陷和改动前/后差异，不转去调视觉掩盖问题。

## Task 4：深度 adapter 与几何投影

**Files:** Create `navigation_baseline/depth.py`, `geometry.py`, `tests/navigation_baseline/test_depth.py`, `test_geometry.py`；复用现有 DA3 代码但消除写死的模型/工程绝对路径。

**Interfaces:** `DepthEstimator.predict(CameraFrame)->DepthFrame`；DA3 与可见 GT 返回相同 z 深度语义；`project_depth(...)->SurfaceEvidence`。

- [ ] 写针孔数值测试：`fx=fy=100,cx=cy=1`，像素 `(u=2,v=1)`、z=2 m，投影为 `(0.02,0,2)`；加入已知刚体变换后坐标与手算一致。
- [ ] 写光线距离测试：同一像素提供 range，按光线范数转换回 z 后得到相同点；不允许直接把 range 当 z。
- [ ] 写仿射像素变换测试：缩放/裁剪/90°旋转后回投同一三维点一致，mask 与深度重映射一致。
- [ ] 写 DA3 单位测试：用假网络输出验证低层适配只应用一次焦距尺度；若使用高级 API，fake 输出已为米时不得二次缩放。
- [ ] 写 NaN/Inf/零/负/过距/全 mask 输入测试，以及相同尺寸但 frame ID、camera ID 或 episode 不符时拒绝。过期结果不可进图。
- [ ] 写姿态补偿测试：同一世界平面在不同 roll/pitch 下回投一致；用错当前姿态的实现应能被测试抓到。
- [ ] 跑 CPU 测试，通过后真实加载 checkpoint；先一对图确认输出，再测至少 30 对稳态输入的耗时和峰值显存。记录 warmup 与稳态分开。
- [ ] 用 100–300 对开发帧生成分区误差、覆盖/漏检和图例报告，不逐帧做 GT scale fit。真实模型不可用时此子项写 BLOCKED，不能用 fake 顶替。

**通过条件:** 几何和来源合同测试通过；真实模型输入输出有证据；深度效果是否足够导航另由误差预算决定。

## Task 5：连续自由证据与三态地图

**Files:** Create `navigation_baseline/mapping.py`, `tests/navigation_baseline/test_mapping.py`；Task 4 的 geometry 增加经过验证的地面小片/可见性支持输出。

**Interfaces:** `ObservedMap.update(SurfaceEvidence)->MapUpdateStats`、`snapshot(t_sim_s)->MapSnapshot`；提供单元标签、来源、last_seen 和连通性诊断。

- [ ] 写纯未知初图测试：空点、无效深度、自体 mask 均不创建自由格；过期障碍回未知而非自由。
- [ ] 写稠密平地测试：只在真实有效像素支持范围形成连续自由区，与核验的 body_support 正常连接。
- [ ] 写孔洞、深度断边、地面后的矮箱、高射线越过矮箱测试：不能跨洞补地面，不能把高射线投成整列自由。
- [ ] 写左右同格冲突与旧自由/新障碍测试：可信占据优先；无效/低质量数据不能清除强占据。
- [ ] 写 grid 索引测试：负坐标、边界、不同 origin 和 cell center；与旧数组行列顺序显式转换，越界不绕回。
- [ ] 写两种通道宽度测试，确认足迹只膨胀一次；小于足迹的拒绝，明确宽于足迹加 margin 的通过。
- [ ] 跑 CPU 测试后实现；导出合成地面、单障碍、遮挡三种图，人工对照原始支持证据检查。
- [ ] 使用 Task 4 的可见 GT 深度回放测试地图覆盖、起点自由连接与障碍高度，不先用 DA3 调参掩盖几何错误。

**通过条件:** 同时满足“不编造自由空间”和“在观测充分的空地可启动”；不能靠把起点格强制放行通过。

## Task 6：可达子目标、运动约束规划与参考保留

**Files:** Create `navigation_baseline/frontier.py`, `planning.py`, `tests/navigation_baseline/test_frontier.py`, `test_planning.py`；适配已有 Hybrid A*/lattice 与跟踪参考。

**Interfaces:** `FrontierSelector.select(...) -> SubgoalDecision`；`LocalPlanner.plan(...) -> PlanResult`；足迹检查返回明确失败原因；reference 标记 `global_goal` 或 `frontier`。

- [ ] 写目标不可见但存在可达 frontier 的地图测试：选择自由区内且足迹完整可走的目标，不读取隐藏地图。
- [ ] 写 U 型/长墙合成图测试：允许暂时远离目标，访问惩罚和方向保持防止相邻调用来回切边；只要求本图内可解释选择，不宣称解决所有探索问题。
- [ ] 写无可达边界测试：返回原因并制动；禁止盲走未知、原地转向或伪造短路线。
- [ ] 写足迹扫掠、转弯半径、对角切角、地图边界测试。路径中心点过得去而实际足迹不行时必须拒绝。
- [ ] 写参考保留测试：不相关地图更新保留；新障碍进入参考扫掠区时失效并重建；临时端点不会报告全局成功。
- [ ] 运行测试并实现适配，复用旧运动原语与跟踪器；确认初始化本任务不读取真值 obstacle map。

**通过条件:** 子目标、路径和参考消费使用同一个地图合同；失败具有明确分类，控制语义未被重写。

## Task 7：完整 policy 和三个基础模式闭环

**Files:** Create `navigation_baseline/policy.py`, `scripts/run_navigation_baseline.py`, `tests/navigation_baseline/test_policy.py`, `test_runtime_adapter.py`；最小修改 runner/runtime 注册接口。

**Interfaces:** `BaselinePolicy.act(OnlineObservation)->PolicyDecision`，结果送现有动作协议、PID/smoother/supervisor；记录器旁路订阅实际执行。模式选择必须确定唯一 source provider。

- [ ] 写旧接口兼容测试：同一 raw request 通过旧链与新适配后的 protocol/applied 一致；新增 recorder 不改变控制请求与 physics step。
- [ ] 写无深度、worker 失败、姿态无效、过期结果测试：进入受控制动且继续记录惯性和接触；零请求不能变 stopped。
- [ ] 写 episode reset 清空地图/缓存/访问记忆，拒绝上一 episode 深度的测试；禁止两条轨迹共享地图对象。
- [ ] 写模式权限集成测试：故意把 full-map/oracle-path provider 传给视觉模式时拒绝；同一 online 输入替换隐藏地图不会改变视觉策略输出。
- [ ] 实现 lockstep，记录 wall/sim 时间和 render/physics 步。CLI `--help` 不加载 Isaac/模型，缺依赖时报具体错误。
- [ ] 运行可见真值深度 E0；若起点无法启动，输出已知自由比例、足迹净空、支持证据图和 reason，回 Task 5 修正。
- [ ] 接 DA3 模式，使用相同任务、执行器和地图参数做闭环；无法达到效果目标时保留失败而非换成功定义。
- [ ] 可选 latency_aware：队列=1，图像副本传 worker，Isaac 只在主线程；测试超时、乱序、未来时间和跨 episode 结果拒绝，以及速度/制动预算消费。

**通过条件:** 三个基础模式能按声明权限完整运行并产出可审计记录；只有真实无碰撞到达才能报告对应任务成功。

## Task 8：统一评测、回归与数据交付

**Files:** Create `navigation_baseline/evaluation.py`, `scripts/evaluate_navigation_depth.py`, `tests/navigation_baseline/test_evaluation.py`, 实际运行说明；输出 `reports/navigation_baseline/verification_*`。

**Interfaces:** `evaluate_episode(records, task)->EpisodeMetrics`；表格与 JSON 来自同一逐 episode 数据；深度、导航、数据质量的结论分别输出。

- [ ] 写到达/碰撞/超时/异常优先级测试，兼容原规则；同一物理步同时接触与进入目标区不能统计无碰撞成功。
- [ ] 写 E2 通道门线顺序测试：参考经过但实际绕过时，`corridor_passed=false`；任务到达与通道资格独立。
- [ ] 跑全部新增 CPU 测试和相关旧回归，比较 Task 0；现存失败有证据记录，不通过删除断言消除。
- [ ] 跑 E0/E1/E2 × 三个基础模式，共 9 个完整任务，以及一条有意物理接触和一条不可达诊断；CPU fake 不计入真实矩阵。
- [ ] 抽取至少 3 个不同任务的 RGB/深度/mask/地图/轨迹叠图和原始动作层，人工核对首帧、转弯、失败或终止帧。
- [ ] 试采 20–30 条 episode，strict audit，抽查 3 条回放；真实执行一次中断和 resume 检查，不仅跑 fake writer。
- [ ] 生成机器可读检查清单，每项写 PASS/FAIL/BLOCKED/NOT_RUN、命令、退出码、日志与证据文件；生成面向用户的结论和已知限制。

**通过条件:** 不存在未披露的新正确性回归；已知地图采集满足设计的最小可用条件。视觉性能未达标时单独写未就绪，不把完整程序运行当作成功导航。

## Task 9：估计定位模式，后续阶段

**Files:** 适配服务器已有里程计/IMU 模块；新增 `tests/navigation_baseline/test_estimated_pose.py`，扩展配置与来源报告。

**Interfaces:** 提供 `source=estimated` 的 `PoseSample`；初始规范对齐与低层真值/理想传感器例外均记录。

- [ ] 先测试真值替换注入：修改真值位姿而不改变允许 IMU/编码器流时，估计定位输出不能跟随隐藏真值变化。
- [ ] 测 IMU/编码器缺失、时间错位和漂移时的状态/回退，再接入；不能在异常时静默切回真值定位。
- [ ] 追加相同 E0/E1/E2 的真实任务和误差轨迹，单独出表，与 GT pose 版本分开报告。

**通过条件:** 估计定位来源可核验且没有隐藏回退。缺可靠估计器可标 BLOCKED，前面阶段仍独立交付。

## Task 10：同状态动作分支，后续阶段

**Files:** 核对/适配 `scripts/collect_candidate_rollouts.py`，新增完整状态/前缀重放验证与分支 audit。

**Interfaces:** context→多个动作序列→局部 rollout；输出 context/group/split、状态恢复摘要和重复误差。

- [ ] 写完整状态恢复测试，覆盖 PID 积分、平滑器和关节/速度，不只对 root pose 断言。
- [ ] 在真实物理中对同一候选重复执行，先冻结状态/轨迹容差再运行正式分支；不能从本次误差倒推出方便通过的容差。
- [ ] 先用少量 context、8–16 候选和 0.5 s horizon 验证，确保同 context 全部分支不跨 split。
- [ ] 无法满足上下文一致性时输出 BLOCKED 和差异证据，不发布为正式反事实动作排序数据。

**通过条件:** 每个正式 context 有可复核恢复/重放记录和重复 rollout 误差；未通过的不进入正式训练索引。

## 最终交付清单

- [ ] 修改摘要、最终实际文件映射、分阶段状态。
- [ ] 可运行代码、冻结配置、测试与真实 launcher 命令。
- [ ] 代码/资产/权重/数据版本、分割清单和小样本 manifest。
- [ ] 新增与旧回归的实测日志，真实模型和 Isaac 运行证据。
- [ ] `implementation_report.md`, `verification_report.md`, `verification_results.json`, `failed_cases.csv`。
- [ ] 无法执行和效果未达标的部分明确列出；不把本计划复述为实施已完成。
