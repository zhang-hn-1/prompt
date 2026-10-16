# 球形机器人视觉导航与已知地图数据采集设计

日期：2026-10-16。状态：交给服务器 AI 的实施设计，尚未实现和实测。

## 1. 目标、范围和证据边界

要完成两个可独立使用、共享执行与记录接口的成果：

1. 双路单目米制深度、显式空间记忆、经典规划和现有低层控制构成的视觉 baseline。
2. 已知障碍地图导航驱动的仿真轨迹采集器，给后续视觉表示、运动预测和世界模型训练提供原始数据与离线标签。

已确认“拼接”指分别估深后的点云或地图融合，不是把左右 RGB 拼成一张再交给单目模型。保留现有相机装配，不因更换算法就擅自改变相机位置和视野。两路单目降低了对跨目匹配的依赖，但不能消除自体遮挡和未观测区域。

本地可核对的历史入口：

- 代码归档：`Spherical-robot-navigation-codex-navigation-core-update-20261008.zip`。
- 核心目录：`research/ball_wm/benchmark`、`scripts`、`training`、`configs`、`tests`。
- 历史深度模块：`research_additions/experiments/visual_information_new_rig_20261004_v2/model_inference.py`，含 `da3_metric` 和 `unidepth_v2`，左右图分别预测；部分文件仅存在压缩包或阅读版中。
- 本地快照的 `PolicyObservation` 同时包含图像、本体历史、动作历史和位置/航向字段。字段存在并不代表其来源满足视觉实验权限，必须重审。
- 汇报材料记录了 Oracle 和视觉实验的不同条件，不能把旧成功率当成本次回归一定通过的保证。

服务器先读取仓库规则、当前提交、未提交改动、实际入口和已运行记录，形成路径映射。旧归档和文档中的命令、绝对路径、指标都只作线索。本次不要求重训深度网络、世界模型或 PID，不开发动态行人导航，不从头替换仿真框架。

## 2. 路线选择与实施顺序

备选路线有：继续 SGBM；双路米制单目深度后融合；联合多视图模型重建。选择第二条作为本轮主线，原因是已有代码接口可复用，且符合用户选择。保留 SGBM 历史入口用于必要回归，不同时重做多种模型。

先实现已知地图采集，再实现可见深度建图，随后接 DA3，最后接估计定位。动作分支采集作为增强项单独验收。无需等待视觉导航完全成熟才开始小批采集。

| 模式 ID | 地图/深度来源 | 导航位姿 | 定位与权限用途 |
|---|---|---|---|
| `known_map_gt_pose` | 场景完整障碍图 | 仿真真值 | 教师、执行诊断、采集 |
| `visible_gt_depth_gt_pose` | 当前实际相机的渲染深度 | 仿真真值 | 可见性、建图、探索诊断 |
| `dual_mono_gt_pose` | 左右当前 RGB 的 DA3 米制深度 | 仿真真值 | 第一版视觉 baseline，明确真值定位辅助 |
| `dual_mono_est_pose` | 同上 | 估计定位 | 后续完整视觉版本，仍要声明低层反馈来源 |

后三种模式不得读完整障碍地图、Oracle 路径、未来图像、未来状态或未观测障碍。`visible_gt_depth_gt_pose` 的深度权限仅限当前相机实际可见表面，禁止用整场景网格替代可见深度。

同一配置下固定跟踪器、动作协议、低层控制、接触评分、初态和任务预算。若现有 Oracle 的规划器硬依赖全图，先抽出执行与参考消费部分；不要将 Oracle 全局路线暗中传给视觉规划器。无法完全共享时，报告具体差异，不能声称是只替换感知的严格对照。

## 3. 系统结构与权限隔离

```text
主仿真线程：采集 RGB/标定/允许状态 -> 观测包
                                 |
                     深度适配器（单目或可见 GT）
                                 |
                  三维投影、自体排除、地面/障碍证据
                                 |
                     地图融合、访问历史、边界子目标
                                 |
                    受运动约束的局部规划、路径保留
                                 |
                     共享跟踪器、PID、限幅/平滑器
                                 |
                          物理执行与接触检查

完整地图 -> 教师规划（仅 known_map 模式）------^

真实状态/深度/地图 -> 离线标签与评估记录器
```

实现两个隔离的数据通道：`OnlineObservation` 和 `OfflineTruth`。视觉策略只接收前者。训练和评估记录器可订阅后者，但不能通过引用、闭包、全局对象、文件路径或 runtime 方法回流到视觉策略。

`OnlineObservation.pose_source` 必须为 `ground_truth_assisted` 或 `estimated`；`low_level_feedback_source` 另行记录。估计位姿模式如果继续用仿真真值倾侧反馈或理想 IMU，必须在报告和 metadata 中保留该条件，不称为完全真实传感器条件。

## 4. 拟定模块和接口

优先遵循服务器的包组织。以下为没有等价模块时的默认新增布局，均相对 `research/ball_wm/`：

```text
navigation_baseline/
  contracts.py             # 数据类型、来源、时间和标定验证
  depth.py                 # DA3 / visible GT adapter，延迟测量
  geometry.py              # 深度语义、投影、姿态、地面和遮挡
  mapping.py               # 三态地图、来源、可见自由空间
  frontier.py              # 可达边界子目标、访问记忆、防振荡
  planning.py              # 现有规划适配、足迹、路径有效性
  policy.py                # 主流程、回退、诊断；复用低层执行
  recording.py             # 统一采集接口，桥接已有 writer
  evaluation.py            # 评分整合和逐任务输出
scripts/
  run_navigation_baseline.py
  collect_navigation_dataset.py
  evaluate_navigation_depth.py
  audit_navigation_dataset.py
configs/navigation_baseline_v1.yaml
tests/navigation_baseline/
```

不要把所有新逻辑塞进已有超大 `policies.py` 或 `isaac_runtime.py`。已有同等组件直接适配，提交 `environment_inventory.md` 说明映射关系。CPU 模块导入不得触发 Isaac 初始化、CUDA 分配或模型下载。

下列是必须保留的接口语义，不要求机械照抄类型名称：

```python
DepthEstimator.predict(frame: CameraFrame) -> DepthFrame
project_depth(frame: CameraFrame, depth: DepthFrame,
              pose: PoseSample, cfg: GeometryConfig) -> SurfaceEvidence
ObservedMap.update(evidence: SurfaceEvidence) -> MapUpdateStats
ObservedMap.snapshot(t_sim_s: float) -> MapSnapshot
FrontierSelector.select(grid: MapSnapshot, pose: PoseSample,
                        goal_xy: ndarray) -> SubgoalDecision
LocalPlanner.plan(grid: MapSnapshot, pose: PoseSample,
                  subgoal: SubgoalDecision) -> PlanResult
BaselinePolicy.act(obs: OnlineObservation) -> PolicyDecision
EpisodeRecorder.append_transition(step: TransitionRecord) -> None
EpisodeRecorder.finalize(summary: EpisodeSummary) -> Path
```

`DepthEstimator` 只读单目图像和标定。不得接收任务 ID、地图文件或位姿真值作为深度模型输入。允许后处理使用模式允许的姿态来投影；这与网络输入分开记录。

主要类型约定：

| 类型 | 必需字段 |
|---|---|
| `CameraFrame` | `camera_id, frame_id, capture_t_sim_s, rgb[H,W,3] uint8, K[3,3], T_base_from_optical[4,4], mask_valid[H,W], rig_id` |
| `DepthFrame` | `camera_id, frame_id, capture_t_sim_s, depth_z_m[H,W] float32, valid[H,W], quality_score optional, quality_kind, model_id, checkpoint_sha256, inference_wall_s` |
| `PoseSample` | `t_sim_s, T_world_from_base[4,4], velocity_world_m_s[3], source, attitude_source` |
| `SurfaceEvidence` | `capture_t_sim_s, camera_id, occupied_points_world[N,3], supported_floor_patches, visibility_support, provenance` |
| `MapSnapshot` | `t_sim_s, frame_id, origin_xy_m, resolution_m, labels[Ny,Nx], last_seen_s[Ny,Nx], evidence_source, occupied_probability optional` |
| `SubgoalDecision` | `status, target_xy, kind, selected_frontier_id optional, reason` |
| `PlanResult` | `status, reference, collision_check_summary, goal_kind, reason` |
| `PolicyDecision` | `raw_request, protocol_request, reference_id, status, diagnostics` |

`status/reason` 为稳定枚举；至少涵盖 `ok, no_observation, stale_observation, no_supported_free_route, no_reachable_frontier, invalid_pose, invalid_calibration, planner_failed, goal_reached`。停止请求不等于物理停稳，二者分别记录。数组形状为约定的一部分；空观测用有效的零长度集合及原因表示，禁止靠伪造点继续运行。

`PolicyDecision.raw_request` 和 `protocol_request` 表示一个决策区间的 `[5,2]` 控制块，适配旧接口时可展平为 `[10]`，顺序固定为 `[m0,a0,m1,a1,...,m4,a4]`。策略若只生成常值请求，由适配器显式复制 5 次；已有非恒定块不能只取首个动作再复制。记录器按每个实际 50 Hz 步保存对应的 `[2]`，实际施加值由 runtime 回传，不能用 policy 请求代填。

`supported_floor_patches` 使用世界系三角片顶点数组 `[M,3,3]` 和每片的源相机/frame ID；`visibility_support` 为与这些片对应的像素支持、遮挡边界和有效高度带检查结果。先在同一 geometry 模块完成可见性审查，再只把已获支持的小片交给 mapper 栅格化。采用其他等价结构时，Task 0 的映射报告必须固定其 shape、坐标、有效性及栅格化规则，不能让两个模块分别猜测其含义。

## 5. 坐标、深度和时间合同

### 5.1 坐标

- 单位统一为 m、s、rad。base 使用 x 前、y 左、z 上；光学系使用 x 右、y 下、z 前。
- 采用列向量，`T_target_from_source @ p_source = p_target`。四元数明确采用 wxyz 或 xyzw，进入适配器时转换到一种固定约定。
- 深度统一为光轴 z 深度：`p_optical = depth_z_m * inv(K) @ [u,v,1]`。若 renderer 输出沿光线的距离 `r`，转换为 `z = r / norm(inv(K)@[u,v,1])`，不能直接代入 z 深度公式。
- `p_world = T_world_from_base(t_capture) @ T_base_from_optical(t_capture) @ p_optical`。相机若安装于可动关节上，外参按采集时刻的关节状态计算；固定外参必须由资产核验。
- 图像旋转、裁剪、缩放必须同步更新 K、mask、像素映射。预测恢复后 K 和深度须属于同一像素坐标。禁止只旋转图像不调整标定。
- 历史配置 `(0.20, ±0.30, 0.05) m` 仅用于识别旧 rig，不作为新服务器的硬编码真值。0.05 m 相对球心，不是离地高度。
- 地图数组为 `[iy, ix]`，cell index 使用 floor，cell center 为 `origin+(index+0.5)*resolution`。声明 origin 是单元左下边界。与旧工程的 round/index 约定之间必须显式转换。

### 5.2 深度适配

默认接服务器已有的 DA3 Metric checkpoint，先记录模型名、配置、权重哈希、源码 revision、预处理和精度模式，再测试。不要因为官方更新就静默更换权重。

DA3 官方区分相对深度与米制模型；低层 `DA3METRIC-LARGE` 网络输出存在焦距换算要求，官方 README 给出 `metric_depth = focal * net_output / 300`。高级 API 是否已执行换算，必须读服务器安装版本的后处理代码并写适配测试。禁止低层输出未缩放，也禁止高级 API 输出二次缩放。参考：[官方 README](https://github.com/ByteDance-Seed/Depth-Anything-3/blob/main/README.md)、[官方 API](https://github.com/ByteDance-Seed/Depth-Anything-3/blob/main/docs/API.md)。

单目推理之间共享同一模型实例；允许用 batch 维提速，但不能偷偷变成联合多视图模型。batch 和串行输出需要在固定输入上做数值容差对照。网络输入保留实际遮挡外观，推理后应用自体无效 mask；若改变网络输入的 mask 处理，作为独立预处理版本记录。

NaN、Inf、非正值、超出已验证工作范围、机体遮挡及映射出界的像素均无效；不可将它们替换成最大距离再清图。没有可用 confidence 时写 `quality_kind=unavailable`，不能填全 1 充当可信概率。模型原始 confidence 只称质量分数，未标定前不解释为碰撞概率。

不得逐帧用测试场景真值拟合深度尺度。若需要仿真微调或全局校正，应另开实验，只用训练集拟合、验证集选择，冻结后在留出集评价。

### 5.3 时间

优先保持现有合同：物理、控制、本体状态 50 Hz，相机 10 Hz。状态与图像时间差上限沿用有效现有配置；若无更新依据，左右同步 0.002 s、状态到图像 0.005 s 作为兼容起点。变更必须更新 schema 和测试。

控制和记录只认仿真时钟；推理耗时另用单调墙钟记录。深度沿用源图像时间与 frame ID，不用推理完成时间冒充拍摄时间。姿态插值采用位置线性插值、姿态正规插值；不得使用未来尚未可用的样本。若采用有界等待配准，记录等待延迟并仍以捕获时刻定义观测。

先实现 `lockstep` 功能模式：计算期间不额外推进物理，明确报告仿真时长、墙钟时长和实时因子，不能声称部署实时。额外 render 是否推进 physics 必须检测，禁止通过“补渲染”隐藏增加的物理步。

可选第二阶段实现 `latency_aware`：主线程访问 Isaac；推理 worker 只收图像副本；队列容量 1，覆盖旧待处理项，拒绝跨 episode、乱序和过期结果。结果投影使用捕获姿态，再由当前地图消费。不得在工作线程随意调用 Isaac 对象。

速度限制应覆盖实测停止距离、观测年龄和控制延迟：`required_clearance >= measured_stop_distance(speed,direction) + speed*total_delay + margin`，并检查转弯扫掠范围。这里是设计约束，不是未经标定的安全证明。缺停止曲线时先低速测量。观测年龄、工作距离和速度上限属于开发参数，不能写一个阈值就宣称安全。

## 6. 建图：占据、自由和未知

### 6.1 表示

静态平地 v1 使用世界水平系的有限工作区地图，分辨率初始 0.10 m。边界来自事先声明的任务工作区配置，不读取隐藏障碍几何；越界停止/重新配置，不允许负索引绕回数组。规划窗口和模型 BEV 可从全局记忆裁剪，不能每帧重置长期地图。

三态编码统一为 `free=0, occupied=1, unknown=2`。若旧模块用 -1 表示未知，适配器完成转换。保存观测来源、last_seen、质量和是否已重复确认。障碍观测过期后最多变未知，不能自动变自由。持久障碍被清除必须有更新、可靠、相同空间位置的可见自由证据；弱分数或无效深度不能清除强占据。

### 6.2 地面与自由证据

平地假设必须在配置和 metadata 中声明；地面高度来自允许的初始标定或观测拟合，不能从完整障碍地图隐式读取。按补偿姿态后的高度识别地面与机器人碰撞高度带内的障碍。高度带、拟合残差、深度工作区间以资产和开发集冻结，不把旧 0.5–3 m 或旧高度阈值直接视为通用正确值。

仅有障碍点不能生成可靠自由空间，仅标记稀疏地面命中格又容易造成不连通。v1 必须实现“有可见支持的连续地面小片”：

1. 从有效相邻像素建立局部地面小片或等价密集支持；要求深度连续、地面一致、无 mask、无明显深度断边。
2. 将小片覆盖的地面单元标记为自由候选；支持范围不得跨越遮挡、空洞、台阶或未观测像素。深度 discontinuity 和小片最大边长须配置并测试。
3. 三维可见射线可提供穿过体素的自由证据，但禁止把高处一条射线直接投成整列地面自由。地面通行必须同时有地面支持及碰撞高度带的可见性约束；无法覆盖的低矮障碍空间保持未知。
4. 占据与自由冲突时，本帧可信占据优先；多帧清图按来源、质量和时间执行显式规则。

初始机器人自身已占据体积可作为 `body_support` 证据，仅覆盖核验后的实际占地，不扩成任意安全圆。必须验证该区域与新观测自由地面能形成满足足迹约束的连接。无连接时记录 `no_supported_free_route`，不能强行把起点或邻近未知格设为可走来通过测试。

自体遮挡首先排除静态 mask；可动组件使用随姿态/关节更新的几何 mask，或经验证的保守遮挡包络。模型不能把机体自身当地图障碍。初版未覆盖的动态遮挡必须明确报告。

### 6.3 足迹与地图膨胀

从资产获得包含外突部件的保守碰撞足迹；地图分辨率误差和定位/深度误差加入声明的 margin。推荐在占据图上做一次足迹/误差膨胀，规划中心路径；如另用多边形足迹逐姿态碰撞，则避免再次完整膨胀。未知区也需要按完整机器人扫掠范围检查。

合成地图必须覆盖：窄于足迹的通道拒绝；明确宽于足迹加 margin 的通道通过；边界、拐角及对角跨角不穿障。不能只检查中心点。

## 7. 子目标、规划、执行和回退

- 目标在连通可行自由区内，直接规划到目标区域；否则选择当前连通自由区内、靠近观测边界且具有足迹净空的子目标。子目标本身位于已知自由区，不把未知格作为当前可执行路线。
- 边界评分采用已知空间的路径代价、可观测未知边界潜力、到目标启发距离和访问惩罚；启发距离不使用隐藏地图最短路。保留选中方向，达到子目标、路线失效或持续无进展才切换，避免左右振荡。
- 机器人不具备经验证的原地转向能力时，不能生成原地旋转动作。探索运动必须通过同一运动约束规划与足迹检查。若当前无法获得更多信息，允许可解释地失败，不盲穿未知。
- 复用已有 Hybrid A* / state lattice 中已验证的运动原语和跟踪器；用实测曲率、速度和制动参数，不把球机当理想差速车。不为接入本方案而整体引入 Nav2 或新控制框架。
- 新地图只在路径受影响或保留条件失效时触发重规划，避免每帧路线翻转。记录 plan ID、保留/重建原因、参考终点类型。
- 临时子目标附近可以按制动条件停车并重新观测，但不能将临时端点判成全局成功。真实目标的成功、临时路径结束和稳定停止使用不同字段。
- 深度缺失、姿态失效、无可行路或 worker 失败时，请求现有执行链的保守制动/零请求，继续记录惯性运动和接触。零请求不直接写为 stopped。
- 首版默认要求左右深度均可用；若提供单目降级，必须有显式配置、有效覆盖和速度条件，记录 `degraded_single_camera`，单独测试，禁止静默降级。

## 8. 已知地图数据采集

### 8.1 教师与轨迹类型

教师使用 `known_map_gt_pose`、同一物理执行器和相同评分。采集器订阅实际执行后的记录，不从理想参考路径生成伪轨迹。

先采正常导航：空地、单障碍绕行、长墙绕行、宽窄通道、转弯和目标接近。再采受控恢复：初始航向/位置偏差、不同初速度、减速、停下后起步和允许的换向。扰动从核验的可行初态开始，不任意传送到障碍里。不同类型独立标记，不把所有数据叫专家成功示范。

小批试采建议 20–30 个 episode，以数据链路验证为目的；其中保留可解释失败。审计和回放通过后再按场景与动作覆盖缺口扩充，不直接启动大规模长时间采集。建议预算不是模型收敛保证。

### 8.2 数据字段和文件

优先兼容现有 `CollectionContract`、`collection_schema.py` 和 writer。新增语义使用版本化 sidecar；若核心数组语义发生改变，升级 schema 并提供转换/读取适配，不能只往 metadata 填一个新版本号。

逻辑布局如下；实际目录可以沿用旧结构，但必须建立字段映射：

```text
dataset_manifest.json
splits.json
episodes/<episode_id>/
  metadata.json
  online/                 # RGB、可用本体/估计状态、校准、时间
  truth/                  # 真值位姿、当前可见GT深度、BEV等
  controls.npz            # 所有50Hz动作层和状态反馈
  frames.jsonl            # 图像/状态/深度ID与时间映射
  events.jsonl            # 接触、制动、重规划、错误
  summary.json
  COMPLETE                # 最后写入的完成标记
```

`metadata` 至少包含：schema、mode、scene/layout/episode/seed、机器人资产 hash、rig ID、图像尺寸与 K/外参、仿真版本、代码提交、dirty diff hash、深度模型/权重、控制器/动作协议、地图分辨率、所有参数、反馈来源、时间模式、split、termination。

50 Hz 记录：physics_step、仿真时间、关节与本体反馈、`raw_request[2]`、`protocol_request[2]`、`applied_actuator_target[2]`、各层时间、接触标志/接触力。若执行链有 PID 输出和平滑前后两层，全部保留。`applied` 是送给执行器的实际目标，不是实测关节速度；实测关节状态另存。

10 Hz 记录：左右原始 RGB、frame ID、各自捕获时间、标定、姿态配准来源、当前观测与累计观察 mask、可选预测深度。真值深度和完整占据标签存到 truth，模型深度另存，不能混名。左右拼接视频仅作展示。

转移定义固定为：观测 `o_t`，随后区间 `[t,t+0.1s)` 内的 5 个 50 Hz 实际动作，随后观测 `o_(t+1)`。不要把结束后生成的动作放进前一段。历史 5 帧在 10 Hz 下最早到最晚相隔 0.4 s；未来 5 个 0.1 s 转移覆盖 0.5 s，二者不能混淆。终止附近不足一个动作块的记录保留为部分区间，训练窗口显式排除或提供 mask。

数据写到唯一临时目录，flush/必要的 fsync 后校验，再在同文件系统原子改名并写完成标记。仅 COMPLETE 且 audit 通过的 episode 进入正式 manifest。失败、超时可为完整 episode；程序中断是 incomplete，不能伪造成 timeout。

断点续采以“已经完整完成的 episode”为单位跳过；同名但配置、commit、seed 或 checkpoint hash 不同必须拒绝合并。v1 不在半截 episode 中恢复物理状态，默认从其初态重新采，保留旧 incomplete 诊断文件。

### 8.3 划分和标签

在采集前按完整 layout/scene family 固定 train/val/test，建议 70/15/15 作为足量场景时的起点；场景太少先补场景，不用随机切相邻帧制造比例。相同几何仅换纹理不能宣称几何泛化。来自同一轨迹、上下文和动作分支的样本归同组，禁止跨 split。

BEV 同时提供真实占据与当前/历史可见 mask，不把完整地图标签当成全部已观测。未来运动标签相对当前水平 base 坐标系：`delta_xy_k = R(-yaw_t) @ (p_(t+k)-p_t)`；航向用明确的 wrap 或 sin/cos。训练 loader 显式白名单选择 online 字段，truth 只进入 loss/评估路径。

## 9. 同状态动作分支：后续增强

先复用已有 `collect_candidate_rollouts.py` 做代码核对，不假设其恢复逻辑已满足本设计。每个 context 可先用 8–16 个候选、5 个 0.1 s 动作块，候选覆盖中性请求、前进、转向、制动和已允许的换向；边界来自当前动作协议，不硬编码新限幅。

根状态一致不代表完整上下文一致。至少恢复根位置/姿态/速度、关节状态、PID 积分和滤波器、smoother/supervisor 历史、随机状态，以及会影响执行的缓存。若仿真无法直接完整恢复，可从同一初态重放相同前缀，再核对分支起点的可观察状态和重复中性 rollout；不能只传送根位置。

分支是仿真局部动作后果数据，不宣称现实反事实真值。保存 context ID、前缀 hash、恢复方式、起点差异和重复 rollout 误差。同一候选重复运行的误差高于预先冻结的系统容差时，该 context 不进入正式候选排序训练。状态不能可靠恢复时将此阶段标记 BLOCKED，正常轨迹采集仍继续交付。

## 10. CLI 和配置要求

新入口可以采用以下命令形式，均为待实现接口示例，不是已验证可执行命令：

```bash
python -m scripts.run_navigation_baseline --config configs/navigation_baseline_v1.yaml --mode known_map_gt_pose --manifest <manifest> --output <run_dir>
python -m scripts.collect_navigation_dataset --config configs/navigation_baseline_v1.yaml --manifest <collection_manifest> --output <dataset_dir> --resume
python -m scripts.evaluate_navigation_depth --config <config> --manifest <depth_manifest> --output <depth_report_dir>
python -m scripts.audit_navigation_dataset --root <dataset_dir> --strict
```

若必须使用 Isaac 自带 Python/launcher，README 写明真实命令和前置环境，不提供一个在普通 Python 下必然失败的“一键命令”。配置解析必须拒绝未知 mode、错误单位、负分辨率、非法 shape、未知关键参数、未声明的 checkpoint 以及同目录覆盖。

缺权重/Isaac/GPU 时返回非零退出码和具体原因；不能回退到 mock 后输出“导航成功”。CPU contract 测试可使用假深度，真实模型和仿真测试必须明确区分。

## 11. 分层验收与实验规则

验收分成“软件正确性”“真实运行证据”“导航效果”。前者通过不保证后者通过。

### 11.1 必须通过的软件正确性

- 所有新增合同、几何、建图、规划、记录与审计测试；关键旧回归与修改前结果相比无新增失败。
- 来源隔离、时间/动作对齐、坐标投影、非法输入、文件完整性检查没有已知错误。
- 人为接触能被真实物理接触检测触发；地面接触不误判为障碍；首个有效碰撞不能被稍后到达覆盖。
- 权限访问必须有运行期证据，如 guard/spy provider；仅搜索源码不够。视觉策略尝试调用 truth/full-map 接口应报错。

### 11.2 深度诊断

先审查覆盖静止、前进、倾侧/转弯、近障和自体遮挡的 100–300 对开发帧。分别报告有效覆盖、米制深度/三维误差、障碍边缘误差、近距障碍漏检、虚假障碍和建图后的自由误判。所有关键障碍区域都进入分母，无效输出单列且计入漏检；不能只在双方都有效的像素上宣布通过。

不预设“模型一定够好”的通过率。先由独立真实深度诊断确定建图可行性，再根据足迹、速度、制动和误差预算冻结 DA3 工作范围及 margin，生成 `acceptance_profile.json`。参数选择只用开发/验证集，正式留出集不调参。若深度偏差大于可分配净空预算，明确判定视觉版本性能未就绪。

真实 GPU 推理测预热和稳态的 p50/p95、显存峰值、左右一对总耗时。无 Isaac 并发的测试和与 Isaac 并发的测试分别报告。不达到 10 Hz 不等于软件实现错误，但必须如实降低感知频率/速度并重新验证；不能伪造 10 Hz 实时性。

### 11.3 最小仿真矩阵

先用 1 条 5–10 s 短任务检查 runtime，再执行固定 E0 空地、E1 必须绕障、E2 通道任务，各 3 个基础模式，共 9 个完整 episode。估计定位模式完成后加入相同任务，不能混在基础模式统计中。主表沿用核实后的原规则；默认兼容 35 s、0.20 m 目标半径、50 Hz 物理接触检查。不同预算的新试验单独出表。

E1 要求起终点直线被真实膨胀障碍阻断且存在可行路线；E2 除到达外单独检测实际足迹/轨迹通过声明通道的门线序列，不能凭参考路径经过通道就称通道通过。地图和门线可以给评估器，不能给视觉策略。增加一条有意接触和一条不可达任务用于评分验证，不计入成功率。

E0 的已知地图和可见真值深度版本应各至少有一条无碰撞到达，否则不扩大视觉实验，先分析执行或地图连通性。E1/E2 和 DA3 不承诺达到率；如完整运行失败，必须交付完整失败分类，不能写“基线已完成且可用”。后续稳定性评估再固定独立布局和重复种子，报告样本数与不确定性，不从 9 条 smoke 推断总体性能。

统一报告到达、碰撞、超时、异常退出、路径长度、时间、剩余距离、物理接触、停稳状态、感知有效率、重规划次数、参考跟踪误差和墙钟耗时。到达后停稳为额外指标，不能静默改旧成功定义。

### 11.4 数据验收

试采 20–30 个 episode，strict audit 全部通过后才能进入正式 manifest；随机抽查至少 3 条，用 RGB、轨迹、接触和动作层回放核对。模拟中断和恢复采集，验证 incomplete 不进入训练，resume 不重复、不覆盖、不混版本。跨 split 几何/episode/context ID 冲突为硬失败。

## 12. 交付内容与后续世界模型接口

服务器 AI 提交代码、配置、测试、实际 README 命令、环境清单、改动报告、验证报告与小样本产物索引。每个阶段写 PASS/FAIL/BLOCKED/NOT_RUN；最终报告分别写：已知地图采集是否可用、可见深度导航是否可用、DA3 视觉是否达到当前效果目标、估计定位/动作分支是否已完成。

为世界模型保留两路原始图像、历史动作、可用本体状态、当前/历史观察 mask、未来相对运动、候选动作分支和所有执行层。后续先比较同一视觉与记忆下的解析运动预测和学习运动预测/评分，不在本轮同时重写感知、控制和世界模型。只有固定共同输入和执行条件后，才评价预测模型是否改善真实动作选择和闭环到达。

## 13. 本设计的已知限制

本设计针对平地、静态障碍和现有球机资产。地面自由证据、实际停止距离、遮挡包络、DA3误差和实时能力需要服务器实测。真值辅助视觉版本和已知地图教师分别属于声明过权限的实验条件，不是现实自主导航能力证明。本包不包含服务器当前仓库、模型权重或仿真资产，也不宣称已复现历史结果。
