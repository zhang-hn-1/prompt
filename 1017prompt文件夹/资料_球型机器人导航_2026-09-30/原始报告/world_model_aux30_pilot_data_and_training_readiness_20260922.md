# Aux30 世界模型试采与训练就绪报告（2026-09-22）

本轮完成了当前限速协议下的小批真实仿真数据、数据质检、训练配置和一次真实数据前向/反向传播。**尚未训练新的 checkpoint，也没有新增世界模型导航成功率。** 这批数据用于打通已有世界模型流程，规模不足以代表完整训练集。

上一阶段已实跑复现 golden Oracle E0/E1/E2：3/3 成功、零碰撞、零超时。该结论来自独立的冻结复现包；本轮没有重跑正式 benchmark，也没有改变 Oracle 的规划或换向控制规则。

## 1. 实际交付与划分

- 工作树：`/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark`。
- 数据：`/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/data/wm_aux30_pilot_20260922_v1`。
- 原始日志、每条质检、脚本与证据：`/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/wm_aux30_pilot_20260922_v1`。
- 实际完成 **8 条完整序列，4800 个 50 Hz 物理状态，960 对 10 Hz 双目图像**；共 96 秒仿真。每条 600 tick/120 对图像，RGB 为 640×480，BEV 为 80×80。
- 每帧有左右 RGB、几何 BEV、当前可见与历史可见标签；另有状态、标定、原始请求及执行链日志。深度用于生成可见性标签，本批未落盘深度和高度图。
- 训练：easy_1、easy_2，4 条完整序列；验证：easy_4，2 条；测试：easy_5，2 条。按完整场景/序列隔离，没有随机拆相邻帧。
- 实际可加载窗口：训练 **444**、验证 **222**、测试 **222**。每窗历史 5 帧、未来 5 帧；同一集合内部窗口可以重叠。验证/测试沿用相同动作模板，只隔离场景，不能据此证明对新动作序列的泛化。
- golden/hard_0/专用 detour/corridor 不在本批训练场景中。easy 场景历史上已用于其他研究，本次测试集仅相对本次训练隔离，不能称为研究全过程 fresh unseen。

采集清单及左右命令序列在采集前冻结。每场景各运行一条左右镜像命令，但起始种子不同，**不是相同初态的控制 A/B**。每条内容为：直行 2 秒 → 一侧转向 3 秒 → 反侧转向 3 秒 → 回正 2 秒 → 请求减速 2 秒。主轴请求 0/0.7/1.0 rad/s，副轴请求 ±30°。没有主轴倒车、前后换向和目标导航任务；“请求减速”不等于已满足 B0 的停稳资格。

## 2. 实测执行与质量

| 序列 | 路径长度 m | 碰撞标记帧 | applied 超过16.1°的物理样本 | 副轴 supervisor 触发 tick | 副轴实测峰值 rad/s |
|---|---:|---:|---:|---:|---:|
| easy_1_left | 2.805 | 0/120 | 125 | 14 | 0.316 |
| easy_1_right | 2.817 | 0/120 | 129 | 16 | 0.320 |
| easy_2_left | 2.805 | 0/120 | 123 | 14 | 0.318 |
| easy_2_right | 2.817 | 0/120 | 129 | 16 | 0.320 |
| easy_4_left | 2.805 | 0/120 | 125 | 14 | 0.316 |
| easy_4_right | 2.817 | 0/120 | 129 | 16 | 0.319 |
| easy_5_left | 2.805 | 0/120 | 123 | 14 | 0.318 |
| easy_5_right | 2.817 | 0/120 | 129 | 16 | 0.320 |

碰撞标记沿用已有 BEV/球体间隙定义，不等于新的独立接触力测量。路径长度也不是导航 SPL 或导航成功率。

8 条全部通过 schema 审计和额外的因果对齐审计：每步只推进一个物理 tick；相机帧号递增；左右相机均对齐当前世界时间，最大误差 **0 秒**；物理 dt 最大误差 4.47e-10 秒。

每个 request 与冻结计划逐样本一致，protocol 与裁剪重建一致；以当时真实关节反馈重建的 smoother/applied 与日志一致，applied 最大差为 0。采用与成功 Oracle 相同的主轴目标变化率 0.5 rad/s²、副轴目标变化率 30°/s、副轴速度 supervisor 阈值 0.30 rad/s。

**0.30 是反馈 supervisor 的触发阈值，不是物理角速度的严格硬上界。** 表中的短时峰值仍可能超过该值，不能声称关节速度从未超限。本批没有为降低峰值重新调整增益或速度策略。

![实测试采轨迹](/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/wm_aux30_pilot_20260922_v1/pilot_trajectories.png)

![执行链与时间对齐](/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/wm_aux30_pilot_20260922_v1/pilot_execution_alignment.png)

## 3. 修复了哪些会妨碍训练的问题

1. **采集执行协议未完整接上当前限速链。** 为采集入口接入现有 ActionSmoother 的真实反馈 supervisor，增加可复现种子、冻结请求序列和逐 tick 执行记录。没有重新拟合动力学模型。
2. **图像、动作和状态的实际时间可能错开。** 原采集器在 stereo tick 使用 `step(render=True)`，会按渲染时间推进额外物理步。现在物理步进与渲染分开。相机内部 10 Hz 浮点计时门又会把实际 0.0999999978 秒判为不足 0.1 秒；一次完整拒绝样本中 60/120 对图像重复。现由采集调度唯一决定 10 Hz，Camera 显式设置每次渲染更新。启动就绪检查要求真实时间对齐；若异步渲染回调仍未更新，在物理状态冻结时有限次刷新，不能推进物理来等图像。每条最多实际补刷 3 次，总计 10 次。
3. **未来输入曾使用尚不可知的 applied action。** 新配置明确使用 `commanded_action` 作为未来动作条件；当前/历史 applied 仍可以作为已知本体反馈。旧配置默认语义保留。数据窗口使用离开该图像的请求，正式适配器中的历史请求相应左移一格，避免一帧错位。
4. **多步预测动作顺序错误。** 原实现效果是依次使用 a0、a0、a1，最后动作未正确参与末步预测；已修成 a0、a1、a2。测试证明改变后续动作只影响相应步及之后，不改先前预测。这会改变旧模型的多步输出，不能把旧 checkpoint 的旧指标直接当作修复后的指标。
5. **可见性标签生成过慢。** 保留全部有效深度像素、半格射线采样和原网格定义，采用可选编译内核。640×480 合成深度对比：标量 7.242 秒/相机，warm 中位 0.130 秒，约 55.6 倍；这不是端到端采集加速比。NumPy 回退可用；NumPy/Numba 双后端均通过冻结原实现的等价检查。numba 0.61.2、llvmlite 0.44.0 装在独立缓存目录，没有修改共享 conda 环境。

相机修复前后，同种子 easy_1_left 的 pos、quat、lin_vel、ang_vel、joint_pos、joint_vel、commanded_action、applied_action 八类数组逐项完全相同，证明该对照中传感器修复没有改变物理轨迹。

## 4. 真实训练通路检查

配置：`research/ball_wm/configs/train_joint_pose_aux30_pilot.yaml`。保持现有 6 通道双目 ViT-tiny、192 维 latent、pose head、BEV/可见性监督；新动作输入明确为当前 aux30 请求协议。

实际从本批训练数据读取一个 batch，在 CPU 完成前向及反向传播：所有损失有限，**287 个梯度张量有限**，总损失 4.732428。验证集与测试集各检查一个样本可加载。该步骤优化器更新 **0 次**，未保存模型；随机初始化损失没有导航性能含义。

运行日志的最终回归结果：**1142 passed, 1 xfailed, 3 warnings in 14.62s**。本次排除了此前长时间未完成的 `test_dynamic_lattice_planning.py`，因此不声称整套测试全通过。其余覆盖包括数据、模型、动作顺序、采集、协议和已有 benchmark 回归。`git diff --check` 通过。

直接启动预定的 10 epoch 小规模训练：

```bash
/home/an/miniconda3/envs/ball_wm/bin/python /home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/wm_aux30_pilot_20260922_v1/train_pilot.py
```

入口先读取 8 条质检和真实 batch 检查的已保存通过记录（不会重新运行审计或核对数据哈希），然后从头训练：batch_size=2、lr=5e-5、seed=3072、GPU、固定场景划分；不加载旧 aux16 checkpoint，不覆盖已有输出。输出为 `/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/wm_aux30_pilot_20260922_v1/training_10epochs`。本报告生成时尚未执行此命令。

## 5. 保留的限制与下一步

- 当前数据只有 96 秒、八条开放环请求序列；适合检查可学性与通路，不足以覆盖完整导航、倒车换向、复杂障碍或各速度域。它不是成功 Oracle 示范数据，也不证明世界模型能控制机器人。
- 本体速度输入仍沿用当前采集器与正式 runtime 的世界坐标约定，二者一致；但数据契约文字写的是 base 坐标，不能声称已满足该条契约。本轮没有悄悄改变历史数据坐标。
- `states.npz` 的 action 时间戳沿用物理区间完成时刻；真正的指令施加前/物理步后时间、关节反馈与四层动作以 `execution_trace.npz` 为准。
- 相机沿用现有安装和标定。检查图像可以看到侧旋，标定显示光轴约沿机体 +X、图像右向约沿机体 +Z；本轮没有调整安装方向或对 RGB 单独旋转。未来若改安装，需同步改标定并重新采集。
- 正式 benchmark runtime 的视觉采样还没有执行本轮这种实测时间戳审计。Oracle 成功不能替代视觉时序正确性；世界模型首次闭环前必须迁移并验证相同的时间对齐保障。V2 适配器的报告标签还保留 legacy 名称，需要按真实协议修正后再引用。
- 下一步直接用冻结配置训练唯一小规模候选，检查独立场景短时位姿/BEV预测和动作敏感性，再接入小规模真实闭环。闭环要保留原 35 秒规则并与冻结 Oracle 比较，分别报告预测质量和导航结果。若失败，应区分训练、模型预测、规划消费与执行问题，不回到无边界标定。

## 6. 可追溯证据

采集清单及 8 条实际运行记录：`collection_manifest.json`、`*_execution.json`、`*.log`；逐条源码 SHA 应相同。源快照：`source_sha256.json`、`source_snapshot.tar.gz`，工作树原有修改未清理、未提交、未上传。

完整质检：`pilot_quality_report.json`、`dataset_stats.json`、`schema_audit.log`；模型通路：`training_batch_check.json`；性能：`visibility_acceleration_benchmark.json`；传感器物理等价：`camera_fix_physics_equivalence.json`；回归：`regression_tests.log`；最终数据文件指纹：`dataset_sha256.json`。

所有被拒绝/中断样本移到运行证据目录，未留在训练数据根目录：初期慢速试采、一次完整相机时序不合格样本、四次在启动阶段被拒绝的尝试，以及两次只观察初始化的相机专项诊断。它们不计入上述 8 条合格序列，也不隐瞒为“首次运行成功”。其中 Kit 曾在中断/不完整时返回 0；采集 driver 因此还要求 episode 已正式完成并通过质检，不能只看退出码。
