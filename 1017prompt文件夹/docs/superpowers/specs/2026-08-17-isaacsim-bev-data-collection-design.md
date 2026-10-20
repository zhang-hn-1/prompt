# Isaac Sim BEV 数据采集设计

## 1. 目标与范围

本次修改将 `research/ball_wm/scripts/collect_data_dubins.py` 升级为符合 Phase 0 schema `1.1.0` 数据契约的正式原型采集器。采集器同时记录同步双目、本体状态、二维动作、统一时间戳，以及由 Isaac Sim 场景碰撞几何产生的障碍物真值。

本阶段只处理静态环境，不标注障碍物类别、实例或速度。`collect_data.py` 保留为旧的固定动作原型，并在文件与 README 中明确标记为 legacy，避免同时维护两套正式格式。

成功标准：用户能够运行一个短 episode，在输出目录中看到双目图像、局部 BEV 图、可见性 mask 和通过审计的 `states.npz`。

## 2. 总体方案

采用“场景几何真值 + 双目可见性”方案：

1. Isaac Sim 4.5 的 `isaacsim.asset.gen.omap` 从静态 USD/PhysX 碰撞几何生成世界系全局占据地图；
2. 每个双目采样时刻，根据 base 的世界位姿裁剪并旋转成 base 系 `80 × 80` 局部 BEV；
3. 左右相机的深度输出仅用于计算当前双目可见区域，不作为部署输入；
4. 将当前可见 mask 按机器人相对位姿累积成历史已观测 mask；
5. 完整几何 BEV、当前可见 mask 和历史已观测 mask 一同保存。

机器人自身 `/World/BallRobot` 必须从地图真值中排除。全局占据图只在场景加载和物理初始化后生成一次；静态环境中不逐帧重建。

## 3. 频率与采集时序

严格使用 `data_contract.yaml` 中的频率：

- 物理、状态与控制：50 Hz；
- 同步双目与 BEV：10 Hz；
- physics dt 为 0.02 s，动作每个 physics step 更新。

主循环唯一允许推进物理的位置是显式 `world.step()`。相机 pose、状态和标注 getter 不得推进仿真。

每个物理步按以下顺序执行：

1. 若到达控制边界，计算、限幅并应用二维动作；
2. 调用一次 `world.step()`；
3. 记录该步状态、当前保持动作和仿真时间；
4. 若到达视觉边界，从同一已完成 physics step 读取左右 RGB、左右深度、相机 pose 和局部 BEV；
5. 左右图使用同一个 `stereo_frame_id` 与同一个 `physics_step`。

10 Hz 与 50 Hz 通过整数 physics step 调度：视觉间隔固定为 5 步，控制边界为每个 physics step。因此相邻视觉帧之间固定包含 5 个状态/动作样本，加载器通过时间戳截取完整动作窗口。

## 4. 动作与状态

动作固定为：

```text
[main_joint_velocity_cmd, auxiliary_joint_angle_cmd]
```

维度为 2，删除旧格式恒零第三维。保存：

- `commanded_action`：控制器生成的原始二维命令；
- `applied_action`：软件限幅后实际传给 articulation controller 的二维命令。

原始状态保存世界系 base pose、线速度、角速度及三个关节的位置和速度。相机 pose 作为标定与标签重建元数据保存，但不属于模型状态。

## 5. 障碍物真值与可见性

### 5.1 完整几何真值

全局占据地图的分辨率为 0.1 m。局部裁剪遵循：base `+X` 对应图像上方，base `+Y` 对应图像左侧，机器人位于局部图中心。类别编码固定为：

```text
0 = free
1 = occupied
2 = unknown
```

局部图保存为 `uint8[80,80]`。地图生成失败、场景没有有效碰撞体或地图原点被占据时，采集器立即报错，不静默生成全 free 标签。

### 5.2 当前可见性

左右相机各自输出与 RGB 对齐的 `distance_to_image_plane`。有效深度像素通过相机内参和固定外参投影到 base 系，并沿视线标记 free、在终点附近标记 occupied。左右结果取并集，产生 `current_visible_mask[80,80]`。

深度数值为非有限值、超出 8 m 局部范围、命中机器人机身或落在地面的像素不参与障碍物终点标注。

### 5.3 历史已观测区域

每个 episode 开始时清空历史 mask。新视觉帧到来时，先用相邻 base 位姿将上一历史 mask 变换到当前 base 系，再与 `current_visible_mask` 取并集，得到 `history_observed_mask`。

训练时：

- 当前可见区域监督即时空间编码；
- 历史已观测但当前不可见区域监督空间记忆；
- 从未观测区域不强制模型猜测完整几何答案，规划时视为 unknown 风险。

## 6. 输出格式

每个 episode 使用一个目录：

```text
ep0000/
  left_000000.png
  right_000000.png
  left_depth_000000.npy
  right_depth_000000.npy
  geometry_bev_000000.png
  current_visible_000000.png
  history_observed_000000.png
  preview_000000.png          # 仅在 --save_preview 时生成
  states.npz
  metadata.json
```

RGB 使用无损 PNG，避免 JPEG 压缩影响双目对应。深度使用 float32 NPY；BEV 和 mask 使用保留离散类别值的 uint8 PNG。文件编号使用连续 `stereo_frame_id`，不再混用 physics step 作为图像编号。

`states.npz` 至少包含：

- `physics_step[int64,Ns]`、`sim_time[float64,Ns]`、`state_timestamp[float64,Ns]`；
- `pos[Ns,3]`、`quat[Ns,4]`、`lin_vel[Ns,3]`、`ang_vel[Ns,3]`；
- `joint_pos[Ns,3]`、`joint_vel[Ns,3]`；
- `commanded_action[Ns,2]`、`applied_action[Ns,2]`；
- `action_command_timestamp[Ns]`、`action_applied_timestamp[Ns]`；
- `stereo_frame_id[Nv]`、`stereo_physics_step[Nv]`；
- `left_timestamp[Nv]`、`right_timestamp[Nv]`；
- `cam_left_pose[Nv,4,4]`、`cam_right_pose[Nv,4,4]`；
- `collision[Nv]`、`minimum_obstacle_distance[Nv]`。

`metadata.json` 保存 schema 版本、频率、单位、坐标约定、固定的 `640 × 480` 图像大小、BEV 参数、场景路径、随机种子、文件命名规则，以及 `waypoints_world_xy`。

`minimum_obstacle_distance` 定义为局部几何 BEV 中机器人中心到最近 occupied 网格中心的距离减去球半径，单位为米；没有 occupied 网格时为正无穷。`collision` 定义为该间隙小于或等于 0。它是二维导航碰撞标签，不替代三维 PhysX 接触力测量。

## 7. 模块边界

- `scripts/collection_schema.py`：纯 Python 数据结构、频率调度、时间戳和 episode 写入，不导入 Isaac Sim；
- `scripts/bev_ground_truth.py`：Occupancy Map 初始化、世界系地图、base 系局部裁剪、深度可见性和历史 mask 变换；
- `scripts/collect_data_dubins.py`：仿真初始化、控制和唯一的物理主循环；
- `scripts/audit_dataset.py`：离线检查 shape、dtype、数值有限性、时间单调性、频率、固定 RGB 尺寸、文件配对和类别取值；
- `tests/test_collection_schema.py`：二维动作、调度和时间语义测试；
- `tests/test_bev_ground_truth.py`：局部坐标变换、裁剪方向、mask 累积测试，不启动 Isaac Sim。

Isaac Sim 依赖封装在 `bev_ground_truth.py` 的运行时适配层中，使几何变换和文件格式可以在普通 Python 环境测试。

## 8. 命令行与失败处理

新增参数：

```text
--physics_hz 50
--control_hz 50
--stereo_hz 10
--bev_extent_m 8.0
--bev_resolution_m 0.1
--no_save_depth
--save_preview
--audit_after_episode
```

正式频率固定为 physics/control/state/stereo = 50/50/50/10 Hz。CLI 频率参数仅用于显式校验；任何不等于这些固定值的传入值都必须在启动时拒绝，不允许选择替代频率。相机帧为空、左右帧编号不一致、深度 shape 错误或 BEV 生成失败时，中止当前 episode 并输出明确原因，不写入看似完整的数据。

深度默认保存；只有明确传入 `--no_save_depth` 才不落盘，但可见性仍在内存中计算。`--save_preview` 生成 RGB、完整 BEV和两个 mask 的拼图，供用户运行短程序后目视检查。preview 默认关闭以减少 I/O。

## 9. 测试与验收

无需 Isaac Sim 的自动测试覆盖：

- 50/50/50/10 Hz 调度；
- 动作为 float32 二维；
- 时间戳单调且左右目同 physics step；
- world 点到 base BEV 像素的方向约定；
- 机器人旋转后的局部裁剪；
- 历史 mask 的自运动补偿；
- 数据审计拒绝旧三维动作、缺失文件和非法类别。

Isaac Sim 冒烟测试命令采集 1 个短 episode，开启 preview 和 episode 后审计。验收结果必须满足：无隐藏 physics step、审计通过、预览中障碍物方向与 RGB/场景一致、机器人自身不出现在局部 BEV 中。

## 10. 非目标

- 不修改世界模型训练代码；
- 不加入动态障碍预测；
- 不把深度、完整几何地图或世界 pose 作为部署输入；
- 不把未知区域伪造为 free；
- 不在本阶段实现真实机器人传感器同步。
