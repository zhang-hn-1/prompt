# ball_wm — PointGoal 联合世界模型与 BEV 数据采集

## 项目概述

本目录包含球型机器人 PointGoal 导航的联合世界模型研究代码，以及 Isaac Sim
同步双目、状态、动作和 BEV 教师标签采集器。当前采集契约是
`configs/data_contract.yaml` 的 schema 1.2.0；`scripts/collect_data_dubins.py` 是正式入口，
`scripts/collect_data.py` 仅保留为旧数据格式参考，不能用于生成新训练数据。

模型部署输入限于同步双目 RGB、本体状态历史、动作历史和规划器使用的相对
PointGoal。深度、完整几何 BEV 和世界位姿是训练或评估真值，不是部署期隐藏输入。

## 固定采集契约

- 物理、控制和状态均为 50 Hz；同步双目、深度及 BEV 为 10 Hz。固定频率顺序为
  `physics/control/state/stereo = 50/50/50/10 Hz`，CLI 频率参数只做一致性校验。
- 动作固定为二维 `float32`：
  `[main_joint_velocity_cmd, auxiliary_joint_angle_cmd]`。同时保存控制器请求的
  `commanded_action` 和限幅后发送给 articulation 的 `applied_action`。
- 每个循环先应用动作，只调用一次 `world.step()`，随后记录已完成 physics step 的
  状态；每 5 个 physics step 在同一完成步采集左右目、深度和 BEV。
- BEV 为 base 系 `uint8[80,80]`，覆盖 8 m × 8 m，分辨率 0.1 m；机器人在中心，
  图像上方是 base `+X`（前），图像左侧是 base `+Y`（左）。类别为
  `0=free, 1=occupied, 2=unknown`。
- `current_visible` 是左右深度视锥可见区域的并集；`history_observed` 是最近 5 个
  视觉帧（含当前帧）经过自运动补偿后的历史并集。

完整定义见 `../Ball_Nav/PHASE0_RESEARCH_CONTRACT.md` 和
`configs/data_contract.yaml`。

## `±30°` 辅助转向物理验收门

动作协议 V2 在正式采集或 benchmark 评测前，必须先用低速、地面专用探针确认机器人
确实能够执行契约中的左右极限角，而不只是配置文件写成了 `±30°`：

```bash
PYTHONPATH=research/ball_wm /home/an/miniconda3/envs/ball_wm/bin/python \
  research/ball_wm/scripts/probe_auxiliary_limits.py \
  --contract research/ball_wm/configs/data_contract.yaml \
  --out research/ball_wm/runs/benchmark_v1/steering_probe --gui
```

探针会分别重置机器人，以 `0.5 rad/s` 主关节目标速度执行左右转向；主轴命令默认按
`0.5 rad/s²` 平滑加速，约 1 秒到达目标速度，避免启动时的速度阶跃。每个方向先静置
50 个物理步，再行驶 100 个物理步。轴2目标默认按 `30°/s` 从 0 平滑移动到极限，
而不是一步跳到 `±30°`；三组对比后默认采用 `Kp=300, Kd=60`。可用
`--main-accel-rad-s2`、`--aux-kp`、`--aux-kd` 和 `--aux-rate-deg-s` 做受控调参。
结果写入 `steering_probe.json`，轨迹写入 `steering_probe.png`。自动验收要求：关节实测
绝对值不超过 `30.5°`、命令跟踪误差不超过 `3°`、轴2峰值速度不超过 `40°/s`、
左右 yaw 变化符号相反且各自至少 `3°`、球心高度相对行驶起点波动不超过 `5 cm`，
并且无非预期碰撞。

根坐标系 roll 仍逐步记录，但只作为振荡诊断量，不参与通过判定：球形机器人正常
滚动时根坐标系可以旋转超过 `20°`、`60°` 甚至完整一周，不能据此判断翻倒。
JSON 中的 `contact_force_norm` 是地面接触诊断量；探针场景只有地面，因此地面接触
本身不算碰撞。

只有同时满足以下两项才允许开始正式采集：

1. `steering_probe.json` 顶层 `passed` 为 `true`；
2. 人工查看 Isaac GUI 和 `steering_probe.png`，确认两条轨迹真实地向相反方向弯曲，
   机器人没有翻倒、穿模或异常抖动。

任一条件失败时，应保留 JSON 和 PNG 作为诊断证据并停止正式采集，不得绕过验收门。

正式采集器和闭环导航评估器也使用同一套动作执行策略：主轴速度目标按
`0.5 rad/s²` 限制加速度，轴2目标按 `30°/s` 限制变化速率，副轴关节增益为
`Kp=300, Kd=60`（主轴 `Kd=80`）。`commanded_action` 保存策略请求值，
`applied_action` 保存经过上述物理限幅后实际发送给 Isaac 的值；复位时滤波器会从零动作
或恢复状态中的 `applied_action` 重新开始，避免跨 episode 的动作跳变。采集器可用
`--main_accel_rad_s2`、`--aux_kp`、`--aux_kd` 和 `--aux_rate_deg_s` 做受控对照，评估器
固定读取共享默认配置。

## 环境与入口

从仓库根目录运行，并使用安装了 Isaac Sim 4.5/Isaac Lab 的 Python：

```bash
/home/an/miniconda3/envs/ball_wm/bin/python \
  research/ball_wm/scripts/collect_data_dubins.py --help
```

## PointGoal V1 golden benchmark

2026-09-29 的后续实测时间规划/速度门实验在同版物理接触、35 秒 hard_0 七条中由 3/7 提升至 5/7，golden E0/E1/E2 仍为 3/3；0002、0004 依旧超时。该实验保存在 `runs/benchmark_oracle_measured_time_hybrid_20260929_v9`，报告见 [docs/benchmark_oracle_measured_time_hybrid_navigation_20260929.md](../../docs/benchmark_oracle_measured_time_hybrid_navigation_20260929.md)。原默认 Oracle 未替换。


2026-09-29 起，正式入口默认 `--collision-mode physical_contact_v2`，评测协议为
`ball_pointgoal_v1_2_contact_35s`：每个 50 Hz 物理步检查过滤后的障碍接触力 >1 N，
块内锁存至 10 Hz 状态，地面不算障碍。地图净空继续记录风险，不再直接判碰撞；
接触数据不可用返回运行错误。`collision_audit.json` 保存逐步可用性和接触证据，
包括未发布的失败目录。新模式的 `physical_contact` 是本动作块曾接触，不是瞬时值。
历史复现须显式指定 `--collision-mode map_clearance_v1`，继续使用
`ball_pointgoal_v1_1_35s`；两种协议不能混合汇总。35 秒时限与控制器保持不变。
本次修复和五条真实回归见
[碰撞判定修复报告](../../docs/benchmark_oracle_collision_adjudication_fix_20260929.md)。


`configs/benchmark_v1/golden_v1_manifest.json` 是当前冻结的最小可复现实验集，包含一项 E0 开放目标、一项 E1 单障碍绕行和一项 E2 窄通道任务。manifest 保存场景、占据图、起终点、任务分类证据和 SHA256；E1/E2 的场景源以及 E2 的占据图和 corridor mask 也随仓库保存。E0 使用 NavDP 的 `cluttered_hard/hard_0` 场景，运行时通过 `SCENE_ROOT` 提供该外部场景库。

E2 场景中的 Cylinder/Sphere 同时写入与物理尺寸一致的 `double radius/height` 和显式 `extent`，以避免 Isaac Sim 将未解析的几何属性退化为单位半径碰撞体。若 `SCENE_ROOT` 指向外部 NavDP 场景库，请将仓库中的 `data/benchmark_v1/scenes/benchmark_e2/e2_corridor.usda` 同步到对应的 `benchmark_e2/e2_corridor.usda`；manifest 会拒绝任何不匹配的副本。

在 Isaac Sim 环境中运行 Oracle 和三个健全性基线：

```bash
export SCENE_ROOT=/home/an/project/NavDP/assets/scenes
export DATA_ROOT=$PWD/research/ball_wm/data
export PYTHONPATH=research/ball_wm
PYTHON=/home/an/miniconda3/envs/ball_wm/bin/python

$PYTHON research/ball_wm/scripts/run_pointgoal_benchmark.py \
  --manifest research/ball_wm/configs/benchmark_v1/golden_v1_manifest.json \
  --split golden --policy oracle --scene-root "$SCENE_ROOT" \
  --data-root "$DATA_ROOT" --run-root research/ball_wm/runs/benchmark_golden_oracle_20260917 \
  --headless --record-video
```

将 `--policy oracle` 替换为 `stop`、`goal_greedy` 或 `random`，并为每个策略使用独立的 `--run-root`。每个 episode 都在独立 Isaac 进程中运行，结果目录包含 `metrics.json`、`environment.json`、`manifest.json`、NPZ 状态/动作、轨迹图和双目/第三人称视频；`summary.json` 汇总 Success、Collision、Timeout、SPL 和协议哈希。

如果要评估副轴 `0.30 rad/s` 速度包络，先运行限速标定矩阵：

```bash
$PYTHON research/ball_wm/scripts/probe_drive_dynamics.py \
  --out research/ball_wm/runs/drive_dynamics_auxvel030_extended_20260918 \
  --profile-out research/ball_wm/configs/benchmark_v1/drive_dynamics_auxvel030_v2.json \
  --auxiliary-velocity-limit-rad-s 0.30
```

这会生成包含限速控制器元数据的 `drive_dynamics_v2` profile。正式评测通过
`--auxiliary-velocity-limit-rad-s 0.30 --drive-profile .../drive_dynamics_auxvel030_v2.json`
显式启用；省略两个参数仍使用冻结的 V1 profile 和原始控制器。限速 profile 的原始
54 工况报告保存在对应的 `runs/drive_dynamics_*` 目录中，不能用未完成稳态的早期报告
替代最终 profile。

2026-09-17 的冻结结果记录在 `docs/benchmark_golden_v1_20260917.md`。Oracle 是特权上界诊断，不属于无地图公平赛道；它在 E0/E1/E2 上分别得到 collision、success、timeout，失败样本必须保留，不能从汇总中删除。

正式采集示例：

```bash
/home/an/miniconda3/envs/ball_wm/bin/python \
  research/ball_wm/scripts/collect_data_dubins.py \
  --scene /absolute/path/to/scene.usd \
  --num_episodes 100 --episode_len 300 \
  --save_dir /absolute/path/to/output
```

采集器默认 headless，深度默认落盘。`--no_save_depth` 只停止保存深度文件，仍会在
内存中使用深度生成可见性；`--save_preview` 生成五联预览；
`--audit_after_episode` 在发布每个 episode 后立即执行离线审计。高程教师默认开启：
`--elevation_teacher {auto,analytic,raycast}` 选择高程来源（默认 `auto`），
`--no_save_elevation` 关闭高程文件写入并将 metadata 的 `elevation_enabled` 设为
`false`；关闭时仍保留 RGB、深度和二维 BEV 标签。`--save_dir` 根目录
可以预先存在，但采集器绝不覆盖其中已有的 `epNNNN/` 或 `epNNNN.partial/`；遇到同名
episode 或残留 partial 目录时会拒绝继续。

每个场景的所有 episode 完成后，采集器还会在 `save_dir/trajectory_map.png` 写入一张
场景级俯视图：全局二维占据地图上叠加机器人 world-XY 轨迹；绿色圆点是起点、蓝色圆点
是终点、红色点表示 `minimum_obstacle_distance <= 0` 的视觉帧。多 episode 场景会用不同
颜色区分轨迹。地图方向为 world `+X` 向右、world `+Y` 向上。

## 一集视觉冒烟检查

使用非空、带有效 collision geometry 的场景运行：

```bash
/home/an/miniconda3/envs/ball_wm/bin/python research/ball_wm/scripts/collect_data_dubins.py \
  --scene /absolute/path/to/scene.usd \
  --num_episodes 1 --episode_len 100 \
  --save_preview --audit_after_episode \
  --save_dir /tmp/ball_wm_bev_smoke
```

使用安装了 Isaac Sim 4.5/Isaac Lab 的环境执行同一冒烟检查：

```bash
/home/an/miniconda3/envs/ball_wm/bin/python research/ball_wm/scripts/collect_data_dubins.py \
  --scene /home/an/project/NavDP/assets/scenes/cluttered_easy/easy_0/cluttered-0.usd \
  --num_episodes 1 --episode_len 100 --save_preview --audit_after_episode \
  --save_dir /tmp/ball_wm_2p5d_smoke
```

兼容性检查使用完全相同的场景和采集设置再运行一次，但关闭高程保存；两次运行的
`geometry_bev` 应逐帧一致，关闭运行不得出现任何 `height_*` 文件。为使两次运行
完全确定，均使用 `--sign_probe main_positive --sign_probe_steps 100`；sign probe
会禁用随机 waypoints，因此不会引入不同的随机导航轨迹：

```bash
rm -rf /tmp/ball_wm_2p5d_enabled /tmp/ball_wm_2p5d_disabled
/home/an/miniconda3/envs/ball_wm/bin/python research/ball_wm/scripts/collect_data_dubins.py \
  --scene /home/an/project/NavDP/assets/scenes/cluttered_easy/easy_0/cluttered-0.usd \
  --num_episodes 1 --episode_len 100 --sign_probe main_positive --sign_probe_steps 100 \
  --audit_after_episode \
  --save_dir /tmp/ball_wm_2p5d_enabled
/home/an/miniconda3/envs/ball_wm/bin/python research/ball_wm/scripts/collect_data_dubins.py \
  --scene /home/an/project/NavDP/assets/scenes/cluttered_easy/easy_0/cluttered-0.usd \
  --num_episodes 1 --episode_len 100 --sign_probe main_positive --sign_probe_steps 100 \
  --no_save_elevation --audit_after_episode \
  --save_dir /tmp/ball_wm_2p5d_disabled
test -z "$(find /tmp/ball_wm_2p5d_disabled -type f \( -name 'height_min_bev_*' -o -name 'height_max_bev_*' -o -name 'height_valid_bev_*' \))"
for f in /tmp/ball_wm_2p5d_enabled/ep0000/geometry_bev_*.png; do
  cmp "$f" \
    "/tmp/ball_wm_2p5d_disabled/ep0000/$(basename "$f")"
done
```

成功运行应产生 100 个 50 Hz 状态样本和 20 个 10 Hz 同步双目/BEV 样本，并打印
`Audit passed`。外部场景的全局 OMap 必须至少包含一个 occupied 单元，否则采集器会在
写 episode 前失败，避免静默发布伪造的全 free 障碍物标签。地面模式（省略
`--scene`）只用于兼容性检查；其中几何 BEV 应为 free，不能据此验收障碍物方向或
占据标签。

可单独复查数据集：

```bash
/home/an/miniconda3/envs/ball_wm/bin/python \
  research/ball_wm/scripts/audit_dataset.py /tmp/ball_wm_bev_smoke
```

### easy_0…easy_9 试采与正式采集流程

先检查磁盘空间及目标目录不存在，再按以下顺序执行；任一 episode 审计失败都应
立即停止，不得带着失败样本继续：

1. 以 `easy_0` 冒烟并确认 schema `1.2.0`、`elevation_enabled=true`、三份高程
   文件均为预期 dtype/有限值/非负有效跨度，且 RGB/深度/二维标签未改变。
2. 试采目录使用
   `research/ball_wm/data/trajectories_dubins_2p5d_pilot_20260828/`；以下命令对
   `easy_0` 至 `easy_9` 各采集 1 个 300-step episode，写入
   `scene_easy_N/ep0000/`：

   ```bash
   for n in $(seq 0 9); do
     /home/an/miniconda3/envs/ball_wm/bin/python research/ball_wm/scripts/collect_data_dubins.py \
       --scene "/home/an/project/NavDP/assets/scenes/cluttered_easy/easy_${n}/cluttered-${n}.usd" \
       --num_episodes 1 --episode_len 300 --audit_after_episode \
       --save_dir "research/ball_wm/data/trajectories_dubins_2p5d_pilot_20260828/scene_easy_${n}"
   done
   ```
   每个 `scene_easy_N/` 完成后可直接查看 `trajectory_map.png`，先确认起点、终点和
   轨迹是否落在预期的 free 区域，再继续正式采集。
3. 汇总试采的有效高程覆盖率、`height_min_bev`/`height_max_bev` 范围和审计结果，
   确认所有场景通过后再开始正式采集。
4. 正式目录必须另行使用
   `research/ball_wm/data/trajectories_dubins_2p5d_20260828/`；以下命令每场景采集
   5 个 300-step episodes（`ep0000`…`ep0004`），不得向试采目录追加或覆盖：

   ```bash
   for n in $(seq 0 9); do
     /home/an/miniconda3/envs/ball_wm/bin/python research/ball_wm/scripts/collect_data_dubins.py \
       --scene "/home/an/project/NavDP/assets/scenes/cluttered_easy/easy_${n}/cluttered-${n}.usd" \
       --num_episodes 5 --episode_len 300 --audit_after_episode \
       --save_dir "research/ball_wm/data/trajectories_dubins_2p5d_20260828/scene_easy_${n}"
   done
   ```

预览从左到右依次为左目 RGB、右目 RGB、完整几何 BEV、当前可见 mask、历史已观测
mask。目视确认障碍物相对方向与 RGB/场景一致、机器人自身不出现在几何 BEV 中，
并结合数组确认 `current_visible <= history_observed`。

对于在加入场景级轨迹图功能之前已经完成的 pilot 数据，可以用后处理脚本从保存的
`geometry_bev` 和 `states.npz` 重建“已观测局部 BEV”地图并生成同样的图：

```bash
/home/an/miniconda3/envs/ball_wm/bin/python \
  research/ball_wm/scripts/plot_trajectory_map.py \
  research/ball_wm/data/trajectories_dubins_2p5d_pilot_20260828
```

后处理图的标题会标注 `observed local BEV`；正式采集器直接使用场景全局占据地图，
因此优先以采集时生成的 `trajectory_map.png` 验收全局障碍物方向。

## 数据集清单与联合世界模型基线

正式数据集使用 `scene_easy_N/epNNNN/` 的分层目录。统计脚本会递归发现这些 episode，
同时兼容旧的扁平 `epNNNN/` 目录，并写出可复查的 JSON manifest：

```bash
/home/an/miniconda3/envs/ball_wm/bin/python \
  research/ball_wm/training/dataset_stats.py \
  --root research/ball_wm/data/trajectories_dubins_2p5d_20260828 \
  --output research/ball_wm/data/trajectories_dubins_2p5d_20260828/dataset_manifest.json
```

manifest 包含每场景/每 episode 的状态与双目帧数、碰撞帧和碰撞率、轨迹长度与位移、
轨迹图路径，以及高程有效帧数、有效单元覆盖率和 mask-first 高程范围。正式采集完成后，
本目录的 manifest 应报告 50 episodes、15,000 个状态样本和 3,000 个双目帧；它属于
数据产物，不应提交到代码仓库（`research/ball_wm/data/` 已被 gitignore）。

训练数据加载器同样递归发现 episode，因此可以直接把场景集合根目录作为单个
`--data_root`。下面命令先跑一轮联合 stereo/proprio→BEV 基线，用于确认数据管线和
模型前向/反向均正常；`--config` 接受仓库现有 YAML 配置：

```bash
/home/an/miniconda3/envs/ball_wm/bin/python \
  research/ball_wm/training/train_joint.py \
  --data_root research/ball_wm/data/trajectories_dubins_2p5d_20260828 \
  --config research/ball_wm/configs/train_joint.yaml \
  --epochs 1 --batch_size 8 --num_workers 0 \
  --out_dir /tmp/ball_wm_joint_baseline
```

默认按场景划分 train/validation；需要严格保留未见场景测试集时加上
`--unseen_scene_test`（会从最后的场景组留出 test），并在日志中核对三组场景没有交集。

训练后可对未见场景生成 rollout 预测与真值对照图。脚本同时报告 BEV 每类 IoU；不要只看
总体 accuracy，因为 free 栅格占多数时“全 free”也会得到很高的 accuracy：

```bash
PYTHONPATH=research/ball_wm \
/home/an/miniconda3/envs/ball_wm/bin/python \
  research/ball_wm/training/visualize_predictions.py \
  --checkpoint research/ball_wm/runs/joint_wm_balanced_20260830/joint_wm_epoch06.ckpt \
  --data_root research/ball_wm/data/trajectories_dubins_2p5d_20260828/scene_easy_8 \
  --output_dir research/ball_wm/runs/joint_wm_balanced_20260830/predictions_easy8 \
  --num_samples 10
```

`train_joint.yaml` 中的 `bev_class_weights: [1.0, 8.0, 1.0]` 用于抑制 occupied 类塌缩；
比较不同权重时应固定场景划分，并同时记录 occupied IoU、mean IoU 和预测 occupied 比例。

2026-09-07 BEV 消融入口为 `python -m training.run_bev_ablation --out_dir <新目录> --phase all`。
它依次执行旧模型 20 秒闭环（5/10/20 秒相同轨迹前缀）、A/B 各 10 轮从零训练、固定测试场景的
离线与闭环评估。`--phase budget/train/eval` 可分别运行阶段；已有同名日志/训练目录会拒绝覆盖，
不会将失败子进程当作成功结果。旧模型预算实验使用历史 41-context manifest，因此需保留原数据与资产。

`ablation_scene_split.json` 固定本轮 train/val/test；`ablation_a_pose.yaml` 通过
`spatial_supervision_enabled: false` 关闭全部空间损失（当前/未来占据、current/history visibility），
冻结并跳过空间头；休眠头仅保留 checkpoint 兼容性。`ablation_b_spatial.yaml` 启用同一空间监督包。
两组初始参数由相同 seed 生成并用 SHA256 核对，shuffle 使用独立固定 generator，规划均关闭 BEV safety。
`run_manifest.json` 和 checkpoint 保存划分、初始化来源/哈希与训练参数。A 的休眠 BEV 头离线输出不应
解释为训练后的空间预测。两组按最后 epoch 比较，不用测试结果选择 checkpoint；仅一组配对 seed，
只能作为初步消融。历史研究已涉及这三个测试场景，不能据此称为研究过程完全未见泛化。

预算文件中的 `reference_spl` 以录制轨迹段为参考，不是最短可行路径意义上的标准 SPL。
每个 scene 的 `*_planner_diagnostics.png` 和原始 JSON 保留实际轨迹、预测轨迹与目标距离变化。

无地图局部 PointGoal 训练使用 `train_joint_pose_local.yaml`：在共享 future latent 上同时
监督五步 BEV 与相对 pose `[Δx, Δy, sinΔyaw, cosΔyaw]`，并在第 1/3/5 步计算 pose loss。
当前 pose 来自轮速计；pose head 只预测候选动作后的相对运动结果。旧 checkpoint 缺少
pose 配置时仍可严格加载，但不能用于默认的 `pose_bev` CEM。候选训练命令如下：

```bash
PYTHONPATH=research/ball_wm \
/home/an/miniconda3/envs/ball_wm/bin/python \
  research/ball_wm/training/train_joint.py \
  --data_root research/ball_wm/data/trajectories_dubins_2p5d_20260828 \
  --config research/ball_wm/configs/train_joint_pose_local.yaml \
  --epochs 10 --batch_size 8 --num_workers 4 --unseen_scene_test \
  --out_dir research/ball_wm/runs/joint_wm_spatial_20260831
```

同一 context 多候选动作的排序训练使用独立的 `candidate_rollout_v1` 数据目录；正式
episode 不会被复制或覆盖。先按场景从冻结 manifest 采集（easy_8 只用于最终测试）：

```bash
PYTHONPATH=research/ball_wm \
/home/an/miniconda3/envs/ball_wm/bin/python \
research/ball_wm/scripts/collect_candidate_rollouts.py \
  --formal_data_root research/ball_wm/data/trajectories_dubins_2p5d_20260828 \
  --candidate_root research/ball_wm/data/candidate_rollouts_v1_20260901 \
  --context_manifest research/ball_wm/runs/navigation_eval_strict_cross_scene_20260831/context_manifest.json \
  --scene scene_easy_0 --candidate_count 16 --horizon 5 --device cuda
```

完成训练/验证场景采集后，从 pose checkpoint 初始化候选分支：

```bash
PYTHONPATH=research/ball_wm \
/home/an/miniconda3/envs/ball_wm/bin/python -m training.train_joint \
  --data_root research/ball_wm/data/trajectories_dubins_2p5d_20260828 \
  --config research/ball_wm/configs/train_joint_pose_candidate.yaml \
  --candidate_root research/ball_wm/data/candidate_rollouts_v1_20260901 \
  --candidate_formal_data_root research/ball_wm/data/trajectories_dubins_2p5d_20260828 \
  --init_checkpoint research/ball_wm/runs/joint_wm_pose_local_20260831/joint_wm_epoch09.ckpt \
  --out_dir research/ball_wm/runs/joint_wm_candidate_20260901
```

候选排序必须先在 train/val 与隔离 easy_8 上报告 pairwise accuracy、Spearman、top-1
和预测终点标准差；只有 Isaac CEM 开环校准通过 `rho > 0.3`、`top1 > 0.5`、CEM 胜
random > 0.5 的门槛，才进入闭环导航评估：

```bash
PYTHONPATH=research/ball_wm \
/home/an/miniconda3/envs/ball_wm/bin/python -m training.evaluate_candidate_ranking \
  --checkpoint research/ball_wm/runs/joint_wm_candidate_20260901/joint_wm_epoch00.ckpt \
  --candidate_root research/ball_wm/data/candidate_rollouts_v1_20260901/scene_easy_8 \
  --formal_data_root research/ball_wm/data/trajectories_dubins_2p5d_20260828 \
  --split all --expected_candidates 16
```

固定 easy_8 的 10 个可视化样本上，空间候选验证集最优 checkpoint（epoch 0）的
occupied IoU 均值为 `0.0721`（预测 occupied 比例 `0.3852`），低于 balanced MLP
baseline 的 `0.1080`（比例 `0.1610`）；PointGoal latent-cost smoke 的
planned/random 均值为 `13.23/12.58`，也没有优于随机动作。
因此当前结果不支持替换 baseline 或宣称导航提升；该实现作为可复现实验分支保留，后续
应优先分析 patch-token 空间解码器或更强的占据监督。

`JointJEPA.rollout()` 的动作语义与数据契约一致：`action_sequence[:, :, 0]` 用于生成
第一个未来帧，后续候选动作依次用于后续帧。局部 PointGoal 评估在每个 MPC 周期以当前
轮速计 `(x, y, yaw)` 和任务目标计算 base-frame goal；不会读取目标未来 RGB、未来状态或
全局障碍地图。

训练完成后先报告未见场景的未来世界模型准确率。该脚本只使用录制的 future action 和真值
标签，输出 H=1/3/5 的 feature、occupied BEV IoU/F1、pose、ADE/FDE：

```bash
PYTHONPATH=research/ball_wm \
/home/an/miniconda3/envs/ball_wm/bin/python \
research/ball_wm/training/evaluate_world_model.py \
  --checkpoint research/ball_wm/runs/joint_wm_pose_local/joint_wm_epoch09.ckpt \
  --data_root research/ball_wm/data/trajectories_dubins_2p5d_20260828 \
  --out research/ball_wm/runs/joint_wm_pose_local/future_prediction.json --device cuda
```

默认按 scene-disjoint 方式留出约 15% 的未见场景；需要降低单场景偶然性时，可用
`--test_scene_fraction 0.3` 留出约 30% 的未见场景，并在 JSON 的 `held_out_scenes`、
`sample_count` 和 `test_scene_fraction` 中核对实际测试范围。JSON 还包含 occupied
目标/预测比例、precision、recall、混淆矩阵，以及 `teacher_bev_per_horizon`；后者绕过
action rollout，用于区分 BEV decoder/目标误差和动力学预测误差。

闭环导航结果必须用 Isaac Sim 的真实执行指标判断，而不是把 latent cost 当作成功率。
`evaluate_navigation.py` 从固定 episode context 恢复球形机器人，CEM 每轮只执行第一个
10 维 action block（5 个 50 Hz 控制对），后续 RGB/proprio 全部来自仿真。成功条件为
距离记录的 world-frame 目标不超过 `0.2 m`、无 analytic-BEV-clearance 碰撞、且未达到
`max_mpc_steps`；每个 JSON 还区分 `collision`、`timeout` 和 `runtime_error`。只要出现
runtime error，脚本会写出 JSON 后以非零状态退出，绝不会退回离线 latent 评估。

规划器的动作边界严格按五组 `[main, auxiliary]` 控制对展开：主驱动为 `[-2, 2]`，
辅助驱动为 `[-0.2809658, 0.2809658]`。每轮 CEM 还固定保留零动作、持续前进/后退、
左右转向、前进转向和变向等 8 个原语锚点，避免高方差随机采样漏掉训练数据中的结构化动作。

单个 easy_8 smoke（只执行 3 个 MPC block）如下：

```bash
PYTHONPATH=research/ball_wm \
/home/an/miniconda3/envs/ball_wm/bin/python \
research/ball_wm/training/evaluate_navigation.py \
  --checkpoint research/ball_wm/runs/joint_wm_pose_local/joint_wm_epoch09.ckpt \
  --data_root research/ball_wm/data/trajectories_dubins_2p5d_20260828 \
  --scene scene_easy_8 --num_eval 1 --goal_offset 5 --goal_tolerance 0.2 \
  --max_mpc_steps 3 --horizon 5 --samples 8 --elites 2 --iterations 1 \
  --planner_mode pose_bev --safety_weight 0.1 --safety_threshold 0.5 \
  --policy cem --seed 3072 --device cuda \
  --out research/ball_wm/runs/navigation_smoke_pose_local/pose_bev.json
```

正式比较由同一个 pose checkpoint 派生三组 policy：`pose_only_cem`（相同 pose model、
BEV safety weight=0）、`pose_bev_cem`（开启局部 BEV safety）和 `random`。比较
`success_rate`、`collision_rate`、`timeout_rate`、`final_distance_mean_m`、`path_length_mean_m`
和 `spl`，并报告 raw episode bootstrap 95% CI；未来预测、模型内 cost 与 BEV IoU 仅是
准入/诊断指标，不能称作导航成功率。

当需要解释 CEM 的闭环失败时，在直接评估命令中增加 `--save_planner_diagnostics`。该开关会将每个
MPC step 的所选 action、latent-goal/BEV-safety/total cost、elite cost 分布与真实执行进度写入
`planner_trace`，并在同一输出目录生成 `<out-stem>_planner_diagnostics.png`；它不改变默认评估或
严格跨场景 suite 的主指标。

第一轮 easy_8 结果只含 5 个短距离目标，不能作为主结论。严格跨场景协议改为 `goal_offset=20`、
初始目标距离至少 `0.5 m`、记录的初始 collision 为 false 且 clearance 不低于显式阈值
`--min_initial_clearance`（正式安全评估使用 `0.2 m`；阈值为 `0` 时保持大于零）；每个 raw episode
最多一个 context。正式数据中因此有 42/50 条有效 episode。`evaluate_navigation_suite.py` 先写入
唯一的 `context_manifest.json`，再对每个 USD scene 启动独立 Isaac 进程；三组 policy
只能读取这一个 manifest，因而不会重新抽样或跨场景复用运行时。

```bash
PYTHONPATH=research/ball_wm \
/home/an/miniconda3/envs/ball_wm/bin/python \
research/ball_wm/training/evaluate_navigation_suite.py \
  --pose_checkpoint research/ball_wm/runs/joint_wm_pose_local/joint_wm_epoch09.ckpt \
  --data_root research/ball_wm/data/trajectories_dubins_2p5d_20260828 \
  --out_dir research/ball_wm/runs/navigation_eval_strict_cross_scene_20260831 \
  --goal_offset 20 --min_initial_goal_distance 0.5 --min_initial_clearance 0.2 --goal_tolerance 0.2 \
  --max_mpc_steps 50 --horizon 5 --samples 64 --elites 8 --iterations 3 \
  --safety_weight 0.1 --safety_threshold 0.5 --seed 3072 --device cuda
```

在运行严格闭环前，可用它生成的 `context_manifest.json` 做一个 scene 内的五候选开环校准。
该结果包含 Spearman 排名相关、CEM 对 zero/random 的真实距离胜率；若 CEM 持续差于基线，
不得把闭环结果解释为有效规划：

```bash
PYTHONPATH=research/ball_wm \
/home/an/miniconda3/envs/ball_wm/bin/python \
research/ball_wm/training/evaluate_planner_calibration.py \
  --checkpoint research/ball_wm/runs/joint_wm_pose_local/joint_wm_epoch09.ckpt \
  --data_root research/ball_wm/data/trajectories_dubins_2p5d_20260828 \
  --context_manifest research/ball_wm/runs/navigation_eval_strict_pose_local/context_manifest.json \
  --scene scene_easy_8 --out research/ball_wm/runs/joint_wm_pose_local/calibration_easy8.json \
  --samples 64 --elites 8 --iterations 3 --safety_weight 0.1 --device cuda
```

输出目录包含 manifest、每个 policy/scene 的原始闭环 JSON、三个 policy 汇总和
`comparison.json`。任一子进程的 runtime error 或缺失 JSON 都使 suite 非零退出；不允许将这类
结果当作成功率。

## Episode 输出格式

每集先写入 `epNNNN.partial/`，完整落盘后原子发布为 `epNNNN/`：

```text
scene_easy_N/
  trajectory_map.png                    # 场景级全局地图+机器人轨迹（采集结束后生成）
  ep0000/
  left_000000.png                       # uint8 RGB，左目无损图像
  right_000000.png                      # uint8 RGB，右目无损图像
  left_depth_000000.npy                 # float32[H,W]，左目 image-plane distance（可选）
  right_depth_000000.npy                # float32[H,W]，右目 image-plane distance（可选）
  geometry_bev_000000.png               # uint8[80,80]，0/1/2 完整几何真值
  current_visible_000000.png            # uint8[80,80]，0/1 当前双目可见区域
  history_observed_000000.png           # uint8[80,80]，0/1 历史已观测区域
  height_min_bev_000000.npy              # float32[80,80]，有效格的相对最低高度（m）
  height_max_bev_000000.npy              # float32[80,80]，有效格的相对最高高度（m）
  height_valid_bev_000000.png            # uint8[80,80]，0/1 高程有效 mask
  preview_000000.png                    # 五联目视检查图（--save_preview）
  states.npz
  metadata.json
```

文件编号是从 0 连续递增的 `stereo_frame_id`。所有 PNG/NPY 视觉资产都有成对、同编号
要求。

### `states.npz`

`Ns` 是 50 Hz 状态数，`Nv` 是 10 Hz 视觉数：

- `physics_step[int64,Ns]`：已完成的 physics step；
  `sim_time[float64,Ns]`、`state_timestamp[float64,Ns]`：同一步仿真/状态时间。
- `pos[float32,Ns,3]`、`quat[float32,Ns,4]`：world 系 base 位置和 wxyz 四元数；
  `lin_vel[float32,Ns,3]`、`ang_vel[float32,Ns,3]`：world 系 base 线/角速度。
- `joint_pos[float32,Ns,3]`、`joint_vel[float32,Ns,3]`：三个关节位置/速度。
- `commanded_action[float32,Ns,2]`、`applied_action[float32,Ns,2]`：请求动作和限幅后
  实际发送动作；`action_command_timestamp[float64,Ns]`、
  `action_applied_timestamp[float64,Ns]` 是对应零阶保持事件时间。
- `stereo_frame_id[int64,Nv]`、`stereo_physics_step[int64,Nv]`：视觉编号及对应完成步；
  `left_timestamp[float64,Nv]`、`right_timestamp[float64,Nv]`：左右目采样时间。
- `cam_left_pose[float32,Nv,4,4]`、`cam_right_pose[float32,Nv,4,4]`：相机 USD
  local-to-world 齐次变换；`left_filename[str,Nv]`、`right_filename[str,Nv]`：RGB 文件名。
- `collision[bool,Nv]`：二维局部 BEV 间隙是否小于等于零；
  `minimum_obstacle_distance[float32,Nv]`：最近 occupied 单元中心距离减球半径，无
  occupied 单元时为正无穷。

### `metadata.json`

- 契约/来源：`schema_version`、`contract_path`、`scene`、`episode_index`、
  `random_seed`、`waypoints_world_xy`、`sign_probe`、`sign_probe_steps`。
- 时序/图像：`rates_hz`、`physics_dt_s`、`rendering_dt_s`、`image_width_px`、
  `image_height_px`、`visual_history_frames`、`save_depth`。
- 动作：`action_names`、`action_minimum`、`action_maximum`、`action_semantics`。
- BEV/碰撞：`bev_extent_m`、`bev_resolution_m`、`bev_shape`、`bev_class_codes`、
  `bev_frame`、`ball_radius_m`、`collision_definition`、
  `minimum_obstacle_distance_definition`。
- 标定：`left_intrinsics`、`right_intrinsics`、`intrinsics_matrix_convention`、
  `extrinsic_matrix_convention`、`left_to_right_extrinsic`、`base_to_left_extrinsic`、
  `base_to_right_extrinsic`、`left_optical_to_base_extrinsic`、
  `right_optical_to_base_extrinsic`、`camera_pose_storage`、`depth_frame`、
  `depth_invalid_storage_value`。
- 高程：`elevation_enabled`、`elevation_teacher`、`elevation_frame`、
  `elevation_pose_policy`、`elevation_shape`、`elevation_unit`、`elevation_height_reference`、
  `elevation_dtype`、`elevation_invalid_value`、`elevation_file_naming`。
- 文件约定：`file_naming`，列出左右 RGB、左右深度、三种 BEV/mask、高程三件套和
  preview 模板。高程数组均为 80×80；`height_min_bev`/`height_max_bev` 是米制
  `float32`，无效格必须为 `0.0`，`height_valid_bev` 是二值 `uint8` mask。

### 高程教师与 mask-first 阈值规则

高程标签采用 base 的 XY+yaw 坐标并以世界重力方向记录相对高度：
`z_world - z_base_world`。先用 `height_valid_bev == 1` 筛选有效格，再在这些格上
进行任何高度阈值、范围统计或可视化；`height_valid_bev == 0` 的
`height_min_bev`/`height_max_bev` 数值虽固定为 `0.0`，不得当作真实地面高度参与
阈值。采集器写盘前的 producer-side `validate_elevation_coverage` 要求每个二维
`free` 单元都有有效高程覆盖；发布后的 `audit_dataset.py` 则检查高程文件的存在性、
shape/dtype、有限值、二值 mask、无效格为 `0.0`、`height_min_bev <= height_max_bev`
和配对文件，不会重新推断或强制 free-cell 覆盖。

每个 10 Hz 视觉帧新增三份文件约占
`2 × 80 × 80 × 4 = 51.2 KiB`（NPY 数据本体，另加 PNG/header 开销）；300-step
episode 含 60 帧，预计高程新增约 3–4 MiB/episode。正式的 10 个场景 × 每场景 5
episodes 预计新增约 150–200 MiB，采集前请用 `df -h` 确认余量。

## 动作方向探针

动作正负只定义为 Isaac Sim 关节坐标增减；导航效果必须由四个独立探针实测：

```bash
/home/an/miniconda3/envs/ball_wm/bin/python research/ball_wm/scripts/collect_data_dubins.py \
  --scene /absolute/path/to/scene.usd --save_dir /tmp/probe_main_positive \
  --sign_probe main_positive --sign_probe_steps 100
```

将 `main_positive` 依次替换为 `main_negative`、`aux_positive`、`aux_negative`，分别使用
新的输出目录，并检查打印的平移与 yaw。四个结果全部检查前，不应在 Phase 0 清单中
确认动作导航方向。

## 纯 Python 验证

```bash
/home/an/miniconda3/envs/ball_wm/bin/python -m pytest -q research/ball_wm/tests
/home/an/miniconda3/envs/ball_wm/bin/python -m py_compile \
  research/ball_wm/scripts/collection_schema.py \
  research/ball_wm/scripts/bev_ground_truth.py \
  research/ball_wm/scripts/audit_dataset.py \
  research/ball_wm/scripts/collect_data_dubins.py
git diff --check
```

这些测试和审计不启动 Isaac Sim，覆盖固定 50/50/50/10 Hz 调度、二维动作、时间语义、
BEV 坐标、历史 mask、文件配对、dtype 和类别取值。
