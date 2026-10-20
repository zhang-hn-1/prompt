# Dynamic Oracle V2 `hard_0_e1_0002` 第一仿真门槛

## 结论

**BLOCKED**。冻结的 `hard_0_e1_0002` 单任务命令没有发布完整审计包；真实
Isaac 运行在 episode 执行前耗尽动态规划器的冻结 `60 s` 墙钟上限。因为没有
执行动作，也没有生成 `metrics.json` 或 `states.npz`，成功、碰撞、35 秒执行
预算、最大横向误差及参考速率均无法认证。按 Task 8 设计，Task 9 停止，不修改
manifest、门槛或实现。

## 冻结输入与环境

- Worktree：`/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark`
- Branch：`codex/e1-detour-benchmark`
- Git commit：`a656d8ca7ff3bddc168df8fdd120e47bfc9edf32`
- Python：`3.10.20`
- Isaac Sim：`4.5`
- GPU：`NVIDIA GeForce RTX 3060`，`12288 MiB`
- NVIDIA driver：`580.178.04`
- Manifest SHA-256：`f3a23e824502990fddf862d5a5918424698b80b81fa22c01cc06b924de8f5e76`
- Scene SHA-256：`7125024056f83009e74f1b5c959d3cefdc7190157ec67b9d43818c613fced9f0`
- Occupancy SHA-256：`547dd73c52b45f9cc1aa8dca561c779349391f6ecef06ef59026921284b467da`

实际 scene 与 occupancy 哈希分别和冻结 manifest 中的值完全一致。episode 为
`dev/E1`，seed `20260913`，起点 `(-9.85, 4.05, 0.45)`，终点
`(-7.75, -4.15, 0.45)`，冻结 A* reference length 为
`12.791168824543142 m`。

## 实际命令

以下命令未增加参数，也未修改 manifest、阈值或实现：

```bash
/home/an/miniconda3/envs/ball_wm/bin/python research/ball_wm/scripts/run_pointgoal_benchmark.py \
  --manifest research/ball_wm/configs/benchmark_v1/hard_0_dev_manifest.json \
  --split dev --policy oracle_v2 \
  --scene-root /home/an/project/NavDP/assets/scenes \
  --data-root research/ball_wm/data \
  --run-root research/ball_wm/runs/e1_0002_dynamic_oracle_v2_gate \
  --headless --record-video --episode-id hard_0_e1_0002
```

沙箱内首次启动退出码为 `139`：Vulkan 报告 driver `0`、没有 GPU device，
PhysX 报告没有 CUDA-capable device。该未进入有效 Isaac 环境的尝试只写出冻结
manifest，保存在
`episode_hard_0_e1_0002.sandbox_gpu_unavailable.partial/`。

按预案在允许 GPU 的环境中用完全相同命令重跑。Isaac 成功识别 RTX 3060 与
driver `580.178.04`，加载场景并初始化 PhysX CUDA device 0；进程最终退出码为
`1`。Isaac 日志中 reset/render 初始化在相对时间 `[19.365 s]` 完成，关闭在
`[79.892 s]` 开始，间隔约 `60.527 s`，与冻结动态规划墙钟上限 `60.0 s`
一致。随后只生成 `task_card.png` 即关闭。结合 runner 的固定流程，这一文件边界
只会发生在 `run_episode` 收到非 success 动态计划、随后审计渲染再次请求缓存计划
而拒绝不可用 reference 的路径；因此失败状态为动态规划 `timeout`，episode 未进入
动作执行循环。

Isaac 日志：
`/home/an/miniconda3/envs/ball_wm/lib/python3.10/site-packages/omni/logs/Kit/Isaac-Sim Python/4.5/kit_20260914_124151.log`。

## 产物审计

Run root：
`research/ball_wm/runs/e1_0002_dynamic_oracle_v2_gate/`。

真实 GPU 重跑保留在
`episode_hard_0_e1_0002.partial/`。它只包含：

- `manifest.json`，`8297` bytes，SHA-256 与冻结 manifest 相同；
- `task_card.png`，`45519` bytes，合法的 `1200 x 900` RGB PNG。

以下必需产物全部缺失：`environment.json`、`third_person.mp4`、
`stereo_view.mp4`、`trajectory_overlay.png`、`actions.npz`、`states.npz`、
`metrics.json`、`human_review.json`。最终
`episode_hard_0_e1_0002/` 目录和 `summary.json` 也不存在。审计 writer 因此没有
发布不完整结果，符合原子发布契约。

## 门槛判定

| 门槛 | 冻结要求 | 实际证据 | 判定 |
|---|---:|---|---|
| 规划状态 | `success` | `timeout`，冻结 `60 s` 上限 | FAIL |
| Episode success | 是 | episode 未执行，无 `metrics.json` | 未通过 |
| Collision | `0` | 无执行数组，不能认证 | 未通过 |
| 执行时间 | `<= 35 s` | 无动作，不能认证 | 未通过 |
| 最大横向误差 | `< 0.2 m` | 无 `states.npz`，不能认证 | 未通过 |
| 参考主轴速率 | `<= 0.5 rad/s^2` | 无动态参考数组，不能重算 | 未通过 |
| 参考副轴速率 | `<= 30 deg/s` | 无动态参考数组，不能重算 | 未通过 |

因为 `metrics.json` 与 `states.npz` 未生成，不存在 certification 数字可与 NPZ
数组交叉核验。这里没有把缺失值解释为零或通过。

## 首次偏差审计

首次偏差时刻：**不适用**。规划在动作执行前失败，动作数为零，因而不存在可对齐
的 reference/actual main、auxiliary、speed、yaw 或 clearance 样本。相应值均为
**N/A（未生成）**，不能从 task card 或规划模型推测替代真实执行观测。

## 后续边界

Task 9 不运行。本任务没有修改冻结 benchmark、`0.2 m` 横向误差门槛、
`0.5 rad/s^2` 主轴速率、`30 deg/s` 副轴速率、`35 s` episode 预算、
`60 s` 规划上限或任何实现代码。下一步若继续，应作为新的规划性能/可行性诊断
任务处理，而不能用假 runtime 或放宽门槛替代本次失败证据。
