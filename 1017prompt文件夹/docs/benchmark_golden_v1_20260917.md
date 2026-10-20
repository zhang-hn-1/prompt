# Ball PointGoal Benchmark V1 golden 评测

本轮冻结 manifest 为 [`golden_v1_manifest.json`](../research/ball_wm/configs/benchmark_v1/golden_v1_manifest.json)，SHA256 为 `722db8e928318205ad513f156b6302faeccf240107767f1831635d05ea947a36`。它包含三项人工审核任务：

| episode | difficulty | 目的 | 起终点距离 | Oracle 路径 |
| --- | --- | --- | ---: | ---: |
| `golden_e0_0001` | E0 | hard_0 开放直达 | 8.839 m | 9.567 m |
| `golden_e1_0001` | E1 | 单障碍绕行 | 6.600 m | 8.837 m |
| `golden_e2_0001` | E2 | 1.2 m 窄通道 | 8.900 m | 8.900 m |

评测协议固定为 `ball_pointgoal_v1_1_35s`：35 s episode budget、10 Hz 重规划、0.20 m goal tolerance、`ball_action_v2_aux30` 动作契约、perfect localization。每项任务都使用 fresh Isaac 进程、headless GPU、视频记录和独立审计 bundle。运行命令见 [`research/ball_wm/README.md`](../research/ball_wm/README.md#pointgoal-v1-golden-benchmark)。

## 结果

| policy | E0 | E1 | E2 | Success | Collision | Timeout | mean SPL |
| --- | --- | --- | ---: | ---: | ---: | ---: | ---: |
| `oracle` | collision | success | timeout | 1/3 | 1/3 | 1/3 | 0.28595 |
| `goal_greedy` | success | collision | timeout | 1/3 | 1/3 | 1/3 | 0.33333 |
| `stop` | timeout | timeout | timeout | 0/3 | 0/3 | 3/3 | 0.00000 |
| `random` | timeout | timeout | timeout | 0/3 | 0/3 | 3/3 | 0.00000 |

Oracle 的逐项证据保存在 [`benchmark_golden_oracle_20260917`](../research/ball_wm/runs/benchmark_golden_oracle_20260917/)，健全性基线分别保存在 [`benchmark_golden_goal_greedy_20260917`](../research/ball_wm/runs/benchmark_golden_goal_greedy_20260917/)、[`benchmark_golden_stop_20260917`](../research/ball_wm/runs/benchmark_golden_stop_20260917/) 和 [`benchmark_golden_random_20260917`](../research/ball_wm/runs/benchmark_golden_random_20260917/)。

Oracle 的细节是：E0 在 `minimum_clearance_m=-0.0394 m` 时发生栅格碰撞；E1 成功，SPL `0.85784`、最小 clearance `0.0472 m`；E2 无栅格碰撞且最小 clearance `0.58995 m`，但在窄通道入口附近停滞并达到 35 s 超时。这个 E2 结果保留为真实的执行上界证据，不能标记为成功，也不能用放宽时间或删除任务来修饰。

## 复现与验证

CPU 合同验证：

```bash
PYTHONPATH=research/ball_wm /home/an/miniconda3/envs/ball_wm/bin/python -m pytest -q \
  research/ball_wm/tests/test_benchmark_manifest.py \
  research/ball_wm/tests/test_benchmark_task_generation.py \
  research/ball_wm/tests/test_benchmark_task_cards.py \
  research/ball_wm/tests/test_e1_detour_scene.py \
  research/ball_wm/tests/test_hard_0_benchmark_scene.py
```

本轮结果为 `50 passed`。E1 场景 SHA256 为 `d6236dea995f04e3a81fb90dcde4bde6b63841c283cc5b2470aee8cda34c8634`，E2 场景 SHA256 为 `3ef266856cc35215f190e8de85cc002b4835cea6a3ec4b7fea4a91e424db15b4`，E2 occupancy SHA256 为 `113bdb0f4d6ee86720718065950785b4c2b9d9abb1cc81a6aa34038c4181a257`，corridor mask SHA256 为 `ffc8a9bedf454cf282b5e1a146584a4ad8bd0922f0bbdafd9f1eeaaf804cad4e`。

这轮完成了可复现的 Oracle/健全性 baseline 和 E0/E1/E2 评测入口。Oracle 尚未满足“所有难度全成功、零碰撞”的更高认证门槛；E0 的 hard_0 执行碰撞和 E2 的窄通道停滞应作为后续 tracker/动力学工作的问题输入。
