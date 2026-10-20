# 碰撞判定修复与真实仿真回归（2026-09-29）

本轮修复了“地图认为发生碰撞，但物理引擎没有检测到障碍接触”的误判，同时补上“已经接触障碍，但地图净空仍大于零”的漏判。修改已进入正式运行入口。没有调整 PID、轨迹、规划安全裕度、机器人半径或速度限制；评测时限仍为 35 秒。

## 根因和实际改动

旧版在 `IsaacBenchmarkRuntime._capture()` 中使用局部栅格净空 `clearance <= 0` 直接给出 collision；物理接触仅作诊断，不参与失败判定。局部地图分辨率、旋转裁剪及栅格距离与实际碰撞几何不同，所以两者并不等价。此次不是把碰撞阈值放宽，而是把正式碰撞判定的依据明确为物理引擎的障碍接触。

| 项目 | 历史模式 | 新默认模式 |
|---|---|---|
| collision mode | `map_clearance_v1` | `physical_contact_v2` |
| evaluation protocol | `ball_pointgoal_v1_1_35s` | `ball_pointgoal_v1_2_contact_35s` |
| 碰撞依据 | 地图净空 ≤ 0 | 过滤后的障碍接触力范数 > 1 N |
| 检查频率 | 每次 10 Hz capture | 每个 50 Hz 物理步，并在 capture 检查 |
| 短暂接触 | 两次 capture 之间可能漏掉 | 块内锁存，到该 0.1 秒动作块结束报告 |
| 地图净空 | 直接结束 episode | 保留风险值及非正样本计数，不直接结束 episode |
| 接触数据不可用 | 诊断可用性标记 | runtime_error，不回退为安全或地图碰撞 |

地面仍从接触过滤器中排除；保留原来的机器人刚体集合、障碍集合和 >1 N 阈值。这是可复现的操作定义，不声称检测到所有小于阈值的轻微触碰，也没有通过扩大/缩小机器人来改变结果。每个动作块仍完成原有五个物理步，碰撞终止报告延迟不超过一个控制块。

新模式下 `states.npz/physical_contact` 表示“从上次 capture 后，本动作块内曾检测到接触”，与同帧 collision 一致；旧模式该字段仍是 capture 瞬时接触。不能直接横比两种模式的接触样本率。`collision_audit.json` 保存逐步的时间、步号、数据可用性和接触布尔值；同一时刻的 physics/capture 记录可能重复，不能把记录数当独立碰撞次数。

CLI 和真实 `IsaacRuntimeConfig` 默认使用新模式；显式 `--collision-mode map_clearance_v1` 用于历史复现。子进程传递该选项，环境、metrics、审计和 summary 记录版本，summary 拒绝混合新旧协议。成功和异常路径均在原生关闭前保存碰撞审计，包括首次 reset 失败。旧结果不修改。

## 五条实际 Isaac 仿真

使用已冻结的 roll PID（Kp=0.4、Ki=0.1、Kd=0.7）、原逆映射、副轴 0.30 rad/s 包络及原参考。course 0004 仅复用此前已经拒绝的 V6 控制器作为已知漏检案例，不重新接受该控制器。

| 案例与模式 | 本轮终止 | 时间 | 最终目标距离 | 关键证据 |
|---|---|---:|---:|---|
| PID hard_0 E1 0002，新模式 | timeout | 35.0 s | 6.4096 m | 38 个地图非正净空状态，但全程无 >1 N 障碍接触 |
| course hard_0 E1 0004，新模式 | collision | 13.6 s | 9.3443 m | 13.56 s / 第 678 物理步首次接触；地图仍未判碰撞 |
| PID golden E0，新模式 | success | 20.0 s | 0.19955 m | 无接触，完整动作与状态和历史成功结果一致 |
| PID golden E1，新模式 | success | 33.1 s | 0.18717 m | 无接触，完整动作与状态和历史成功结果一致 |
| PID hard_0 E1 0002，显式旧模式 | collision | 15.7 s | 8.23799 m | 原地图失败精确复现，仍无 capture 物理接触 |

第一条新运行共 1750 个物理步及 351 次 capture，2101 条审计记录均可用且无接触，最小地图净空 -0.10 m。不能再将此例称为物理碰撞失败；它仍然没有到达目标，应归为超时。

第二条新运行共 680 个物理步，在第 678 步首次检测到障碍接触，在第 680 步 capture 终止；全过程地图最小净空仍为 +0.01231 m。历史 V6/V7 已定位此接触为球壳 link1 与 Sphere_67 障碍，新规则无需依赖地图判碰撞即可捕捉它。

没有把两条 golden 成功汇报为新的完整 E0/E1/E2 成功率：本轮没有重跑 golden E2 或完整 hard_0，更没有将新旧判定分数拼成一个总分。历史拒绝结论保持不变。

## 控制不变核验

所有五条运行的请求/执行动作、物理状态及相对时钟，与对应历史运行在共同前缀中完全一致；预期变化只有 collision 字段。0004 和旧协议复现的原始绝对时钟偏移一物理步（0.02 s），相对 episode 时钟和物理状态一致，不作为行为差异。两条 golden 全序列一致。

实际仿真使用的 20 个源码/控制器文件在运行前后哈希一致，保留运行快照。`verification.json` 的断言核验通过：五次真实运行返回码均为 0、审计无不可用样本、版本正确、新模式 collision 等于锁存接触、历史前缀不变、旧模式复现。

## 测试和审查

125 项相关测试通过（3 条既有依赖弃用警告）。其中新增 14 项覆盖地图假阳性、物理接触假阴性、真实 execute 循环中短暂接触锁存及下一块清空、NaN 接触矩阵错误、不可用传感器、CLI/子进程/协议传播、两种单独汇总及混合拒绝、异常关闭前保存证据。首次 reset 故障以空初态 runtime_error 返回模拟；中途故障测试模拟 runner 抛错以验证外层审计，并不冒充真实 Isaac 传感器损坏实验。

独立只读代码审查发现的“首次 reset 失败丢失接触证据”问题已修复并复核，无剩余阻塞意见。此前回归先因缺少新版本接口失败，随后实现并通过测试。没有提交或推送 Git，也没有清理历史改动。

## 文件与复现

工作树：`/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark`。

修改：`research/ball_wm/benchmark/isaac_runtime.py`、`research/ball_wm/scripts/run_pointgoal_benchmark.py`；新增回归：`research/ball_wm/tests/test_collision_v2.py`。README 补充新旧判定用法。

本轮原始目录：`/home/an/Documents/ChatGPT/球型机器人导航/experiments/oracle_collision_fix_20260929_v8`。包含：

- `before/`、`after/`、`collision_fix.patch`：本轮改动的精确边界；不以脏工作树与 HEAD 的大 diff 代替。
- `source_identity.json`、`source_snapshot.tar.gz`、`identity_after.json`：实际运行源码身份。
- `*_command.json`、五条日志、`status.json`：真实执行命令、退出码和墙钟用时。
- 五个运行目录：metrics、environment、collision_audit、状态/动作、轨迹图、双目/第三人称视频。
- `analyze.py`、`results.json`、`verify_results.py`、`verification.json`：新旧前缀及判定对照。
- `test_collision_v2.py`、`tests.log`：测试与输出。

重跑必须使用新的空输出目录；`run_validation.py` 保留本轮精确命令。新旧分数必须按协议分开。项目内另保存同名归档 `research/ball_wm/runs/benchmark_oracle_collision_fix_20260929_v8`。

## 解决了什么，仍需做什么

已解决判定链把栅格近障误当碰撞，以及实际碰撞未触发终止的问题。修复没有让困难路线自动成功：0002 现在能多走一段，但 35 秒仍距目标 6.41 m；真实碰撞案例也仍然会失败。下一项控制工作应使用新判定和真实进度分析这些超时/实际碰撞，针对轨迹可行性与跟踪能力改进，而不是继续通过调整碰撞口径改善分数。
