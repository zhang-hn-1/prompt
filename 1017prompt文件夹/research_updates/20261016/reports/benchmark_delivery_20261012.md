# 球型机器人导航：分层基准与记忆证据入口

## 交付状态

已完成既有12个固定条件的原始证据复算，以及已封存空间记忆消融的范围核验。本轮没有新增仿真、模型预测、训练或教师查询。基线与基准证据可以交付；可靠估计位姿导航和学习导航增量仍未完成，严格冷启动视觉导航尚未成立。

| 方法 | 已知位姿 | 估计位姿 |
|---|---:|---:|
| 冻结传统双目风险记忆闭环 | 0/3到达 | 0/3到达 |
| 滚动参考末端变体 | 1/3到达 | 0/3到达 |

每个格子是E0/E1/E2三个固定开发条件各一次，不是独立随机重复的成功概率。两方法版本与特权条件分别列出，不能合并成12个独立泛化样本。新航向反馈候选已拒绝，不迁入这张历史矩阵。滚动参考的E0局部成功也没有完整RGB因果前缀，不能宣称因果增益或可靠导航。

## 全条件表

| 方法 | 位姿 | 场景 | 终态 | 净目标轴进展(m) | 目标剩余(m) | 接触时刻(s) | 原始目录 |
|---|---|---|---|---:|---:|---|---|
| 传统 | 已知 | E0 | timeout | 5.309220 | 3.532848 | 无 | [sgbm_baseline35_e0_coverage_run_v1](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/sgbm_baseline35_e0_coverage_run_v1/task_episode.json) |
| 传统 | 已知 | E1 | timeout | 0.713518 | 5.890992 | 无 | [sgbm_baseline35_e1_coverage_run_v1](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/sgbm_baseline35_e1_coverage_run_v1/task_episode.json) |
| 传统 | 已知 | E2 | timeout | 0.785604 | 8.118060 | 无 | [sgbm_baseline35_e2_run_v1](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/sgbm_baseline35_e2_run_v1/task_episode.json) |
| 传统 | 估计 | E0 | timeout | 5.269048 | 3.576908 | 无 | [native_pose_baseline35_e0_run_v1](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/native_pose_baseline35_e0_run_v1/task_episode.json) |
| 传统 | 估计 | E1 | timeout | 0.753600 | 5.850502 | 无 | [native_pose_baseline35_e1_coverage_run_v1](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/native_pose_baseline35_e1_coverage_run_v1/task_episode.json) |
| 传统 | 估计 | E2 | collision | 2.426319 | 6.475446 | 19.640 | [native_pose_baseline35_e2_coverage_run_v1](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/native_pose_baseline35_e2_coverage_run_v1/task_episode.json) |
| 滚动参考 | 已知 | E0 | success | 8.656947 | 0.198045 | 无 | [rolling_endpoint_navigation_e0_paced_run_v1](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/rolling_endpoint_navigation_e0_paced_run_v1/task_episode.json) |
| 滚动参考 | 已知 | E1 | timeout | 0.975779 | 5.626473 | 无 | [rolling_endpoint_coverage_known_e1_run_v1](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/rolling_endpoint_coverage_known_e1_run_v1/task_episode.json) |
| 滚动参考 | 已知 | E2 | collision | 2.416235 | 6.485318 | 26.680 | [rolling_endpoint_coverage_known_e2_run_v1](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/rolling_endpoint_coverage_known_e2_run_v1/task_episode.json) |
| 滚动参考 | 估计 | E0 | timeout | 1.359185 | 7.487079 | 无 | [rolling_endpoint_native_e0_paced_run_v1](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/rolling_endpoint_native_e0_paced_run_v1/task_episode.json) |
| 滚动参考 | 估计 | E1 | timeout | 0.659754 | 5.951325 | 无 | [rolling_endpoint_coverage_native_e1_run_v1](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/rolling_endpoint_coverage_native_e1_run_v1/task_episode.json) |
| 滚动参考 | 估计 | E2 | timeout | 3.487544 | 5.413114 | 无 | [rolling_endpoint_coverage_native_e2_run_v1](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/rolling_endpoint_coverage_native_e2_run_v1/task_episode.json) |

运行目录各含states、physics、contacts、requests、policy/source记录、身份、终态、封存清单及视频。可机读表：[baseline_12_conditions.csv](../experiments/benchmark_delivery_20261012_v1/baseline_12_conditions.csv)。复算记录：[existing_evidence_audit_v1.json](/home/an/Documents/ChatGPT/球型机器人导航/experiments/benchmark_delivery_20261012_v1/existing_evidence_audit_v1.json)。

## 评分与特权

原时间上限35秒、目标XY半径0.2米、真实接触优先；不依据路径游标判到达。碰撞可发生在20ms物理采样之间的100ms捕获前，因此碰撞条件的目标距离和净进展取不晚于接触的最后一次捕获，表中接触时刻与该状态时刻可能不同。原评分语义完整保留，不将碰撞后的捕获用于提高进展。

已知位姿条件使用当前仿真真实姿态/速度进行配准与反馈，是特权诊断。估计位姿条件使用在线估计，也有初始位置/旋转规范、理想模拟IMU与平地高度等条件。TaskH地面先验、安全起步和受控未知探索均保留。没有匹配或没有新点不能解释为已知自由。固定相机ball_stereo_front020_up005_v2、SGBM及照明；完整配置和来源在各原始封存目录。

## 估计位姿诊断

对六条估计位姿序列的所有已保存在线预测，与同刻物理记录逐点核对，保持初始XY规范且不做事后配准。各运行最大XY位置误差为0.0062–0.0237米，最大水平投影航向误差为0.382–0.620度，最大XY速度误差为0.0124–0.0190米/秒。

这些数值取所有已保存预测；经典估计位姿E2末预测为19.70秒，接触为19.64秒，故误差final并非碰撞评分瞬间。真实状态只用于本次误差比对，原估计器的真实初始规范和理想IMU特权并未消失。

这些数值只描述本组理想模拟传感器条件。不能将失败归因于大幅位置漂移，也不能反向证明小估计误差不影响离散规划、记忆配准或命令资格。这里没有安装位姿修正，更没有估计位姿改进的因果实验。详细样本数与误差见[saved_pose_diagnostic_v1.json](../experiments/benchmark_delivery_20261012_v1/saved_pose_diagnostic_v1.json)。

## 空间记忆：机制与导航结果分开

历史风险组35秒超时、净进展0.713518米，无接触；最新曝光组在21.18秒发生接触，评分净进展2.569541米，均未到达。最新曝光走得更远不构成更好的导航。两臂源图像和原始点几何前缀不同，因此仅支持成对观测结果，不授完整因果导航收益。

同一最新曝光序列、同一过去源记录、当前位姿与已保存参考下，重算30个原保留资格边界，其中11个边界当前曝光允许保留、历史风险会拒绝。从17.5秒起有连续8个曝光没有风险入选点，历史仍保留之前的风险。该证据表明历史风险确实参与资格判断，不能证明改动后的机器人一定避开碰撞。没有新风险点始终为未知，不能当作自由；不能把无点来源全部称为遮挡。

记忆组原35秒实验预算已关闭，不追加场景或重跑。本次复核不重复policy act，不产生预测或替代轨迹。既有有限证据足以支持机制与负结果交付；公平源输入下的导航因果收益尚未成立。

## 学习数据资格

现有12条轨迹只对应3个固定episode组，不能把相邻帧随机分训练/留出后宣称独立验证。实际执行轨迹不提供任意候选命令的反事实标签。已有风险资格学习器覆盖不足、执行记忆153库/30锚点仅6可评分的结论及关闭预算均保留。

本轮未选择或训练新候选；独立学习目标、在线消费者与新整序列数据协议仍是下一项工作。[learning_data_qualification_decision_v1.json](../experiments/benchmark_delivery_20261012_v1/learning_data_qualification_decision_v1.json) 明确现有数据复用不自动授予训练、采集或排名预算。这并不否定所有世界模型。

## 代码与复核命令

Python：/home/an/miniconda3/envs/ball_wm/bin/python。原代码工作树只读，数据引用原位置。此交付是有来源身份的复核入口，不是已经可搬移到另一机器的独立安装包。

复核代码：[audit_existing_evidence_v1.py](../experiments/benchmark_delivery_20261012_v1/audit_existing_evidence_v1.py)、[audit_saved_pose_v1.py](../experiments/benchmark_delivery_20261012_v1/audit_saved_pose_v1.py)。下列命令为本次执行记录，已存在的审计输出会拒绝覆盖，不应直接重跑封闭物理预算：

```text
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm /home/an/miniconda3/envs/ball_wm/bin/python /home/an/Documents/ChatGPT/球型机器人导航/experiments/benchmark_delivery_20261012_v1/audit_existing_evidence_v1.py
PYTHONDONTWRITEBYTECODE=1 /home/an/miniconda3/envs/ball_wm/bin/python /home/an/Documents/ChatGPT/球型机器人导航/experiments/benchmark_delivery_20261012_v1/audit_saved_pose_v1.py
```

原始物理驱动与运行命令以各封存contract/identity和原报告为准。没有为了交付而授权重做12条件或修改原项目。环境可用性与可迁移打包仍需在最终最小入口中明确。

## 后续

任务三已有12条件表和位姿诊断；没有值得迁移的新反馈候选。任务四完成既有消融的有限结论复用，不新增物理。下一直接冻结任务五的独立学习目标/数据资格设计，或明确独立采集的必要范围与否证条件；再完善任务六可复现最小入口与依赖清单。可靠导航目标继续单列未完成。

## 用户可直接运行的只读核验

完整交付清单生成后，以下最小入口可重复检查原文件与交付身份，不覆盖评分、不启动仿真或训练：

```text
/home/an/miniconda3/envs/ball_wm/bin/python /home/an/Documents/ChatGPT/球型机器人导航/experiments/benchmark_delivery_20261012_v1/verify_delivery_v1.py
```

本入口依赖原始数据绝对路径，搬移后需要独立路径映射与验证；尚未声称跨机器安装包已完成。
