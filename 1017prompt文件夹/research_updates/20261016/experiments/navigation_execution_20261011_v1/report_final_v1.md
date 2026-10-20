# 水平航向角速度反馈：E1 实际对照结论

## 结论

本轮候选不予保留：两个开发条件均未在35秒到达，候选净进展低于同版基线。保留原基线并关闭本轮实际两臂预算，不调整增益、延长时间或复跑候选。整体项目未完成。

| 指标 | 同版B0 | 候选 |
|---|---:|---:|
| 到达 | 否，35秒超时 | 否，35秒超时 |
| 净目标方向进展 | 0.713518 m | 0.657022 m |
| 最终目标剩余距离 | 5.890992 m | 5.947890 m |
| 接触采样为真 | 0/2101 | 0/2101 |
| 横向偏差RMS | 0.271112 m | 0.271345 m |
| 横向偏差峰值 | 0.610739 m | 0.578500 m |
| 端到端时延中位 | 0.306763 s | 0.308630 s |
| 端到端时延p90 | 1.378648 s | 1.351509 s |

实际请求介入235次，两组首个请求分叉为frame 68。当前输入的算术重建证明修改确实进入BankMap并发出动作。完整源/物理前缀并不一致，因此不把全部轨迹差异归因于这一个修改；这是同条件开发比较，未授完整因果证明。

## 实施与审查

唯一改变：以当前四元数和world角速度计算机体X轴水平投影航向的瞬时导数，替换现有外环反馈中的world-z测量项。反馈增益、±5度限制、主通道、BankMap、规划、风险记忆、PID和两臂稀疏SGBM输入规则不变。26项候选/评分CPU检查及23项既有跟踪检查通过。独立预审的奇异姿态回退、身份说明与源帧完整性问题已修正，旧字节与RED/GREEN证据保留。独立原始结果闭合审查另存。

旧数据坐标核验：351捕获/1751物理采样，投导航向导数对20ms航向差分的残差p90为0.003490rad/s，对world-z为0.026398rad/s。坐标更一致并未带来本轮导航收益。

## 启动与原始记录

隔离环境最初B0在SimulationApp构造时runtime_failed(-11)，未创建任务场景或开始捕获。原协议、no_retry、状态、日志与启动产物全部保留。宿主只读显卡检查成功，独立审查事前登记唯一环境恢复；新driver/protocol/监督仅版本路由不同，归一后AST一致。经权限审批完成宿主B0与candidate各唯一一次；后续恢复和重试禁止。

宿主B0：2026-10-11 16:05:08–16:08:32 UTC；candidate：16:08:47–16:12:07 UTC（北京时间10月12日）。两臂均finished0，无监督超时、强杀或父信号。各350请求、351捕获、1751物理采样、2101接触检查、2284封存产物；各双视频351帧/10Hz。评估器已核对源码/输入/封存SHA、真实请求和物理目标、完整源帧及原surface replay、实际反馈映射、同刻姿态/角速度/速度、时间与视频帧数。

## 可复现入口

- 设计：[heading_rate_design_v1.json](heading_rate_design_v1.json)
- 原始评分：[heading_rate_e1_host_assessment_v1.json](heading_rate_e1_host_assessment_v1.json)
- B0冻结配置：[protocol_v3](heading_rate_e1_b0_protocol_v3.json)；候选：[protocol_v3](heading_rate_e1_candidate_protocol_v3.json)
- 代码：[heading_rate.py](heading_rate.py)、[driver_v2](run_heading_rate_e1_v2.py)、[scorer](assess_heading_rate_e1_host_v1.py)
- 基线原始目录：/home/an/Documents/ChatGPT/球型机器人导航/experiments/navigation_execution_20261011_v1/heading_rate_e1_b0_run_v2
- 候选原始目录：/home/an/Documents/ChatGPT/球型机器人导航/experiments/navigation_execution_20261011_v1/heading_rate_e1_candidate_run_v2
- 环境：Python /home/an/miniconda3/envs/ball_wm/bin/python；RTX3060/驱动580.178.04；原场景/依赖/输入路径详见冻结协议。

本轮运行命令为宿主监督入口加b0/candidate臂参数；已封存目录和status禁止再次运行，命令只用于理解重现流程，不授已关闭预算。评估入口同样禁止覆盖已有评分。新复现实验必须有独立设计、独立输出和明确预算。

![本轮轨迹与角速度对照](/home/an/Documents/ChatGPT/球型机器人导航/experiments/navigation_execution_20261011_v1/heading_rate_E1_comparison_v1.png)

主执行者已查看此图；视频仅做帧数/帧率解码核验，未声称人工观看全视频。

## 剩余工作

复用既有12条件分层矩阵（本次55个引用SHA全部一致），不把新E1开发对照混入成功率。原传统known0/3、estimated0/3；滚动参考known1/3、estimated0/3。新候选未通过，禁止迁入估计位姿或E0/E2矩阵。

接下来先对齐已有空间记忆消融的版本、输入及结论范围，再明确学习目标/新数据覆盖是否具备独立设计资格，最后整理基线与基准的权威交付。旧学习、建库和密集采样预算仍关闭。可靠估计位姿导航、严格视觉导航及学习导航增量尚未成立，不能将负结果交付等同于这些目标完成。


独立实际闭合审查：无重要问题，原始轨迹、235次介入、完整源码/输入/产物SHA及视频重新核验，确认measured_navigation_reject。审查记录：/home/an/Documents/ChatGPT/球型机器人导航/experiments/navigation_execution_20261011_v1/independent_actual_closure_review_v1.json。
