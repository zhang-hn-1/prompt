# 基础运动预测模型：等更新预算的数据扩充对照（2026-09-27）

实际完成原8条与扩充12条数据的A/B训练。扩充组原28项门槛通过 **24/28**。**拒绝本候选，封存fresh未启用。**

## 实验问题与边界

上一轮独立校准门控未带来整体改善。本轮不使用门控，检验增加四条真实运动序列能否改善基础加速度残差预测；网络结构、初始权重、损失、优化器与评估窗口都不变。冻结双目编码器；其视觉特征仍参与预测，但本轮未验证视觉相对本体信息的独立贡献。

| 项目 | A：原数据 | B：扩充数据 |
|---|---:|---:|
| 完整训练序列 | 8 | 12 |
| 总训练时长 | 192秒 | 288秒 |
| 训练窗口 | 1848 | 2772 |
| 完整遍历轮数 | 30 | 20 |
| 优化更新次数 | 27720 | 27720 |
| batch size | 2 | 2 |
| 验证/选模机会 | 每924次更新，共30次 | 相同 |

两组84,931参数，同一个seed3072和逐张量完全相同的原始初始化，AdamW lr=5e-5、weight_decay=1e-4。均按历史验证motion loss最低选择，平局选更早；不按测试分数或28项通过数选轮。扩充组每条样本重复次数减少，是在相同更新预算下扩充数据支持范围的对照；不是“相同轮数多训练50%”。只有一个随机种子，不宣称统计显著性。

## 数据隔离与已知例外

复用已核验的原始数据及相同冻结编码器的特征缓存。新增四条原先用于独立门控校准，这次在训练前明确划入基础模型训练集；今后不能再当独立校准或fresh测试。原val/test各两条完整序列仍只作历史回归，既不进入训练，也不重新随机拆帧。原封存四条fresh保持隔离。

第三条speed_histories最大applied转向目标16°，原正式V2逐序列>16.1°覆盖要求仍失败；四条的29项结构/时序/执行一致性全部通过，其余三条满足原覆盖。本轮训练前明确允许此完整低角度序列用于研究训练，不修改旧审计或称原正式审计全通过。所有接触标签片段保留。

## 原组复现与实际训练

原组完整30轮重新运行，和原实验相同初始化、顺序与更新。训练/验证历史最大loss差 0；通过预定1e-9复现要求。

A_original：选中第15个验证点，累计13860次更新；实际训练耗时83.97秒。

B_expanded：选中第23个验证点，累计21252次更新；实际训练耗时82.77秒。

![相同更新预算下的训练比较](/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/wm_aux30_data_expansion_20260927_v1/equal_budget_training.png)

## 相同历史窗口的预测结果

每个集合94个窗口；XY为平均欧氏误差cm，yaw为平均包角绝对误差°。不是导航成功率。

| 集合 | 时域 | 匀速 XY | A XY | B XY | B/A XY变化 | A yaw | B yaw |
|---|---:|---:|---:|---:|---:|---:|---:|
| val | 0.1s | 0.0881 | 0.0874 | 0.0781 | -10.61% | 0.0452 | 0.0433 |
| val | 0.3s | 0.5850 | 0.4720 | 0.4015 | -14.92% | 0.2149 | 0.1992 |
| val | 0.5s | 1.5051 | 1.1163 | 0.9444 | -15.40% | 0.5103 | 0.4834 |
| test | 0.1s | 0.0763 | 0.0643 | 0.0615 | -4.27% | 0.0348 | 0.0380 |
| test | 0.3s | 0.5497 | 0.3448 | 0.3062 | -11.21% | 0.1678 | 0.1977 |
| test | 0.5s | 1.4636 | 0.7997 | 0.6906 | -13.64% | 0.3547 | 0.4317 |

![数据扩充后的预测误差](/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/wm_aux30_data_expansion_20260927_v1/data_expansion_prediction.png)

原“请求变化”分组含未来半秒内请求变化，不等价于预测起点刚发生换向。

| 集合 | 时域 | 匀速 XY cm | A XY cm | B XY cm |
|---|---:|---:|---:|---:|
| val | 0.1s | 0.0484 | 0.0805 | 0.0686 |
| val | 0.5s | 0.6664 | 1.0297 | 0.9387 |
| test | 0.1s | 0.0448 | 0.0565 | 0.0388 |
| test | 0.5s | 0.7411 | 0.6282 | 0.5294 |

## 验收与处置

原28项模板及比较对象保持不变：24项历史规则，加上半秒相对冻结加速度模型的XY/yaw保留要求。不会改成只比较较差的新门控。

| 未通过检查 | 实际比值 | 原门槛 |
|---|---:|---:|
| val/request_change_0.5_xy | 1.408664 | 1.0 |
| val/request_change_0.1_xy | 1.417734 | 1.0 |
| test/preserve_parent_0.5_yaw | 1.085276 | 1.05 |
| test/preserve_acceleration_0.5_yaw | 1.217209 | 1.05 |

历史门槛未通过：拒绝候选，不采集fresh。

本轮未改变planner、Oracle、执行限速、动作范围或正式35秒规则；没有运行E0/E1/E2。候选完整权重和失败项保留，不用局部误差改善替代所有验收项。

## 验证证据

新数据划分/等预算采样四项测试先失败、实现后通过；本轮针对运动预测与数据划分的实际测试为 **18 passed、3 warnings，2.27秒**（14项既有运动测试+4项新划分/采样测试），不是整个仓库测试套件。两组全部评估窗口执行target删除/NaN替换不变性测试；前缀因果性、相同窗口和匀速基线逐项核验。运行前后检查189份源码、固定输入及全部原始数据指纹。

证据：[训练前冻结规则](/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/wm_aux30_data_expansion_20260927_v1/protocol.json)；[数据划分](/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/wm_aux30_data_expansion_20260927_v1/data_scope.json)；[原组复现](/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/wm_aux30_data_expansion_20260927_v1/reproduction.json)；[28项验收](/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/wm_aux30_data_expansion_20260927_v1/B_expanded/assessment.json)；[选模记录](/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/wm_aux30_data_expansion_20260927_v1/B_expanded/selection.json)；[实际测试](/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/wm_aux30_data_expansion_20260927_v1/behavior_tests.log)；[最终核验](/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/wm_aux30_data_expansion_20260927_v1/final_verification.json)。

## 结论与后续重点

这次数据扩充对位置预测有明确的本次实验收益：验证/历史测试半秒XY平均误差分别下降15.40%和13.64%；历史测试请求变化组的0.1秒XY由0.0565cm降到0.0388cm，本次该项已优于匀速0.0448cm。原组已逐张量复现，因此此对照没有把不同初始化或更多总训练更新混作数据收益。

同时，历史测试半秒yaw从0.3547°上升到0.4317°，增加21.72%；验证请求变化组的0.1/0.5秒位置预测仍差于匀速。通过项计数为24/28，与未加门控原组相同，但失败项发生了变化，不能只看通过总数。保留本候选作为位置预测改善的研究结果，不晋升为导航模型。

下一步应集中解决朝向预测的退步及验证请求变化片段，不再继续叠加门控。当前证据尚不能区分位置/朝向共享训练的折中、具体状态覆盖或单种子波动；应先在现有数据上定位这些误差对应的真实运动状态，再选择一个受控修正。新增数据已用于训练，不能再伪装成fresh验证。
