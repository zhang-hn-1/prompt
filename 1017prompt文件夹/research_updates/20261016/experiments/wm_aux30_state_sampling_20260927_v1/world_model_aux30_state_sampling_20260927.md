# 实际状态均衡采样：单候选对照（2026-09-27）

完成一个固定比例的训练候选C。原28项通过 **23/28**；加入四项上一版位置收益保留要求后共 **25/32**。**拒绝候选，fresh未启用。**

## 唯一改动

B为上轮12条数据均匀遍历的扩充模型；C复用同一12条数据、2772训练窗口、初始权重、84,931参数网络、损失、AdamW配置与27,720次更新。每924次更新验证一次，共30个选模点，按历史验证motion loss最低选择，不按测试分数或通过项数选权重。本轮只训练C，B沿用已完整运行并核验的冻结结果。

唯一改变是训练样本频率和顺序：较快倒车、大副轴角的正/负角子组各占5%，其余90%。总10%接近原8条训练数据9.79%的联合状态占比，正负对称；比例在训练前固定，没有扫描。只有单一随机种子，不把这一次结果当作统计显著结论。

分类只读取训练窗口锚点的实测状态：XY速度≥0.15m/s、yaw前向速度<−0.05m/s、副轴角≥15°或≤−15°；不读取未来真值、请求计划或测试误差。所有推理模型仍使用原输入，没有按测试序列名、时间或状态切换动作。

| 训练子组 | 不同窗口数 | 整个预算出现次数 | 固定比例 |
|---|---:|---:|---:|
| 其余状态 | 2568 | 49896 | 90% |
| 正副轴角倒车转弯 | 97 | 2772 | 5% |
| 负副轴角倒车转弯 | 107 | 2772 | 5% |

固定配额打乱排列，子组内部反复随机无放回遍历；跨遍历边界可能在同一batch出现重复样本，属于事前披露的采样语义。不存在均匀“20轮”概念，不能将加权重复称为新增独立数据。

新增四条仍属于训练数据；其中speed_histories原正式高角度覆盖失败记录继续保留，研究训练适用范围沿用上轮，其余完整性检查未放宽。原val/test仍是已用于开发的历史回归，不是fresh。

## 实际执行与选模

实际训练耗时85.38秒；选中第14个验证点，累计12936次更新。到该权重为止，正/负倒车转弯子组实际出现1317/1309次，其余23246次。完整训练预算与选中权重之前的实际曝光分开报告。

## 相同历史窗口的结果

A=原8条训练模型，B=12条均匀采样模型，C=本轮状态采样模型。位置为平均欧氏误差，朝向为平均包角绝对误差。

| 集合 | 时域 | A XY cm | B XY cm | C XY cm | A yaw° | B yaw° | C yaw° |
|---|---:|---:|---:|---:|---:|---:|---:|
| val | 0.1s | 0.0874 | 0.0781 | 0.0812 | 0.0452 | 0.0433 | 0.0474 |
| val | 0.3s | 0.4720 | 0.4015 | 0.4276 | 0.2149 | 0.1992 | 0.2248 |
| val | 0.5s | 1.1163 | 0.9444 | 0.9927 | 0.5103 | 0.4834 | 0.5120 |
| test | 0.1s | 0.0643 | 0.0615 | 0.0644 | 0.0348 | 0.0380 | 0.0405 |
| test | 0.3s | 0.3448 | 0.3062 | 0.3220 | 0.1678 | 0.1977 | 0.2132 |
| test | 0.5s | 0.7997 | 0.6906 | 0.7452 | 0.3547 | 0.4317 | 0.4806 |

![相同历史窗口的预测](/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/wm_aux30_state_sampling_20260927_v1/state_sampling_results.png)

原请求变化分组含未来半秒的变化，不等于预测起点正在实际换向。

| 集合 | 时域 | 匀速 XY cm | B XY cm | C XY cm |
|---|---:|---:|---:|---:|
| val | 0.1s | 0.0484 | 0.0686 | 0.0703 |
| val | 0.5s | 0.6664 | 0.9387 | 0.9147 |
| test | 0.1s | 0.0448 | 0.0388 | 0.0473 |
| test | 0.5s | 0.7411 | 0.5294 | 0.6075 |

## 未通过项目与处置

原28项的数值门槛和比较对象全部保留；额外四项为val/test的0.1/0.5秒XY误差不得超过冻结B的1.05倍，防止以牺牲位置收益换朝向收益。

| 检查 | 实际比值 | 上限 |
|---|---:|---:|
| val/request_change_0.5_xy | 1.372676 | 1.0 |
| val/request_change_0.1_xy | 1.452525 | 1.0 |
| test/preserve_parent_0.5_yaw | 1.208227 | 1.05 |
| test/request_change_0.1_xy | 1.056626 | 1.0 |
| test/preserve_acceleration_0.5_yaw | 1.355107 | 1.05 |
| val/preserve_B_0.5_xy | 1.051177 | 1.05 |
| test/preserve_B_0.5_xy | 1.079093 | 1.05 |

判定：拒绝，不接入导航，封存fresh保持未生成。 没有调整阈值、重选测试上更好的轮次或再训练第二个比例。

## 测试与复现

六项采样器测试先失败、实现后通过；本轮相关运动模型与采样器测试 **20 passed、3 warnings，1.96秒**，不是完整仓库套件。训练函数由上轮脚本逐字提取，仅替换minibatch索引迭代器；源码身份和差异记录见implementation_provenance.json。训练前后核验原缓存、数据状态、比较模型和189份源码指纹，未改planner、Oracle或限速。

完整采样索引提前保存并固定SHA；没有训练/验证/测试完整序列混用。预测移除target或将其换为NaN后不变，原窗口/匀速基线与请求前缀因果性保持原检验。

[训练前协议](/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/wm_aux30_state_sampling_20260927_v1/protocol.json)；[完整采样计划](/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/wm_aux30_state_sampling_20260927_v1/schedule.npy)；[选中权重实际曝光](/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/wm_aux30_state_sampling_20260927_v1/selected_exposure.json)；[全部32项验收](/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/wm_aux30_state_sampling_20260927_v1/assessment.json)；[选模记录](/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/wm_aux30_state_sampling_20260927_v1/C_balanced/selection.json)；[唯一代码差异](/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/wm_aux30_state_sampling_20260927_v1/implementation_provenance.json)；[最终核验](/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/wm_aux30_state_sampling_20260927_v1/final_verification.json)。

## 局部收益与整体代价

按上轮已冻结的实测状态标签复核，test中“较快倒车＋大副轴角”的11个窗口，半秒yaw由B的1.1307°降到C的0.8657°，改善约23.4%；但val中同类6个窗口由0.4897°升到0.6420°。目标状态在test有局部收益，却未跨序列稳定复现。

全体test半秒XY由0.6906cm升到0.7452cm（+7.91%），yaw由0.4317°升到0.4806°（+11.33%）；全体val也退步。不能用目标小组的改善抵消整体代价，或事后按episode选择不同模型。

选模规则自动选中C的第14个验证点，而B选在第23个验证点。C选中时正/负目标子组曝光1317/1309次，B为1495/1646次；虽然C的采样比例更高，选中权重的累计次数更少。保留真实选模结果，不用后续轮次的完整预算曝光来解释已选模型，也不改为测试上更好的权重。

本轮只能拒绝这个固定比例、单种子的采样候选，不能证明所有状态采样都无效，也不能证明唯一根因。停止继续扫描比例，保留B作为位置预测较好的研究对照，其原拒绝结论仍在。世界模型导航仍缺可靠的跨状态预测、候选动作排序与闭环验证。
