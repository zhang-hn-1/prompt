# 原完整目标35秒：双目已知位姿控制与同源只读记忆

本轮ONE真实完整goal运行，未延长35秒、未换网络/增益/映射/速度/规划/参考规则。继承局部2m current到达与history候选拒绝；为隔离前轮重置RGB混杂，在同一源流上增加独立history shadow。整体研究仍未完成。

## 完整目标结果

原golden E1目标保持 (-3.3,0)→(3.3,0)，6.6m直线位移。actual episode只换实验ID，其他原字段逐项一致。原6秒观察含在35秒内。

- 实际35秒超时，终距4.489753m，障碍physicalcontact0、runtimeerror0。
- 实际朝目标直线方向进度2.112201m；轨迹总长3.859148m。直线progress不是绕障参考进度，不按policy path_index认定通关。未接近分隔墙的实际绕行段，不能称已通过主要导航难点。
- 350次动作、351相机捕获、1751个50Hz物理采样、71次冻结双目网络预测，两完整视频各351帧/10fps。
- 原完整导航本实例未通过；这是knownpose+显式未知风险条件的诊断，不是正式RGB+estimatedpose/E0E1E2成功率或worldmodel结果。2m局部到达没有被升级为完整目标通过。

| 实际控制阶段 | 仿真时间 | 进入→退出的真实目标方向进度 |
|---|---:|---:|
| 初始观察 | 0–6.0 s | 0→.03325 m |
| 非零请求 | 6.0–20.0 s | .03325→1.58345 m |
| tracking零请求 | 20.0–21.6 s | 1.58345→1.63755 m |
| 非零请求，包含倒行 | 21.6–23.8 s | 1.63755→1.42603 m |
| tracking零请求 | 23.8–25.6 s | 1.42603→1.31508 m |
| 非零请求 | 25.6–29.0 s | 1.31508→1.57011 m |
| tracking零请求 | 29.0–29.5 s | 1.57011→1.61374 m |
| 非零请求 | 29.5–35.0 s | 1.61374→2.11220 m |

初始观察之外，251块非零请求、39块tracking零请求、0块no-route。不能把完整goal失败都解释成前轮的第一次无路，也不能只说因速度慢：本轮还发生倒行/方向确认，使真实progress下降。阶段分类只由真实request和当前plan状态产生，详细方向/参考链保存在原records；它不是新的低速停稳资格门槛。

## 同源在线shadow验证

primary仍为同一current RGB来源控制器；shadow为独立history对象，输入当前/过去**同一个**source包、同一个实际当前状态与给定目标。输入传副本，各自policy仅调用一次。shadow不调用simulator、不拿未来反馈/未来请求构建计划，raw请求只记日志，永不执行。每次调用断言primary动作、边界状态和物理时钟不变。没有用另一个计划的B0未来轨迹当真实标签。

350次均核对来源frame/time一致，source年龄最多.4秒；shadow有242块请求与primary不同，第一次6.0秒。它说明同源记忆消费者能改变建议请求；**没有执行shadow，所以不能给出其替代轨迹、接触、到达时间或导航成功率**。当前真实轨迹只属于primary。运行中不变断言和CPU on/off小fixture是接口证据，本轮没有另一条实际shadow-off fullgoal物理序列。

模型/source旧门槛与M2/位姿/旧导航拒绝仍保留。flat-ground/safe-start层不进观测free；global map/Oracle reference不给policy，GTdepth/occupancy只离线。

## 计算和物理预算

模型wall p50/p95 .943995/.971080 s，shadowwall p95 1.800510 s，采集推理/规划期间sim暂停。不能宣称实时10Hz。当前XY实测速度p95 .186519m/s；即使只按6.6m直线、扣除6秒观察，平均至少.227586m/s才可能35秒到达，实际绕障还更长。这是预算差距证据，不是全域动力学上界，也不单独解释倒行/停车。

## 新的有界原因核验：运动时漏掉机体表面

只用前轮第一次阻断的两个fresh假障碍格 (-1.1,-.2)、(-1.0,-.2) 做事后诊断，不运行新模型/物理、不改旧mask/TTL。12个来源点都通过旧bool置信度/左右一致性，且全部落在冻结leftmask之外；其当前像素真值三维误差1.1166–1.1358m。真实端点距机器人中心.3957–.39695m，符合球壳表面附近；预测位置却在前方两格。

校验原E1场景SHA，使用其authored圆柱/地面静态表面计算距离：预测点距静态表面.642–.753m，真值端点距静态表面.517–.525m。它支持这些像素来自近身自表面而非静态环境，但未读取渲染实例ID，不能把具体刚体身份写成已独立确认。

冻结mask元数据明确 `covered_body='base_link only, rigidly fixed to both cameras'`。CAD还存在独立运动的link1/2/3；公共proprio的17维向量已包含三关节位置（9:12），不是需要未来标签才能获得的信息。下一项只验证一个基于CAD与当前关节的自遮挡口径，先核对FK与实际刚体/渲染时间，再决定能否接入。旧base_link mask保持原字节，候选若扩展也必须独立命名；不凭这12点降低旧source门槛或直接删除“看起来是假”的点。

## 实际证据和接续

3个新shadow TDD import RED→GREEN3passed/3.64s；完整369suite/14.84s。新runner明确替换可逆回到封存局部runner；taskgoal恢复及只读shadow之外primary无代码变化。1252文件运行前封存；UTC19:38:40→19:47:19、exit0；3807项源/实际产物SHA核验、两完整视频解码和左右capture/render clock差0。posthoc assessor/plot/diagnosis源码单独记录，不能追称在运行前预封存。

H=`experiments/stereo_memory_navigation_20261004_v1`：`rgb_complete_goal_protocol/status/derivation/assessment.json`、`rgb_complete_goal_current_run`、`run_rgb_complete_goal.py`、`same_stream_memory_shadow.py`、测试/日志、两段视频；fresh blocking cells诊断v1/v2保留。旧局部报告与其final owned SHA未修改。

![真实轨迹及只读请求](../experiments/stereo_memory_navigation_20261004_v1/rgb_complete_goal_actual_trajectory_and_shadow.png)

下一项针对自遮挡来源，保持相机安装/光照/控制不变；不无限更换网络/拟合运动模型，不把完整goal失败转成延时“成功”。可信记忆、RGB+估计位姿、学习增量与原35秒正式E0/E1/E2全层仍未完成。自动每小时接续保持ACTIVE。
