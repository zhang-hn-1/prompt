# 当前移动方向反馈：固定局部参考实测

局部跟踪确有改善，但本轮原验收条件未全部通过，保留REJECT；没有据此启动正式导航或替换正式控制器。

对照使用相同近场起点、−20°机体朝向、18秒退后观察前缀和同一条完整局部参考。参考来自之前18秒规划的前四个点，末点请求归零，终点为(-1.06468233,.18290872)，全部位于障碍起始侧。这是新的短段跟踪任务，不是原始整条绕障路径；已知参考明确标为先验，不冒充当时的视觉观测。

唯一候选把追踪目标转换到当前实际XY速度方向，以已接受的正/反向转换到机体方向约定；速度低于既有.05m/s有效性阈值时精确沿用机体yaw。没有修改视觉世界姿态、全局点目标、参考/索引投影、BankMap逆映射、PID增益、速度策略、.3上限或五次停稳/反弹规则。实际类链已核验：outgoing-endpoint → confident-speed → BankMap → cusp-boundary → OraclePathPolicy；本版本只使用pure-pursuit几何曲率，profile是诊断量。真实参考目标和追踪代理目标分别记录，root_heading字段一直是实际机体朝向。

## 实际结果

| 指标 | B0 | 当前速度方向候选 |
|---|---:|---:|
| 到达局部目标时刻（含观察） | 25.7s | 25.5s |
| 目标余距 | .195152m | .194307m |
| 实际接触 | 0 | 0 |
| 交接后横向RMS | .059118m | .033200m |
| 交接后横向peak | .118410m | .062459m |
| 各自结束时真实参考进度 | .717584m | .706974m |

两臂都实际到达.2m目标区域、通过1/3和2/3位置检查点。候选横向RMS减少43.84%，peak减少47.25%。936个共同前缀物理采样的17字段完全一致，188个完整smoother/PID快照及561项原policy字段比较一致；raw时钟原点相同。18.7秒首次实际请求分叉，来自当时可用的有效速度方向，并非固定时间触发；340个共同窗口内applied tick不同。参考全程逐点相同，改善不是索引提前跳转或只改诊断字段。

原门槛另外要求候选各自结束时的实际最大参考进度不低于B0：候选少.010610m，所以严格保留REJECT，不放宽或回改旧结果。这不等于“机器人物理更慢”：两臂在不同时间进入相同半径目标区域；较大的横向误差使B0必须沿轴走得更远才进入区域。由末段切线、真实终态横向距离和目标距离重构进度，精确解释两臂终态数值。共同25.5秒时，候选真实进度.706974m，B0 .692783m，候选多.014191m；该补充只解释测量口径，不能替换原门槛或倒改REJECT。

## 尚未测到的问题

两臂到达目标后立即结束原采集，尚没有实际停车尾迹、五次低速资格或离开目标区域的数据。不能从“到达.2m”推出已停住，更不能推出整条视觉绕障导航完成。后续有界工作应补这个未测物理时间段，并预先规定共同事件窗口；不能继续换速度方向阈值、滤波网络或PID参数。

八项几何/真实继承接口RED→GREEN；第一GREEN有一处测试把旧自适应lookahead误设为.5m，实际旧方法为1m，保留失败日志与原fixture后修正测试。固定参考/未来来源两项RED→GREEN，整个469项CPU测试通过21.67s。87条旧实测有效tracking行只算几何/同状态BankMap请求：旧协议副轴全数精确重构，71条候选副轴请求会不同；没有二次调用stateful policy或把算术当物理预测。两臂实际原始RGB、来源/CAD、地图、物理、执行状态、视频、版本SHA全量核验。

旧深度拒绝、过滤器局部通过但移动覆盖不足、Mmem1拒绝和全部原导航失败保留。本轮不是E0/E1/E2，没有新的正式导航成功率。独立审查尚未覆盖此跟踪分支。

证据：velocity_fixed_reference_protocol/runner_derivation/status_v1、两actual_run_v1和assessments、comparison_v1、velocity_goal_event_measurement_audit_v1、RED/GREEN/全量测试日志、fixed_pursuit_reference_v1及来源说明。原proto部分继承大任务字段，实际任务以当前actual_goal_xy、实际episode、固定reference及真实请求为准；旧manifest的observation_scheduled_prefix=False因旧variant名判断而失真，两臂实际均有观察前缀，不据此改变运行结果。


Author additional verification 2026-10-07T01:15:52.628431+00:00: full CPU suite 469 passed in23.24s. Actual V1 XY/progress/error plot generated and personally inspected at /home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/velocity_fixed_reference_trajectory_v1.png. This visual retains original REJECT and contains no postgoal measurements. OldprefixSHA 4d5d1d60312c65ddc32838cbe2d6baea6c5e9ad696737cd015195dc2c9b39346


Prefix execution-record scope clarification 2026-10-07T01:17:15.622928+00:00: 188 matching execution records comprise initial None (capture precedes feedback wrapping) and 187 actual PID/smoother snapshots. Initial None is not a physical execution-state measurement; 936 physics samples across17 fields independently remain exactly matched. Reviewer noticed this reporting distinction during first independent V1 review. OldprefixSHA 582b30fbd108b0debec8b43f2753aef62ca9219a17a889c5f1a4846b28c846d1
