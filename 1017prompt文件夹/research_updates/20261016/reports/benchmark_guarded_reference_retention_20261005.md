# 有效局部参考保留：唯一35秒真实控制对照

2026-10-05 05:42UTC heartbeat。完成一个参考消费候选和一条实际golden E1导航；归档B0不重跑。**接口通过、预封存有限改善通过、完整导航失败**。全研究未完成、automation ACTIVE；RGB/估计位姿/学习增量/正式E0/E1/E2尚未准入。

## 做了什么与结果

仅改变“何时替换已安装的局部参考”：每原.5s边界用当前观测地图/XY核验旧短段剩余部分和当前位置到参考的桥接；连续与原栅格.6m规则都通过、remaining≥.5m且投影距离≤.5m时，保留原参考与进度状态。否则交回原planner重建，不锁定绕行侧/方向，不要求先停稳，不重置物理/PID/smoother/换向计数。原五次10Hz XY速度<.05m/s和反弹清零完全保留。检查仅在.5s边界，不能说两边界之间已认证安全。

此前“两个切侧事件均可直接保留”被拒绝且保持拒绝；本次是独立带有效性条件的候选，不追改旧screen或.6阈值。地图Unknown原样，是显式风险探索。空hazard不产生地面观测。连续polyline、原格点/nominal primitive及实际动态扫掠层次不同，guard不认证3D/停止安全。

| 同初态、同独立评价参考 | 原B0 | 候选 |
|---|---:|---:|
| 终止 | timeout35s | timeout35s |
| 距目标 m | 4.486646 | 4.156980 |
| 实际路径长度 m | 5.890657 | 6.411888 |
| 独立参考进度 m | 4.167298 | 4.855576 |
| 横向RMS m | .497267 | .174067 |
| 横向peak m | 1.114722 | .475266 |
| 实际接触 | 0 | 0 |
| 2m空间检查点首次通过 | 24.9s | 13.9s |
| 4m空间检查点首次通过 | 34.2s | 25.3s |
| 6/8m检查点 | 未通过 | 未通过 |

检查点来自冻结共同参考对应世界位置、统一.3m实际XY半径，不读取policy索引；时间按10Hz索引显示，实测浮点时钟保留在原数据。进度增加.688277m、目标距离减少.329666m，且RMS/peak均改善；五项预封存有限改善条件（两项各≥.05m、横向各不退步1e-5、无接触）全部通过。不是停车或索引制造提升，实际路径更长且更早到达真实空间位置。但距真实goal .2m原门槛仍很远，所以**拒绝完整导航可用性**，不报新导航成功率/默认替换正式baseline。

![真实对照轨迹](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/guarded_reference_comparison.png)

图为实际两臂轨迹与同一离线评价参考，已实际检查；无未来反事实轨迹标签。全球参考仅离线评价，不入局部planner/来源map。

## 冻结配置、来源和实际运行

完整读取mission304行、主计划263行、ledger407行、最新报告85行，核验前轮77源/2798旧输入/33阶段产物/5文档身份，上次实际run finished/exit0。H=`experiments/stereo_memory_navigation_20261004_v1`；原项目只读，已有H隔离/unborn master无无关Git提交/清理。

新spec/plan `docs/superpowers/{specs,plans}/2026-10-05-guarded-reference-retention*.md`；新策略 `guarded_reference.py` 继承冻结 `DepthLocalCurvePolicy`，内层仍ConfidentSpeed→MappedBankOracle→OraclePathPolicy，原BankMap/roll PID(.4,.1,.7)/lookahead.5、主轴.5/1/1.5调度、aux supervisor.3。局部target2.4m/crop±4m/Hybrid node3000/.3m primitive、ProvenanceGrid、协议/smoother、固定rig X.20/Y±.30/Z.05和照明均不变。6s零请求观察包含在35s内，不额外延时。这是与上轮相同depth+knownpose风险任务域的单因素对照；与旧InformationPolicy .3 cap域不同的历史限制保持。

复制原runtime，仅替换policy/协议/输出名及“参考安装时间”记录，统一差异在 `guarded_reference_runtime_diff.patch`；capture/physics/PID/时钟/监督器没有修改。外层privileged=False只禁止runner全球Oracle context；当前两眼renderer depth和knownpose是明确特权传感层，不叫纯RGB。没有全图/源网格/未来实际角度输入planner，没有body footprint seed、GT过滤或预测free。

预封存 **05:51:05.605849UTC，86源/2834旧输入身份**；唯一实际run **05:51:21.407818—05:54:11.093362UTC**，exit0。350动作/351双目capture/1751计分physics；2101可用接触记录全部无实际接触；双目与第三视角视频各351帧10fps，无旧5尾帧。没有第二条仿真、基线重跑、训练/立体网络推理或新标定矩阵。原有old-gates未改变。

58次边界检查：**50次保留、8次重建**（6成功、2失败）。拒绝原因计数remaining不足2、原栅格不足3、连续距离不足1，后两项可同一事件重叠。最长参考age8.9s；每次保留前后索引/方向/低速/失跟/失败/映射计数都一致，之后正常tracker act照常更新。非边界帧明确guard_measured_this_call=false且保留原测量时间；不把过去guard当当前反馈。重建/失败均原.5s节奏，25s失败后不偷偷继续旧无效plan。

## 执行链与因果证据

第一次参考保留6.5s，首raw请求分叉6.9s/frame69。分叉前346physics采样的17非raw字段（含raw_command_target本身）逐值相同、69请求/70完整PID+smoother状态/70双目来源packet与history一致。raw时钟初始化相对B0 **-.02s/-1tick**；不称原始时钟完全同源。物理相对时钟/初态/前缀相同。249raw请求块、1393 applied物理tick真正改变，不是将raw±35提前写成±30的行为等价。

作者 **05:55:08.554643UTC** 完整复核86源+2752关闭后产物：351当前两眼depth packet重新构造、350来源map/局部参考/真实请求/diagnostics/mapping逐值重建，history移位350次且每次仅一次。3个有限过去状态checkpoint（起/中/末）future payload抛错替换与截断等价；不是3条fresh物理验证。所有1750物理raw请求与实际action对齐；以实测反馈重建1745tick smoothed/applied及完整状态，首5tick未克隆，**不是递归物理预测**。摄像内容事务/原mask与零age/hidden markers检查保留，不冒称全表面曝光认证。源RGB拒绝不解除。

协议裁剪在224有效运动tracking块为54.464%（B0 265块70.189%）；V1请求超支持比例45.089%（B0 33.585%），后者没有改善。合法目标到角界与实际关节跟不上仍不同层次。policy p50 .00254/p95 .00804s，实际重建planner p50 .5801/p95 1.8413s，capture partial p95 .3049s；减少重规划有计算收益，但**因果在线≠实际10Hz墙钟实时**。

## 为什么还没走到目标

实际请求来源分段保存在 `guarded_reference_failure_segments.json`，不预认定首次交接是唯一主因。

| 实际阶段 | 时间 | 沿冻结评价参考进度变化 m | 事实 |
|---|---:|---:|---|
| 初始正常前向tracking（夹本地段末制动） | 6—17.5s | +3.481约 | 已到绕障前半段；2/4m检查点明显提前 |
| 第一次换向制动 | 17.5—19.5s | +.239171 | 合法五次低速确认后才reverse，未改控制 |
| 实际倒车tracking | 19.5—21.2s | -.158711 | reverse请求-.5，17运动帧94.118%裁剪 |
| 转回前向制动 | 21.2—23s | -.158343 | 第二个原路线方向段，五次资格后forward |
| 重新前向tracking | 23—25s | +.212494 | 25s旧短段连续/栅格失效 |
| 没有局部路径 | 25—26s | +.122249 | 仅余惯性位移；不称成功运动 |
| 后续同方向tracking | 26—35s | +1.084178 | 仍未通过6m空间点，末评价误差.475266m |

前段显示真实进度改善，17.5—23s是实际reverse→forward完整局部maneuver；它包含3.8s制动和倒车退进度，不能说控制器没有换向或延迟是bug。Planner初始状态同时免费播种forward/reverse根，`local_curve`只传XY/yaw/goal，未传当前接受的执行方向；这是值得下一项核验的**名义规划初态与执行交接接口差异**，本轮没有对它做反事实控制干预，不证明补方向就能导航成功。原reverse multiplier1.25和方向切换代价.5m也不等于实测制动时间。

末段26—35s只使用main+.5，90实际运动帧平均XY速度.171505m/s，83.333%协议角裁剪。对它真正安装并持续9s的旧局部曲线，真实polyline进度+1.397859m、横向RMS.045075/peak.102368；距离共同评价参考则逐渐增大。因此不能把末共同参考偏差全部归PID没跟上，也不能用每次换参考的零误差掩盖；路线本身/运动域和35s进度预算仍是问题。6—17.5有效运动均速.276634、裁剪11.34%；不把末段低速当所有状态硬物理上限。含回折的局部polyline最近投影会跨邻近段，描述量不是段方向完成证明；实际换向和全球检查点独立记录。

## 测试、材料与接续范围

7项缺功能RED＋1原cadence回归→**162全H CPU测试通过/6.40s**。覆盖retain不reset索引/latch、hazard/bridge/投影/耗尽rebuild、原失败重试节奏、五低速及反弹、诊断当前/过去标记。首次GREEN7pass/1fixture失败：桥接hazard使原planner无法重建，测试仍期待“已安装时间”；改为有plan才记录安装时间，**仅测试在封存/数据前修正**，原失败日志保留，没有调控制或gate。

实际代码/完整数据/视频/图/协议/作者核验/失败日志全部在H及 `guarded_reference_run/`，科学结果的assessment与verification独立保存，前轮全部身份不倒改。唯一只读独立审查在最终收尾前完成；作者核验范围不冒称审查者全部运行。

本候选在同一开发初态的有限控制改善可以保留为局部诊断分支，完整导航仍拒绝且正式baseline不自动替换。下一项只核验**planner根方向与当前执行方向的交接**：先用本次17.5s已保存输入做一个有界同目标几何对照，考察把原.5m方向切换代价用于实际当前方向起点是否真正改变已选择maneuver；允许必要倒车、不强制forward，不动增益/速度/参考目标/地图/rig，不扫描penalty、不展开新全矩阵。该新接口需独立版本和因果测试；没有行为/可行性证据不新增物理run。既有guard和原低速/反弹/PID不改。不得把补初始方向当动态模型已通过或保证35s成功。

RGB可靠障碍来源/估计位姿/唯一学习增量/原35秒E0/E1/E2仍按总计划待分层验证；本轮没有新世界模型或正式成功率，全研究未完成、ACTIVE。

## 独立审查与作者收尾

一次 fresh-context 只读审查完成，Critical/Important/Minor 均为0。审查原正文SHA `a5f21c0d1f11dad7b9ec56bd20ad2c556bd7abf7208ac7267f9857dfc4540202` 保持原字节；此作者追加段不是第二次审查。全文审查与逐条范围裁决见 `guarded_reference_review.json`，审查“ready to merge”仅作为诊断阶段交付意见，没有Git合并/提交，也不代表导航基线验收。

审查人亲自核验86源/2834旧输入/2752关闭后产物SHA、全部58交接/55非空guard(独立连续段标量误差<=1e-10，栅格复用冻结helper，未逐点独立暴力复算)、69请求/346physics17非raw字段/70完整execution+source/history前缀、全部1750真实command targets、249请求和1393applied差异、真实进度/横向误差/空间检查点/最后installed窗口。实际完整解码两视频各351帧10fps；另亲自运行6个几何helper测试0.26s。162全H测试、350因果policy重建/351双方packet/350historyshift/3finite checkpoint futuretrap及1745执行链重建是检查过的作者证据，没有把它们记作审查人亲自重跑；审查0search/0control replay/0sim/0inference/0文件变更。

九项declined范围逐条接受：正式/泛化导航仍timeout，RGB/estimatedpose/学习未测试，动态/3D/unknown安全及边界间安全未认证，折返polyline投影不是direction-aware完成证明，raw起点offset如实保留，余下timeout唯一因果和root改动收益未证明，wall10Hz未证明，审查后作者封存另核验。均限制当前结论，不产生被掩盖的Critical/Important问题。原科学源、报告正文与所有旧gate不改；接口与事前有限改善通过，真实35s导航拒绝。随后作者仅封存并同步mission/mainplan/ledger、核验全部现有身份；不增加第二条物理运行或下一阶段root搜索。研究未完成，automation ACTIVE。
