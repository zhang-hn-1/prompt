# 当前 roll-PID 响应身份核验与单条必要补测（2026-10-05）

本阶段解决一个具体疑问：过去的倾侧角响应表，是否因为当前 PID 参数不同而已经不能使用？结果是：**在主轴 +0.5 rad/s、期望倾侧 −22.5° 的一个稳定工况，当前实测与旧表一致。** 这条证据否定了“参数不同就使这个单点失效”的说法，但不能认证整张表、短时转移或导航控制。没有调 PID、安装控制候选或报告新导航成功率。整个研究仍未完成。

## 为什么必须补这一条

旧 BankMap v2 的生产者使用 PID `(Kp,Ki,Kd)=(0.8,0.1,0.35)`；当前 endpoint 导航使用 `(0.4,0.1,0.7)`。PID 实现文件字节相同，配置却不同。四份旧运行配置和实际当前 execution snapshot 提供依据，不凭 B0/roll_pid 名称认定系统相同。

当前运行实际加载的是 `experiments/gated_speed_012/roll_pid.py`：execution_snapshot 的路径插入与 Python 模块缓存决定这一身份。它与 hybrid 目录及旧 v2 的 PID 实现 SHA 相同；完整路径与字节身份保存在原始 identity/protocol。旧生产者五项身份核验中四项相同，`benchmark/isaac_runtime.py` 已变；因此不能认证历史整套运行与当前完全相同。

初始排查发现旧 aux-rate 配置为 30°/s、当前为 None。**在补测前已核验 None 的正式 fallback，当前有效值仍为 30°/s，并不是速率不一致。** 保留最初 ledger 假设及前置纠正，不倒改历史。当前有效 inner 配置为 dt=0.02 s、主轴加速度上限 0.5 rad/s²、副轴目标变化率 0.5235987756 rad/s、副轴速度 supervisor 上限 0.3 rad/s。此前当前版本 large_bank 数据主要是主轴 0.3、6 秒倾侧保持，且没有显式启用这项副轴速度上限；缺少本次主轴 0.5、20 秒保持的匹配覆盖。

旧/新初始姿态、历史、资产和 runtime 还存在差异。本次没有只改变增益，因此 **不能把新旧差异归因于增益**，也不算一次增益 A/B。

## 请求到实际运动的链条

原始主轴/副轴编码请求 → 正式 protocol 裁剪 → BankPID 期望倾侧 `0.75 × clip(aux, ±π/6)` → PID 产生副轴关节目标 → 正式 ActionSmoother → 副轴速度 supervisor → 实际关节和 XY 运动。

这里 raw aux=−30° 是倾侧编码请求，期望 bank=−22.5°，不是要求实际副轴关节等于 −30°。bank 用归一化 wxyz 四元数的 `asin(2(wx+yz))` 表示机体 Y 轴在世界 Z 方向上的倾侧，不用普通 Euler roll 混同定义。运动曲率复用原生产者的 world-XY 几何测量，再按主轴符号转换成 body 曲率；没有用 root yaw/速度比替代。实际关节、机体倾侧、XY 速度与曲率分别留存。

没有本轮学习模型或递归物理预测。执行链一致性检查读取当时实测反馈，核验代码能否重建 smoothed/applied；不把这种检查称作无未来真值的物理预测。曲率使用离线居中 31 点、0.02 秒采样平滑，它属于事后标签，不能直接当成在线观测或预测输入。

## 冻结方案与实际运行

冻结设计/计划：`docs/superpowers/specs/2026-10-05-bank-response-design.md` 和 `docs/superpowers/plans/2026-10-05-bank-response.md`。协议于 **15:59:06.919158 UTC** 封存；SHA `005f949f35a280dad5d1b08e5e8640ee28a0156dcf63696fb5e28bf04731fa61`，172 项源码、14217 项输入。旧可变文档作为输入时使用原字节档案，当前进展另外追加，不能覆盖旧 seal。

唯一实际开放平面序列共 32 秒：0–6 秒 raw zero；6–26 秒主轴 +0.5、aux 编码 −30°；26–32 秒 raw zero。开头完整重置一次，内部没有重置物理、smoother 或 PID。seed 3072、既有 0002 起点，安装位置和照明保持冻结。没有 planner 搜索、拟合、第二条序列或阈值扫描。固定总时长是诊断采集，不改正式 benchmark 的原 35 秒规则。

实际进程 **15:59:25.751806–16:01:18.940992 UTC，exit 0**。沿用已知 GPU 运行权限，自动授权批准；本轮没有失败启动或重试。继承的 hidden marker primvar 警告保留在日志，marker 隐藏和图像时钟逐帧检查通过。

| 实际记录 | 数量/结果 |
|---|---:|
| 10 Hz 请求块 | 320（每块含五个 50 Hz 动作） |
| 50 Hz 物理状态，含初态 | 1601 |
| 10 Hz 捕获及 execution JSON，含初态 | 321 |
| 完整实测反馈执行重建 | 1600 ticks，smoothed/applied 精确一致 |
| 双目 RGB/位姿/depth 采样包 | 65，0.5 秒间隔 |
| 双目与第三视角视频 | 各 321 帧，10 fps，无尾帧补分 |
| 接触采样 | 1921，全部 available，实际接触 0 |
| 进程关闭后的产物清单 | 717 项 SHA |

接触 0 只描述这条开放平面诊断，不能称作无碰撞导航成功。RGB 已保存作为采集证据，没有本轮 RGB 深度/估计位姿准入结果。

## 预先指定窗口的结果

只重测一段旧数据：`calibration_0.5` 的 80–84.5 秒；新数据窗口在运行前固定为 21–25.5 秒。没有重新跑旧矩阵、挑最佳窗口或 refit。旧窗口复算与既有 assessment 行精确一致。

稳态四项旧 gate 原样保留：有效比例≥0.9；曲率 IQR≤max(0.2,0.3×|中位曲率|)；bank 95–5 分位跨度≤3°；实际 XY 方向与机体朝向/主轴方向匹配比例≥0.9。测量有效速度下限仍为 0.08 m/s。

| 实测量 | 旧生产者一个窗口 | 当前一个窗口 |
|---|---:|---:|
| 有效样本/总数 | 225/225 | 225/225 |
| XY 速度 m/s | 0.189514935 | 0.187309146 |
| 实际 bank ° | −22.3736636 | −21.8170066 |
| body/XY 几何曲率 1/m | 0.796603322 | 0.792373478 |
| 曲率 IQR | 0.031943798 | 0.021359980 |
| bank 分位跨度 ° | 1.07624848 | 0.60918539 |
| 方向匹配比例 | 1 | 1 |
| 四项稳态 gate | 全通过 | 全通过 |

旧冻结 BankMap 对应请求点曲率 0.796603322，点速度 0.189514935；同档五个角度的层中位速度 0.188549653 是另一个统计量，这里没有混用。对照的请求/冻结值不读取新序列未来实际速度作为预测输入。

| 静态名义比较 | BankMap 对应点 | 原 V1 同命令 |
|---|---:|---:|
| 名义曲率 1/m | 0.796603322 | 0.880051966 |
| 名义速度 m/s | 0.189514935 | 0.214536693 |
| 对当前窗口曲率绝对偏差 | 0.004229844 | 0.087678488 |
| 对当前窗口速度绝对偏差 m/s | 0.002205789 | 0.027227547 |

BankMap 原必要曲率误差限为 max(0.2,0.3×|旧曲率|)=**0.238980997**，新预声明速度差上限 0.03 m/s。当前单点全部通过。**两种名义曲率误差都在原宽容差内，不能把 V1 宣布为违反旧 gate。** BankMap 在这一个窗口偏差更小，不等于接受了新模型或完成全域改进。V1 历史生产者的完整定义仍未认证，静态值比较不代表当前短时预测验证。

## 达到角度、稳定与停稳分别是什么

倾侧达到旧描述标准（实际 bank 距期望≤2°，连续五次 10 Hz）发生在 **8.8 秒**，距请求开始 2.8 秒。这个时刻不表示 XY 曲率已经稳定；图中启动振荡仍持续。稳态结论只对应预指定 21–25.5 秒。

26 秒开始 raw zero 后，世界 XY 速度<0.05 m/s 连续五次、反弹清零的实际停稳资格发生在 **28.5 秒**。6 秒停车尾段峰值速度 **0.170063838 m/s**，相对停车起点的最大径向位移 **0.216794774 m**。旧 0.140884 m 停止包络的拒绝保留，不能用本次通过稳态误差 gate 将它重新认证。停稳资格不是副轴过零，也不是停止距离保证。

![当前响应，绿色为运行前固定的比较窗口](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/bank_response_measurement.png)

第三幅曲率图纵轴显示范围固定为 −5 到 5，仅为可读性裁掉显示范围外的低速瞬态；原始数据和 gate 没有按这一显示范围过滤。曲线属于离线居中几何测量。

## 验证与失败记录

可调用 scaffold 的 RED：8 fail/1 pass，0.17 秒；GREEN：9 pass，0.52 秒；当前 H 全测试 **246 pass，9.73 秒**。包含固定动作时序、独立数组、正反向理想圆弧和短窗口/错误采样间隔拒绝。RED/GREEN/full 日志全部保留，不通过重复全测试制造证据。

作者 verifier v1 首先完成全部 1600 次反馈条件下 smoothed/applied 精确重建，随后把 float64 的 smoother 内部 current_action 直接与实际 float32 applied 比较，产生约 3.2e−9 差异而失败。源码显示正式 `ActionSmoother.step()` 在输出边界转为 float32。v1 源码和原失败日志保留；独立 v2 仅让 snapshot 经过相同 float32 边界，再用**精确数组相等**核验，没有放宽 tolerance、改生产数据或重跑物理。

作者 v2 于 **16:06:30.845731 UTC** 通过：172 源码、14217 输入、717 实际产物身份；请求/时钟/markers/物理/当前状态对齐；1600 ticks；两个视频实际逐帧解码；接触样本和事件。此为作者核验，独立第二人审查另列附录。

证据位于 `experiments/stereo_memory_navigation_20261004_v1/`：`bank_response_protocol.json`、`bank_response_assessment.json`、`bank_response_verification.json`、`bank_response_status.json`、`bank_response_runner.diff`、RED/GREEN/suite/run/assess/verify 日志、测量/runner/assessment/verifier 源码；`bank_response_run/` 保存 requests、physics、execution、history、frames、initial snapshot、闭进程 feedback 与 717 项清单。

[双目视频](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/bank_response_run/stereo_view.mp4) · [第三视角视频](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/bank_response_run/third_person.mp4)

## 分层结论与下一项有界工作

| 层次 | 本阶段结论 |
|---|---|
| 当前控制身份/实测反馈执行链 | 已核验；有效 rate 相同、增益不同；重建通过 |
| 单个前进/负倾侧稳态点 | 通过冻结必要一致性 gate |
| 整张 BankMap、其他主轴/倒车/倾侧点 | 未认证，不扩大单点结论 |
| 动态/无未来反馈物理预测 | 未实现/未准入；M2 拒绝保持 |
| 规划/控制改善 | 本轮无候选，不能报告改善 |
| RGB+估计位姿、学习记忆/世界模型 | 未准入；旧拒绝保持 |
| 正式 35 秒 E0/E1/E2 导航 | 未触发，无新成功率 |

下一项是**一次名义运动 primitive 的请求→响应消费审计与单候选资格实验**：核对当前 planner 的 aux/V1 名义曲率与实际 roll 编码/BankMap 前向响应是否一致，明确可支持请求域和静态/动态边界；仅在现有数据支持时选择一个响应来源调用改动。冻结请求、同一地图/起态、原几何与进度 gate 后做 CPU 资格，不能同时改代价、增益、速度、地图或换向规则，不能救活上一轮已拒绝的曲率 cap/time-cost 候选。一个已测单点不足以认证整个响应域；若缺证据，明确拒绝候选及具体缺口，不能无界补测。

导航证据仍来自先前 privileged depth+known pose 闭环，endpoint 35 秒超时结论不变；它不是 RGB 导航成功。RGB+估计位姿、盲区可信记忆、唯一学习增量及正式 E0/E1/E2 仍须分层完成。当前阶段完成以 final identity/doc-sync/fresh closure checker 为准，**全研究未完成，automation ACTIVE**。报告与所有失败/原始数据保留，不提交无关 Git 变化或删除产物。

## 作者收尾附录（不属于已审正文）

唯一 fresh-context 第二人审查为 gpt-6-astra/high `/root/bank_response_review`；成功派发一次，本阶段没有采用旧上下文 fallback。发现 0 Critical、0 Important、1 Minor，允许接收本次有界诊断。评审本人核验172源码/14217输入/717运行产物/19package产物SHA、原正文11557字节及SHA；个人9fixture通过0.44s，比较当前/旧配置与执行初态、检查图。作者246全测试、1600tick重建、完整视频解码/传感器对齐/事件时间仅为评审读到的作者证据，未独立重跑。评审最初只读命令因python未安装失败后换可用解释器；不算科学失败或新试验。评审不改代码/数据/Git；没有第二次评审。

Minor 的读者澄清：`execution/frame_000.json` 是 null，因为初始capture在PID包装安装前。完整可克隆初态另存 `bank_response_run/initial_execution_after_reset.json`；所以是 **320个完整编号执行记录+1个独立完整初态=321个可用状态**，不是321个编号JSON全都完整。sealed verifier/原数据保持；计数表示与继承runner旧docstring的小项不作代码修饰，没有实施修复或重跑。

实际PID完整路径补充：`/home/an/Documents/ChatGPT/球型机器人导航/experiments/oracle_feasible_route_20260929_v9/gated_speed_012/roll_pid.py`。正文短路径表示该子目录身份；应以实际identity中的完整路径为准。

作者逐条处理评审保留范围（评审原因与作者裁决分别保存在 bank_response_review.json）：

1. Independent attestation of original run count：Author observed original process exit0 and status/manifests; one actual sequence is author evidence, not reviewer eyewitness. 风险：False counting if author evidence wrong; no independent live attestation claimed.

2. Fresh held-out/generalization：Single prescribed diagnostic plus one historical window; no fresh held-out or generalization admission. 风险：Single point could fail elsewhere; full model remains unaccepted.

3. Entire BankMap admission：Other speeds/directions/banks untested; full BankMap not admitted. 风险：Extrapolation would mislead control; no extrapolation deployed.

4. Historical whole-system equivalence：Five audited files4same/1runtimechanged, assets/state notcertified; whole identity not claimed. 风险：Unidentified historical differences prevent causal comparisons.

5. Causal PID gain effect：No gain-only intervention; causal_gain_effect_isolated=false retained. 风险：Misattribution prevented by explicit scope.

6. V1 historical producer identity：Unverified producer remains unverified; current consumer numeric comparator only. 风险：Nominal value semantics cannot authenticate old data.

7. Transient/recursive physics prediction：1600 measured-feedback reconstruction is execution consistency; offline centered geometry is label only. 风险：Model could fail transient; no dynamic admission.

8. New controller/planner effectiveness：No control candidate/search/fit in this stage; no claimed improvement. 风险：No new navigation result can be inferred.

9. Future response-consumer candidate eligibility：Conditional next stage must freeze one candidate/command support/gates before testing, no fullmap transplantation. 风险：Single point cannot license all-domain consumer; reject if unsupported.

10. Global stopping/collision safety：Openplane0contacts not navigation success; old.140884 envelope rejection unchanged/newtail.216795. 风险：Uncertified stopping/collision guarantees remain prohibited.

11. RGB/pose/visual navigation：65 RGB sample bundles are acquisition evidence only; no RGB depth/pose admission. 风险：Privileged geometry cannot masquerade visual success.

12. Blind memory/learned world model：No new fitting/validation; prior refusals retained. 风险：Whole mission remains unfinished.

13. Formal35s E0E1E2：32s diagnostic only; original35s unchanged, no formalrates. 风险：No new benchmark success rate inferred.

14. Fullsuite personal verification：Author246pass9.73s recorded; reviewer personally9pass.44s only. 风险：Evidence attribution must remain separate.

15. Fullvideo/sensor/event personal checks：Author v2 inspected source/log/artifacts, originalfull decoding/reconstruction timings attributed author. 风险：Not independent rerun; identity cannot substitute live process.

16. Later document sync/finalclosure：Author fresh checker required after append; reviewed body exact prefix preserved. 风险：Administrative discrepancy would keep stage unclosed.

17. Live automation：Author freshchecker reads unique actual scheduler status ACTIVE without modifying settings. 风险：If inactive check fails, no claim continued automation.

18. Whole mission completion：stage_complete conditional checker only; whole_research_complete=false. 风险：Finishing diagnostic cannot finish overall navigation research.

用户持续自主授权与原始证据保留要求优先于重复审批、删除实验workspace或无关Gitcommit；没有提交/上传/删除原始资料。本次 stage 的最终身份和文档同步由 `bank_response_final_identity.json`、`bank_response_document_sync.json` 与独立新鲜 `bank_response_final_check.json` 核验，后者实际PASS才关闭本阶段；不再运行本轮科学矩阵。完整使命未完成，automation ACTIVE。
