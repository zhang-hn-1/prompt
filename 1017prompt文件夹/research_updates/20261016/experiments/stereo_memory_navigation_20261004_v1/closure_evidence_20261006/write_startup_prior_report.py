from pathlib import Path
import json
W=Path('/home/an/Documents/ChatGPT/球型机器人导航');H=W/'experiments/stereo_memory_navigation_20261004_v1';a=json.loads((H/'startup_prior_runs/assessment.json').read_text());s=json.loads((H/'startup_prior_status.json').read_text())
f=W/'reports/benchmark_perfect_visible_depth_and_startup_prior_20261006.md'
f.write_text('''# 完美双目可见深度与独立安全起步先验（2026-10-06）

结论：算法深度错误和可见近场支持不足是两个独立问题。在这条既有完整序列、冻结双目域和端点表示下，即使当前可见深度完美，严格起步 footprint 仍全未知。单独提供初始185格的 task prior 只在起步时满足2.5D必要条件，仍不足以支撑整条运动。此条件诊断拒绝；无新导航成功率。不能推导固定相机在所有表示/方法下永久无法导航。

## 预先冻结的问题和对照

上一阶段两模型均拒绝，不继续换网络、fit、调阈值或做Oracle矩阵。原主计划允许分别标注 safe-start prior；本次仅问“完美可见来源 + 初始安全区域”是否就能解决缺支持，不改旧 coldstart 或 90/99/.1m gate。完整设计 docs/superpowers/specs/2026-10-06-startup-prior-separation-design.md 与 plan 同名 startup-prior-separation.md 在运行前封存。

复用 E2 yaw−20 完整65帧32秒、321capture/1601physics，已用于设计的旧数据，非fresh。本阶段0新采集/physics/infer/fit/action。实际consumer START→END UTC退出0；全部65map与packet先落盘，再由独立assessor读取真实未来6..32s轨迹。后者仅离线空间诊断，不是consumer输入或在线可预测计划。

## 显式测量与先验隔离

“完美来源”是 privileged 当前renderer axialdepth/current knownpose：stride2、.5..3m、固定机体mask、点距root>=.55m；投影到当前右眼实际pose，右眼FOV/mask/finite depth与同表面容差 .03+.01z。使用真实右depth证明可见性，不能称双目RGB估计器。sigma=0只是理想测量假设，root/pose是真值诊断层。每点真实左/右像素与frame/time保存。

PointSupportMap旧SHA原样：world floor0、.05m带、hazard .05<Z<=1.5优先，TTL10s，不插值/raycarve/扩点/脚下seed。161×161/.1m、footprint .7m加半格对角沿用旧定义，未缩小救分。没有读取完整occupancy或以真实floor标签在线过滤深度；worldfloor0是显式假设。

独立 ImmutableStartupPrior 明确 source=task_assumption/trusted=True/is_observation=False，仅t6初始185格，静态不随robot走、不扩大、t6前禁用。prior不进入observed labels/provenance/timestamps/records；它只能单独计入必要support，hazard仍优先拒绝。没有认证完整3D或发动作。

## 全序列实测结果

|指标|结果|
|---|---:|
|理想可见端点来源记录|110186|
|实际floor端点来源记录|225|
|history峰值floor格|18|
|整条真实运动扫掠格|498|
|初始固定prior格|185|
|扫掠在prior外格|313|
|strict当前disk必要支持通过|0/65|
|prior启用后当前disk必要支持通过|1/53|
|strict整条sweep通过|0/65|
|prior启用后整条sweep通过|0/53|
|prior外313格最少未知|299|

225是跨帧来源记录，18是同时存活floor格，不是面积、独立实体数或自由体积。原两来源出错不是唯一问题：这里无网络对应误差，仍有95.53% prior外扫掠格未知。这个比率限于冻结当前双目共同可见端点、.5..3域、TTL、栅格和本次轨迹，不是所有视野的信息理论上限。

6s首次运动前：strict footprint185 unknown/0floor/0hazard；prior臂0unknown/185prior/0observed。整条sweep严格497unknown/1floor；prior臂312unknown/185prior/1floor。受控先验确实改变了必要支持计算，但没有产生任何新观测。之后仅一个当前disk query通过也受到栅格和固定region边界影响，不能解释为“只允许物理移动0米”或一个真实碰撞定理；本次representation/gate不支持授权。

真实未来轨迹只为后置标签，不能据此挑prior覆盖整条路或在线构造动作。未尝试扩大prior、改变可见域/TTL/容差/格或换mapper。本阶段不试受控动作，因为必要支持条件已失败且原RGB来源模型仍REJECT。

## 实际验证及产物

4prior literalRED失败/.15s→GREEN4/.13s：明确trust/type、t6时钟、prior不写observed且hazard优先、copy immutable不跟robot。首次 missingmodule collectionerror单独保留，不冒充literalRED。3perfect投影literalRED/.15→GREEN3/.18：手算native右伙伴/真实floor、右同表面与mask、当前白名单/NaNclock。

9source/66input（65原raw帧+mask）运行前SHA封存。作者完整重建65packet/65historymap逐值相同，270SHA、3actual timestamp前缀未来payload替换/截断检查通过；原prior层从未修改observed数据。完整H 361passed/10.81s。新posthocassessor/plot/verify不冒称采集或消费前已封存。

H/startup_prior_protocol.json、status.json、runs/manifest.json、immutable_prior.npz、全65packet/map/frame、assessment.json、offline_actual_sweep.npz、verify/RED/GREEN/suite日志保留。startup_prior_actual_maps.png与full_series.png作者已实际查看：蓝点明确taskprior，灰点是离线轨迹所需格，绿点是privileged visible floor，不画成全空间free。0新视频，原真实采集两视频仍保留。

## 分层裁决与必须保留的范围

测量反事实：当前完美renderer双目可见端点定义实现和因果重建有证据，不是视觉算法成绩。
起步先验接口：显式隔离有证据；它能覆盖初始disk的必要条件，但不能认证3D。
条件支持：strict与仅初始安全区的整条运动均REJECT。
真实RGB/估计位姿/学习记忆/正式E0E1E2：未触发，原拒绝和原35秒规则保持，没有新成功率。

这次关闭完美端点/初始prior诊断，不扩大同类矩阵。现有严格unknown-blocked消费者无法给出有证据的RGB下一步；原主方案的安全起步分支也不能由这最小prior自动解锁。若要改变任务条件，必须明确新增信息来源或另定义受控探索/地面假设任务，不能默默放宽当前验收。

尚不能声明任何替代方法都无解，也不能把一个端点地图表示的失败当传感器物理不可导航。下一接续应先整理既有正/负结果、可用基线与真实剩余门槛，让条件选择依据具体证据；不再自动换网/调参救分。整个研究未完成。本次终审及作者裁决独立追加，原正文保留。
'''.replace('START',s['start_utc']).replace('END',s['end_utc']))
with (H/'startup_prior_ledger.md').open('a') as out:out.write('\nTask2 ONE65frame perfectvisible consumer '+s['start_utc']+'→'+s['end_utc']+'exit0;110186records/225floor/18peakcells. Task3 AFTERmanifest offline498sweep185prior313outside;best299unknownoutside;priorcurrentpass1/53 full0/53, strict0/65. Author270SHA/65source/maprebuild/3actualfuturetraps PASS,361suite10.81s, both PNG personallyviewed. ScientificconditionREJECT/wholeNOTcomplete. ONEreadonlyreviewpending.\n')
print(f)
