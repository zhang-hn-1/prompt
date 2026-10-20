from pathlib import Path
import json
W=Path('/home/an/Documents/ChatGPT/球型机器人导航');H=W/'experiments/stereo_memory_navigation_20261004_v1'
a=json.loads((H/'dense_source_memory_runs/assessment.json').read_text());s=json.loads((H/'dense_source_memory_status.json').read_text());v=json.loads((H/'dense_source_memory_author_verification.json').read_text())
body='''# 稠密双目来源的空间记忆与起步支持验证（2026-10-06）

结论：两种来源均拒绝，记忆消费也不满足起步支持条件。适配后的深度产生更多端点，却把错误保留进了空间记忆。不能进入 RGB 控制、估计位姿闭环或学习记忆训练。本阶段是完整既有序列的因果回放诊断，没有新在线运行、导航成功率或新 held-out。

## 问题、对照和预先冻结条件

问题是已拒绝的 B0/adapter 深度是否仍有可用的任务空间支持。复用 E2 yaw−20 的 32 秒实际采集：65 双目对、321 capture、1601 physics；本阶段没有采集、推理、训练或控制动作。比较同样 65 帧的原网络/单次适配网络，各自 current-only/history 两臂，共 260 张地图。序列已用于上一阶段设计分析，不能再称 fresh。

设计与执行计划分别为 docs/superpowers/specs/2026-10-06-dense-source-memory-consumer-design.md 和 docs/superpowers/plans/2026-10-06-dense-source-memory-consumer.md。来源原门槛与拒绝不变。必要的 2.5D 支持要求当前 footprint 和冻结真实整条运动扫掠格 unknown=0、hazard=0；即使满足，也不等于完整 3D 自由空间或控制准入。 floor 端点要求错误代理为零，只是诊断门槛，不是安全保证。

消费者仅读取 depth/confidence/正 native disparity、frame/time、当前 camera/root knownpose 与固定机体 mask。世界地面 Z=0 是明确的平面假设；knownpose_sigma=0 是仿真真值位姿层，不是估计位姿。native 右伙伴 v=y−D；USD optical diag(1,−1,−1)。stride2，深度 .5..3m，D>1、有限置信度、双目 FOV/mask 有效，到当前 root>=.55m。每条来源保留左/右像素、frame 与时间。±1px 深度敏感性不是实测误差界或对应正确概率。

未改变 PointSupportMap：10s TTL，不刷新旧来源，不插值、不 ray carve、不 seed 脚下、不膨胀端点或把未知填 free；hazard 优先于 floor。161×161、.1m 栅格，原点初始 root XY−8，不读取 occupancy 内容。floor 资格 abs(Z)+sigma<=.05 且 sigma<=.05；hazard .05<Z<=1.5。 footprint .7m 加半格对角沿用旧定义，未缩小救分。每个 query 的 current disk 只用当前实测 root；实际未来 6..32s 轨迹仅由独立后置评估读取，不是运行时计划。

## 实际执行与隔离

9 个 source 和 196 个 input SHA 在消费前封存（130 深度输出、65 净化 knownpose、固定 mask）。一次消费者执行 START至END，保存 130 端点包、260 maps、130 行摘要。整份 manifest 保存后才解码 GT endpoint 标签与未来实际轨迹。所有行 certified_free/action_authorized=false，model_imputed_regions=0，source_model_admitted=false。回放代码并没有证明部署到正式 runtime 的在线接口。

## 实际测量

| 指标 | 原 B0 | 单次 adapter |
|---|---:|---:|
| 全序列端点记录 | 29409 | 40363 |
| 估计 floor 端点（重复来源记录计数） | 70 | 208 |
| 有 GT 标签的 floor 记录 | 70 | 208 |
| 实际不在 floor 高度带 | 70 | 172 |
| 3D 误差 >.1m | 70 | 188 |
| 同时满足实际 floor 且 <=.1m | 0 | 20 |
| history 峰值 floor 格 | 4 | 17 |
| 峰值纯历史 floor 格 | 4 | 16 |
| 最少未知整条扫掠格 /498 | 492 | 485 |
| history 当前 footprint 必要支持通过 | 0/65 | 0/65 |
| history 整条扫掠必要支持通过 | 0/65 | 0/65 |

wrong-height 与 wrong-3D 可重叠，不能相加当总错误。它们是 source 记录计数，不是独立物理点或碰撞次数。current-only 两项也全部 0/65。错误诊断沿真实 renderer 轴向深度与当前 camera pose 反投影；保留 renderer/曝光及像素采样标签局限。

在 6s、首次运动前，两模型两臂均 185/185 当前 footprint 格未知，floor/hazard=0。整条未来实际扫掠的离线 498 格在此刻：B0 两臂 497 unknown/0 hazard/1 floor；adapter current 同样 497/0/1，history 495 unknown/1 hazard/2 floor。少数远处 floor 代理不能认证近处起步，也不能从后来运动没有碰撞反推当时已观测 free。全时段最佳 history 也分别还有 98.80% 与 97.39% 扫掠格未知。

## 验证与产物

5 个 literal 测试先 RED 5 fail/.17s，再 GREEN 5 pass/.13s。覆盖原生伙伴方向/投影/敏感性、conf/mask、GT schema 拒绝和无效时钟。完整 H 测试 354 passed/10.84s；本次作者核验 595 SHA，原样重建全部 260 maps 的 labels/provenance/floor/hazard times，6 个实际前缀未来 payload 替换/截断检查（两模型 t0/6/16），没有读取未来 payload。数据与模型输出未变。

首 verification-only 程序直接把 NPZ 0D frame/time 当 tuple key，抛 unhashable ndarray。原日志源码保留；独立 v2 只反序列化成 int/float 后复原实际运行的类型，不改消费者、数据或科学口径。未重复科学实验。

产物：H/dense_source_memory_protocol.json、dense_source_memory_runs/manifest.json、assessment.json、offline_actual_sweep.npz、两模型全部 packet/maps/endpoint_labels/frame 文件、dense_source_memory_author_verification.json、complete_suite.log 与 ledger。当前两张实际图已由作者查看：dense_source_memory_actual_maps.png 与 dense_source_memory_full_series.png。图中红线是离线实际轨迹，不是在线已知轨迹；空白为 unknown。此阶段无新视频，原实际双目采集视频仍保留在 dense_adapter_heldout_run。

## 分层裁决及接续

来源模型验证：原 B0 与 adapter 均 REJECT，旧 90%/99%/.1m 门槛未改。
空间消费者：身份与因果前缀重建有证据；必要支持 REJECT，误 floor 污染存在。
在线接口：本阶段没有执行；不能把回放称为在线验证。
控制与正式导航：未触发，无新成功率。原 Oracle 30.7s 只作历史控制回归；复杂 IMU 3D 拒绝、privileged-depth 局部规划 35s 超时均保留。

关闭这唯一消费诊断，不改 TTL、半径、置信阈值或换 mapper，不训练第二适配模型。下一项独立工作按原方案核验安全起步假设与“完美当前观测”支持：使用明确标注的 privileged renderer-depth 诊断分离算法错误与几何盲区，并保持 task-prior/observed 标签隔离。若完美来源也不支持起步，不能用继续训练挽救未观测信息；报告具体额外信息需求。整体研究尚未完成，自动接续保持 ACTIVE。

第二人审查与作者裁决将在独立附录追加；此正文不追改旧报告。
'''.replace('START',s['start_utc']).replace('END',s['end_utc'])
f=W/'reports/benchmark_dense_source_memory_and_startup_support_20261006.md';f.write_text(body)
with (H/'dense_source_memory_ledger.md').open('a') as f:f.write('\nTask2/3 actual replay finished '+s['start_utc']+' → '+s['end_utc']+';130packets/260maps saved before labels;source and support REJECT. Author595identity/260rebuild/6actualfuture replacement+truncation checks PASS. Whole H354passed10.84s. Verification-only V1 unhashable deserialization failure retained; V2 int/float only. Both actual PNG personally viewed. Report prepared; ONE finalreview pending; mission NOTcomplete.\n')
print(str(f))
