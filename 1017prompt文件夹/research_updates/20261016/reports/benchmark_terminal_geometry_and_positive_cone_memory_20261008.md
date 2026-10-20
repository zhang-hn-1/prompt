# 终点几何、视觉可能区域与对照输入契约

研究仍未完成。前一条60秒终点制动运行没有到达，严格A/B因初始视觉来源不同而拒绝；本阶段没有新增物理运行、导航成功率或正式35秒E0/E1/E2。保留全部历史拒绝，不改相机、光照、PID、速度、规划阈值与验收标准。

前一条实际运行及视频：[完整结果](benchmark_terminal_bank_actual_and_source_variability_20261008.md)。本阶段解决的是“是否存在可直接采用的终点修正”和“多次看见物体是否已经足够定位”的证据问题，尚未解决稳健到达与移动视觉几何。

## 终点参考与真正目标

复用已逐次重建且460请求完全一致的事实链，不再次调用有状态policy。120个终点附近采样的参考终点距已知粗目标0.048178–0.197629米；115个正常跟踪采样中，原lookahead与直接目标的距离差中位数0.431329米。555/556帧剩余局部参考0.299231/0.322543米，粗目标还距0.423356/0.400296米。制动基于局部参考长度，并不自动意味着代码错误。

唯一新几何候选只在原lookahead已取最后waypoint、剩余参考方向全部已接受、终点仍满足原0.2米规划容差且粗目标在当前接受方向前方时，将追踪点换为粗目标。参考、方向锁存、其它控制规则都不改。36个改变的事实状态全部超出冻结稳态端点包络[-0.827225, +0.796603] m⁻¹；首次527帧从-0.648150变为-1.074743 m⁻¹，距目标0.925463米。**候选在GPU前拒绝**，没有缩小容差、插值混合或调增益救分。此包络只是必要覆盖检查，不能称机器人的绝对转弯硬上限；这些逐点事实状态也不是候选未来轨迹。

证据：`terminal_reference_goal_gap_v1`、`terminal_known_goal_target_preflight_v1`的actor-side数据、输入身份、预真值封存与assessment；几何helper四项CPU测试通过。初始RED是缺失模块的收集失败，不能称行为测试先失败。

## 不依赖错误深度的“可能区域”能否有用

固定相机29个旧开发曝光，尝试一个正样本投影锥模型。仅使用当前及过去右目正样本、native估计曝光位姿、标定和CAD自遮挡。每条射线保留0.5–3米深度可能性，在0.1米网格中要求两个不同曝光且相机中心间距至少0.1米支持。ROI只取当时已知相机位置范围，地面以上0.05–1.45米仅为分析高度窗。未来位置没有用于确定早期网格。负样本、自遮挡、视野外仍未知，不写自由。

输出明确为 **POSITIVE_CONE_MODEL_POSSIBILITY**，不是双目匹配点、实测占据、已知障碍高度或完整3D安全。全部坐标、曝光、支持像素与源身份先封存，再使用物理障碍标签评估。没有读取NN深度或真实几何构造输出；没有新网络、拟合、参数扫描或在线准入。

| 指标 | 实际结果 | 含义 |
|---|---:|---|
| 所有曝光的可能voxel记录 | 45,456 | 跨时刻重复计数，不是唯一地图面积 |
| 最后时刻可能voxel | 1,889 | 此时网格中的独立中心 |
| 全部记录距物理障碍体积≤10厘米 | 25.165% | 未达到预冻结95% |
| 体积距离p90 | 0.95米 | 可能区域过宽 |
| 原185个障碍XY格覆盖 | 100% | 用既有0.2米MODEL不确定范围，覆盖不代表精度 |
| 最后时刻体积外voxel | 1,424 | 不能直接当可靠占据地图 |

**结构在接入actor前拒绝。** 最后1,889个候选中，1,887个确有两次基线足够、真实命中障碍的支持像素；体积外1,424个中1,422个也满足这一点。这说明错误不仅是颜色误识别：同一物体在多张图里出现，仍不等于射线上任意候选距离成立。可能voxel与对应真实首个表面命中距离中位数1.029529米、p90 1.566918米。这些事后真值只做诊断，没有用来删点、改模型或写自由区域。

![实际可能区域及高度分布](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/positive_cone_memory_actual_postseal_v1.png)

上图已个人查看。最终XY候选延伸到物体前后约一米多，单靠“至少两次看见”无法准确界定障碍范围。证据：`positive_cone_memory_applicability_v1`，预真值seal SHA256 `9e19fd8e48bdebc48f4b189d01c098e5bad9789d069a216c5d2e0a4691f3a63a`；三个CPU测试通过，包含未来payload不可读取、自遮挡/负样本不写自由和基线不足拒绝。

## 公平控制对照的来源接口

另立历史MODEL提供接口：旧seed原始字节/SHA及序列时间命名空间保留，几何逐项不变；当前时刻只是MODEL安装/形成时刻，不伪装成新的相机曝光。source时间在当前序列标记为过去-1并保留原时钟原值和prior sequence身份；不存在新来源准入、当前占据测量或自由空间声明。其用途仅是隔离控制因素，不能代替原严格视觉任务。这个接口不证明MODEL在其它场景保持有效。

两个独立policy在相同事实运动状态、相同提供MODEL与粗目标下，分别读不同的已录RGB内容，检查请求、参考、地图与锁存。V1完成460次但省略了前140步mapper更新，故只证明两actor之间的相等；原文件保留，并在独立scope supplement限制结论。V2补齐前缀并逐字段核验原实际地图；最终数值见独立assessment，不能把重建称在线物理A/B。

682项阶段回归通过，3条警告，47.67秒（在最后MODEL接口加入前）；MODEL接口另有两项测试通过。未宣称完整视频观看、未重新运行已闭合的物理矩阵。

下一项需要先完成这个共同来源契约的核验，再冻结有明确输入/参考身份的下一实际实验。终点几何候选、正样本锥、之前深度/时序匹配/接地解释及旧M2拒绝都保持；不再循环尝试这些结构，也不把更多模型可能性当已观测地图。原严格视觉导航、学习增量与正式35秒评测仍有未通过的前提。


V2实际核验完成：两个独立policy各460次调用；460帧RGB内容均不同，在给定相同MODEL与当前运动状态下，全部460请求、完整当前参考、方向锁存与5个地图字段逐项相等，且请求/参考/地图也与原实际B0记录一致。完整前140步mapper更新已补齐。只证明当前消费者对这两种RGB内容的独立性；给定MODEL隔离了图像→初始化地图的通路，不表示图像来源已可靠，不是在线或物理对照。


### 2026-10-07T22:17:50.634124+00:00 — Provided MODEL actual CLOSED: pose pass, strict physical identity rejected; bounded sensor attribution closed
ONE actual 60s B0, session96178 EXIT0/257.9358 wall, 600requests/3001PHY/601captures/zeroNN/two601-frame videos; no active process or repeat. Protocol fb5a3134806910ea4139fa38898524c4836f6daa2bfe9f618917965b1f3139ad/postseal c4df9f77bde7e91f9b878ccb26927eb9a73a0871718f649e6082b8aa0adc7c09. SensorV2 original gate PASS_POSE_ONLY RMS.01490826/peak.03530539/anglep90.85140258; 5155sealed artifacts; 2995 execution-clone ticks exact. Original fullCPU684passed3warnings48.75; first scoped review C0/I0/M1 resolved metadata only, no rereview.
Canonical actualV4 REJECT_PHYSICAL_PROVIDED_MODEL_B0_IDENTITY: all2300 five-map-field checks and given geometry exact; first native-pose difference0, reference140, request146, rawcommand731, applied/smooth/velocity748, attitude/joints750, position752. Diagnostic trueXY closest.23614857/final.51138017, timeout60/c0; no nav/source/control admission. Author administrative verifier repairs kept old bytes/logs: inherited R0/R1 writer→B0 writer V2; constructor es0None; later supervisor blocked flag does not imply zero policy request; V4 reconstructs every initialized frame. Supplement clarifies blocked_frames=1 is startup omission, no gate/actual rerun.
One pre-frozen posthoc 748-packet common physical prefix audit: eight physical/applied fields, initial gauge/normal and relativeclock/encoder EXACT; absolute SDK raw-clock origin differs one .02s step. Gyro differs ONLY packet0 by9.16897e-8rad/s; acc420packets/max.199163m/s², gravity eligibility743 each identical. Substituting ONLY gyro0 matches full gyro-channel probe but does not reconstruct all historical states; accel-only also notexact, encoder-only no effect. Both measured estimator replays exact; this localizes divergence to native measurement history before physical divergence, not mapper or learner, but does not uniquely prove SDK implementation/timestamp bug. No packet beyond747, future truth, actor repair or alternative trajectory labels. Old strict identity reject kept; no tolerance/rounding rescue.
Next actual work returns to perception: design one opposite-lean reverse-bank observation coverage probe on same fixed rig/nativepose/source, conditional eligibility and .7 safe-start postcheck frozen, not reverse turning model/planner calibration. Must freeze/review before any actual. Whole/learning/original35 incomplete.
