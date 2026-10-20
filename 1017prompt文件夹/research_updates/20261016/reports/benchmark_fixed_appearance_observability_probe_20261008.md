# 独立外观域中的双目可观测性诊断（2026-10-08）

唯一 14 秒实测已完成。在物理与执行状态完全一致时，新外观材料使原双目网络的部分表面深度数值门槛通过；严格局部 NCC 匹配仍太稀疏，盲区体积尚未通过。不能改写原均匀障碍来源拒绝或宣布导航完成。

## 干预与身份

新 fixed_appearance_fixture_v1 保留同一个半径 .35 m、高 1 m 的解析 Cylinder、位置与 PhysicsCollisionAPI、原 occupancy 字节。原 fixture 原样保存。只在已有 marker hook 中绑定一个固定中性灰随机纹理 OmniPBR 材料；未新增照明、几何、物理步骤或 reset。hook 在 world warmup 后、原最后 articulation reset/pose restore 前执行。相机安装、灯配置、姿态/当前传感器、观察策略、PID、protocol/smoother、冻结网络、色类及 NCC 参数均不改。

该材料包含 shader、反射属性和平均反照率变化，因此是**外观干预**，不能独立证明纹理是唯一原因。新条件结果与原 benchmark 分开；没有上传、更改原资产或正式成功率。texture seed20261008、173..255 灰度/512²/projectUVW/scale2、diffuse .18、albedo brightness .27、roughness .5、metallic0、无 emission/displacement 均在新渲染前固定。SDK shader 源码核验避免把常数颜色误当纹理乘数；无参数扫描。

CPU 属性/唯一目标/纹理重现/启动前延迟 USD 导入 4 测试通过（.53 s），driver 编译通过。首次独立 review C0/I0/M0 launch blocker；review 的标准解释器缺 pxr 提醒作者发现启动前 import 风险，在 GPU 前改为 hook 内 lazy import，新增启动环境回归，不安装包或重跑科学实验。

## 实际运行与独立重建

01:10:53.89—01:12:56.09 UTC，122.1985 s 墙钟，退出0。140 请求/701物理/141 capture/29实际 NN，双视频均完整解码141帧。700实测反馈 PID/protocol/smoother/supervisor tick 精确重建，841 available接触样本0。当前 native gyro/acc/encoder 的估计位姿 RMS .0032755 m、peak .0049493 m、角误差 p90 .6953°通过原门槛，保留初始位姿/平地与低层理想IMU特权，不是纯RGB位姿。

与旧 native_live_point_memory_shadow 的 9 类物理/执行字段及全部请求，整条701采样精确相同；29次左目 GT 深度与曝光姿态也精确相同。绑定前后 collision属性逐项不变。当前RGB改变，实际纹理已出现。

预先指定 frame120、同一离线物体内区4084像素：7×7局部标准差中位1.0267→3.1768，std≥2比例5.71%→85.87%；平均灰度48.42→45.46、冻结色类覆盖96.16%→77.86%。这些 GT 掩膜仅离线分析材质效果，未进入视觉源或动作；再次说明不是只改变纹理而维持所有光度量不变。

![旧左目、新左目、新右目；frame120](../experiments/stereo_memory_navigation_20261004_v1/fixed_appearance_render_refined_source_v2/frame120_uniform_left_new_left_right.png)

## 来源、记忆与覆盖分别验收

| 层 | 原网络部分表面 | 原局部 NCC 精化 |
|---|---:|---:|
| 全部记录 | 10047 | 148 |
| 10 cm 内比例 | 99.9403% | 99.3243% |
| 无过滤 p90（m） | 0.01815065 | 0.06687694 |
| 障碍分类 | 100% | 100% |
| 两次≥100点 | 通过 | 未通过（峰值45） |

原 NN 的部分表面 layer 在这条全新外观采集中通过原数值条件；这不是跨场景、原均匀域、所有点或盲区完整体积认证。没有将该结果自动接入控制。NN/source/class/partialworld先于当前GT读取；独立 raw四字段重建与色类、native曝光变换精确。

NCC只得到148点（99.3243%/p90 .06688），没有两次100点，明确拒绝；29次只读精化及10秒记忆精确重建、未来日志替换/截断接口通过，最终45当前+103过去、9个XY小格。实际出现纹理并未让该局部匹配器密集，不能再扫窗口/阈值找分。

NCC所有点的 .2 m 固定 MODEL XY disk 仅支持185代理格中的67（36.216%），不是已观测障碍体积，也不认证地面自由或完整3D安全。原研究中全图185覆盖和导航的拒绝保留。

源码/配置1993项、输入18项，protocol SHA 3e8820e4237edaec8b06850eade759dddc7461553a05a1deca6a91e11e2ce48d；post-close SHA ff84d002f4960c9282a6c738f91dec0d7d8560e38f10d67331db43c5ea135b04。所有原始数据与双视频在 native_fixed_appearance_probe_run_v1。本人重建、核验并查看上述实际图片；视频完整解码，未声称全程人工观看。

render/refined审计V1最后因 occupancy键名失败，日志/图片保留；V2仅读取实际 global_origin_xy 字段，无原始数据、判据或结果筛选变化。原相机/灯/均匀fixture全部保留。

## 接续工作

当前正在同一已封存14秒数据上运行旧冻结“部分线索→可能占据区→历史core→负向射线排除”方法的适用性检查（CPU，非新神经模型）。使用原始 paired packet 和当前/过去 native 估计速度，首次原资格 pair、原 .2 m 半径、原在线颜色初始化、原95%覆盖/6 m²/类精度/错误veto条件。新 raw/NCC 标签已经分析，因此这个后续模型检查只能称开发适用性，不能再称 fresh。

模型输出先封存再读新覆盖标签；失败就拒绝，不选更晚seed、扩大半径或换纹理。部分表面layer通过不等于体积/消费/导航通过。世界模型/学习增量、原35秒E0/E1/E2与整体研究仍未完成。


## 原空间假设和实测点记忆：独立后续核验

保持旧0.2m半径与原方法，首次满足条件的观测对为6.5/7.0秒。全部29条生成结果封存后再检查：92个原始核心，删除1个后91个；覆盖134/185=72.4324%，面积0.73m²，120个贡献视图中最低分类精度91.0287%。覆盖与分类两项失败，明确拒绝；没有改半径、另选时刻、修改原门槛或启动导航。被删除的核心没有真实占据误删。该数据已经用于开发，不再称fresh。

一次有限原因核验：唯一低于95%的贡献视图为window70/frame55/left，836个分类像素中761有效、75因真实深度无效或超出原0.5–3m范围失败；其他互斥高度、机体距离与CAD原因均0。这是事后标签诊断，不能过滤掉这75个像素救分，也未证明唯一因果。缺失的51个代理XY格及实际MODEL核心见覆盖图。作者实际查看了图。

首个交接前缀15条源输出/351个原生传感器样本，在未来原生包替换和截断后逐字段完全一致；后续RGB不由前缀循环读取。测试V1在输出时间列表仍访问被毒化的未来包而失败，原记录保留；V2仅限制导出到相同前缀，源算法与候选不变。它不是全29条未来RGB毒化测试，更不是来源/导航通过。

直接使用已保存的10047个原始双目实测表面点，完成了10秒不可变点记忆的历史重建：29条前缀截断/未来点负载替换均完全一致，最终1019当前记录+9028历史记录，14个不同XY格。58,213个历史记录-时刻组合中100个在当前双目CAD/视域都不可见；最终全部9028又几何可见。组合不是独立空间样本，不证明看见整个障碍或完成盲区导航。已有PartialSurfaceMemory在实际采集时确实在线生成原始表面；此次10秒记忆接口为归档重建，不能冒称新在线控制。

原生位姿、深度表面精度通过与完整障碍空间范围拒绝并存；原严格视觉条件、世界模型和35秒E0/E1/E2仍未通过/未运行。下一独立Task H消费者仅将当前/过去表面点标为风险，其他空间保持未知，平地及起步安全区单列；不把被拒绝的MODEL装进导航。CPU预检与实际控制验收分别报告。

证据：`fixed_appearance_hypothesis_source_v1/assessment_v1.json`、`fixed_appearance_hypothesis_rejection_cause_v1/assessment.json`、`fixed_appearance_hypothesis_prefix_assessment_v2.json`、`fixed_appearance_raw_point_memory_v1/assessment.json`、`fixed_appearance_raw_memory_visibility_v1/assessment.json`。
