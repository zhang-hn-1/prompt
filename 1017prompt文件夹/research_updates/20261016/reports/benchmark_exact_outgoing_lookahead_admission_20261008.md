# 精确前视点候选：执行目标有差，但提前拒绝无望的局部对照

**REJECT_EXACT_LOOKAHEAD_BEFORE_FIXED_REFERENCE_GPU。** 此候选未进行新导航仿真，不能报告改善或新成功率。它不是全程行为等价，但有效变化出现太晚，无法修复此前已经超过门槛的偏离。

在负反证记忆R1封存输入上，同版本policy逐帧一次调用、125/125原请求精确复现；109个正常tracking字段有效，其余提前返回字段不使用。原outgoing lookahead取整路点，距离超调最大.299689m。反事实同向连续线段目标与旧目标位置差最大.433889m、中位.086559m。这说明离散取点与期望距离不同，不能仅凭该差异认定实现bug或控制改善。

唯一候选只改变目标点计算，首次正常reverse→forward被原handler接受后使用当前位置在同方向参考上的投影加原adaptive距离并线性内插。反向与之前状态委托旧函数；不改path_index、原参考、曲率公式、BankMap、增益、速度、PID、方向确认或阈值。已有授权设计及CPU几何/事件验证，7个feature assertions通过；最初RED是feature缺失的导入失败，不称7个真实behavior RED。全套638通过40.93秒。

同一历史current-input探针中，候选仅在frame255开始改变协议请求，此前115个导航请求完全一致。之后10个协议请求不同；第一255当前执行snapshot的5步常量当前反馈比较applied仍完全相同，256/257两个当前状态探针applied不同，其余8个相同。这是当时已有状态和给定请求的执行链探针，没有读取未来实际反馈，也不是反事实轨迹标签；不能将后续旧实际运动当作候选真实运动。

在首协议差之前，相同已知参考/同前缀的真实横向peak已经.321407m，超过事前固定的.2m门槛。后来的执行差不能逆改早先的峰值，故不运行注定无法满足该指标的GPU pair，不提前改触发事件、不调lookahead值或放宽.2门槛。计划仍保留原bytes，候选拒绝独立存档。

行政失败：初版探针新建最派生tracker忘记按原factory初始化mapping_records，clock审计报AttributeError，未跑GPU；独立v2仅补同一原factory初始化，v1源码/失败目录保留。候选计算出的新前视distance超调接近0、target_difference字段0来自与同一精确算法自比较，不能当作旧/新收益，旧/新几何差以独立factual audit为准。

实际请求链的重要发现：frame190、200、210、220、230、240、250、254曲率请求约1.96、1.47、1.20、1.35、.90、2.21、1.06、1.15，BankMap全部curvature_clamped，最终bank目标−22.5°。当前匹配的single+.5/−22.5稳态曲率约.792373，整张BankMap仍未认证。earlyheading偏差/有限转角响应/不可执行名义参考需要分别核验，不能继续把问题仅归于前视点取整或M2没通过。

接续一条事前冻结72秒当前低速前进coverage：零→+.5直线→+.5正bank22.5→+.5负bank22.5→零，三个20秒段固定尾窗。补明确缺少的当前正转、直线与移动反转覆盖，最多1序列，无矩阵/拟合/PID调节/新controller；需要实测与既有来源对应点比较，不能直接把单点或三点结果当新规划模型。原来源/位姿/记忆/导航/学习拒绝、原35秒E0/E1/E2规则保持。整个研究未完成。

证据：H下`discrete_lookahead_chain_audit_v1/{trace,assessment}.json`、`exact_outgoing_request_probe_v2/{trace,assessment}.json`、`exact_outgoing_current_execution_probe_v1.json`、`exact_outgoing_candidate_admission_v1.json`、`exact_outgoing_lookahead.py`、CPU/失败日志及`docs/superpowers/plans/2026-10-07-exact-outgoing-lookahead-candidate.md`。本阶段没有新GPU实际轨迹、second-person review或Git提交。
