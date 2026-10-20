# 观测出生时间、可用时间与固定延迟消费

为后续异步感知先建立正确接口，未改规划器、控制器、相机、网络或旧记忆。这里是受控CPU重放，**不是另一条在线导航，不是实测1.5秒延迟，也不是异步worker已通过**。

`CausalSourceDeliveryMemory`分开记录不可变采集时间与可用时间。未来available在读取任何packet字段前拒绝；要求birth≤available≤当前消费时间，availability/current clock单调。沿用原查询身份、原始pixel/provenance校验、逆序/重复拒绝和10秒出生TTL。晚到数据不会把出生时间刷新为收到时间，被过期/未可用的点不能进入风险图；未知不改为自由。

新增3项实际测试0.25秒通过，涵盖未来payload先拒绝、出生TTL/deepcopy、错误/重复提交原子性。首次独立审查C0/I0/M1，审查者亲自跑3项0.28秒，仅只读作者完整重放，不宣称其本人执行了所有序列。

对已封存配对纹理505请求序列，先完整重建365次事实导航请求与参考，再保留原0..140共同观察/交接前缀，只把后续来源可用时间延后75个float32(.02)物理间隔，即1.499999966秒。它是冻结的合成扰动，不来自时延拟合，无第二个延迟值或扫描。

候选在本序列365次请求全部与事实相同，没有首次请求分叉。算法规定若出现首个不同请求，立即在该次调用后停止，禁止再读取下一次反馈或用原轨迹当候选标签。没有差异的本次结果也只支持接口重放等价，不能作为新到达/碰撞/实时性结果。

另外完成145、252、503三个中间截断点，各6、113、364次导航调用：未来native/IMU/policy日志和**已采集但仍不可用**的来源payload替换/截断后，前缀请求完全一致。文件读取guard在打开历史/来源payload前检查采集与availability，三段分别记录175、301、602次允许读取。

Minor是旧replay由frame公式推导birth：作者独立核验全部102个NPZ保存frame、acquisition_time_s和source log time与该公式严格相等，当前数据没有重新标时间，原脚本字节及首次审查保留。该相等关系**不是未来通用契约**；实际异步实现必须读取/校验捕获packet的出生字段，不能凭帧号重新定义它。

本实验没有NN推理、物理仿真、新控制A/B或标签重定义，也未恢复被冻结学习头。仅验证handoff之后的受控延迟；真实进程初始化、GPU/render并发、有限队列、掉帧和实际到达时钟仍未验证。正式35秒评测和整个任务仍未完成。

证据位于`experiments/stereo_memory_navigation_20261004_v1/`：组件、3项测试及RED/GREEN；`causal_delayed_source_delivery_replay_v1/`（75390退出0，源/输入先封存、365记录）、`causal_delayed_source_middle_prefixes_v1/`（45074退出0）、`causal_delivery_clock_input_alignment_v1.json`、`causal_delayed_source_delivery_first_scoped_review_v1.json`。

具体接续：先用独立spawn进程执行固定三组已封存RGB/当前native pose/实测关节任务，验证提交/接收不阻塞主接口、queue≤1、当前/过去packet身份不变、异常可收束，且不读GT/SDK现场。它是处理器运输验证，不能冒充在线导航。此关通过后，再冻结一次导航阶段异步来源的60秒实际接口实验；观察前缀、原roll-PID、点规则、路径策略和验收不变。
