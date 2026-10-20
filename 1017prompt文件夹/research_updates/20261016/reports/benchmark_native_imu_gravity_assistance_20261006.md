# 原生IMU与唯一重力辅助候选：接口拒绝，尚无物理预测成绩

实际执行一条原Oracle B0+原生IMU只读shadow，2026-10-05T19:17:31.378000+00:00→2026-10-05T19:20:18.599376+00:00UTC，退出0。Oracle仍30.7秒到达、距离.195485623m；17项非raw动作和物理数组精确相同，raw时钟delta0。新增传感器没有改变本次控制行为。没有控制A/B、新导航成功率、正式E0/E1/E2或学习记忆网络。

## 唯一候选与输入边界

基于上一项实测：实际球壳角速度编码器链正确、旧积分漂移主要为roll/pitch；仅尝试一个低阶传统重力辅助滤波。当前gyro SO3、此前速度积分，acc模长9.81±.5m/s²时以k=1/s校正重力方向。半径/编码器/no-slip假设保持；不fit、估gyro bias、改solver、补动态GT或扫参数。初始重力normal只在t0从明确仿真标定提供；它不是纯RGB或实际硬件独立IMU验证。

Isaac原生sensor挂base_link原点、单位局部姿态，.02s/三个filter size1，use_latest_data/read_gravity=True。读数只复制time/valid/gyro/acc，绝不读取orientation字段；严格packet只time/gyro/acc/actual joint1 encoder。当前GT合成gyro仅用于接口资格比较，模型从不接收该比较值或GT姿态/位置；读取发生每个正式50Hz步，控制不消费预测，新增代码机械diff保存。6模型toy测试RED6fail.18s→GREEN6pass.22s；6接口测试RED6fail.14s→GREEN6pass.12s，其中orientation getter故意抛错证明不访问。315全suite10.24s通过，不等于实际传感器或估计器通过。

源代码核验纠正计划生命周期假设：marker hook在首次world.reset/预热之后、既有正式articulation重置之前。sensor在那里创建，随后原有world.reset注册；无额外reset/物理步。这一纠正与代价在preseal前追加spec/plan/ledger，没有事后更改协议。85source封存19:17:08.237605UTC，6baseline；最多一个实际序列。同一开发B0重复用于首次原生传感器资格，**不是fresh环境或fresh whole-sequence验证**。

## 预冻结门槛与实际拒绝

原生time须与当前raw clock差<=1e−5s，gyro body比较norm<=1e−5rad/s，startup仅允许<=.04s无效读数；其余拒绝。模型仍要求原t>=1s完整分母、3D RMS<=.05m/peak<=.1m/姿态p90<=2°。没有改为仅XY/settled或利用未来applied。

1536/1536原生读数is_valid，但全部time_mismatch：age均约20ms，RMS.02000000153s，峰值.02000095323s；减一个名义dt后的绝对残差峰9.5323e−7s。时间门槛未通过，模型没有消费任何样本，完整1485评估分母中0条预测；3D RMS/peak/attitude都是null而非0，**不能评价该滤波的物理终态性能**。接口REJECT，候选没有准入。没有事后把时间加20ms或放宽轴容差生成成绩；旧gyro-only、RGB深度/来源、M2/导航拒绝保持。

## 有界原因核验与证据限制

独立post-rejection标签诊断只计算已采读数、没有模型重放/新运行/改参数：>=1s原生gyro对当前body gyro RMS2.78183e−5rad/s，对上一物理步body gyro RMS.00743851rad/s；当前匹配门槛仅681/1485满足。读数内容远近当前，**不能仅看20ms时间差就断言它整帧延迟**，也不能据此唯一证明是开始时刻标签或回调时钟约定。固定1e−5轴匹配门槛同时不满足全部样本；这是接口资格过严/读数差异待解释，不证明大坐标系旋转错误，更不是允许事后改gate救分。

在原norm范围内1485/1485步acc对当前GT重力方向p90=1.30631°/peak1.91810°，是只读标签诊断，未输入模型或产生新滤波成绩。free-fall/落地早期模长范围.06605..48.75079；不把加速度天然当重力真值。[本地对应代际的官方Isaac IMU文档](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/sensors/isaacsim_sensors_physics_imu.html)说明use_latest_data读取最新物理步、输出本地轴gyro/acc；公开说明未规定时间戳是步首还是步尾。本地IsaacSensorTypes.h声明time float、gyro rad/s、acc m/s²；SimulationContext._step_callback用step_size累加运行时钟。这些支持进一步检查时钟身份，尚未唯一证实内部IMU采样标时。没有把最新版6.x接口替换入当前SDK。

## 实际产物与后续接口问题

原生1536读数/白名单packet记录/availability、实际link/physics/PID/308capture/完整两视频/source与postclose身份全部保存。两视频实际逐帧解码，各308帧10fps，0/153/307拼图与测量图已打开。空预测曲线表示未准入，不是零误差。实际日志包含既有渲染primvar警告，退出0且计数/动作一致；不以日志警告掩盖接口拒绝。

下一项明确针对**传感器标时语义**，不继续调这个滤波器：需要一个独立受控事件时间见证，分别记录step开始/结束、原生sensor timestamp及当前/前步已知激励，证明读数对应什么时刻；不以此B0事后最佳偏移当证据。不重做导航矩阵，不启动第二个滤波候选或fresh准入验证；当前原始门槛与拒绝保留。传感器同步解决也仍不解决RGB来源及近场盲区，整体任务未完成、automation ACTIVE。

## 独立复核与作者附录

一次fresh只读终审C0/I0/M1，12fixture亲自通过.29s；85source/6baseline/2489postclose、机械diff、实际生命周期、orientation白名单、t0标定、1536null packet/0of1485、17nonraw/raw0、读数算术及两完整视频/图均亲自核验。315suite/RED与实际启动退出仅检查作者日志，未重演。Minor文字澄清：原正文“当前gyro SO3”不准确，应为**前一步gyro用于SO3积分，当前gyro用于滚动速度**；源码/协议始终如此，原已审正文不覆写，拒绝结论不变。作者保留未知时间戳唯一语义、未评分滤波、非真实硬件/fresh/nav、未来独立见证不在本审范围。无第二审/滤波重跑。
