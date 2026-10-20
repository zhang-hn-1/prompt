# 原生IMU事件时间见证：当前内容、步首标时

独立SDK受控立方体完成50个.02s物理步、1秒已知Z轴激励；运行19:26:14.943644→19:26:25.065220UTC退出0。没有机器人episode、滤波预测、控制改动或新导航结果。11来源事前19:25:57.134871封存，13来源/实际产物SHA通过。5literal fixture RED5fail.09s→GREEN5pass.08s；全320suite11.03s。

自由立方体gravity/damping设0，仅为该隔离见证而非修改机器人物理。sensor原点/identity/.02s/三个filter1/latest/read_gravity保持原SDK配置。每步之前发送固定已知Z角速度[0,.3,−.4,.7,−.8,.1]重复，读取step前/后时钟与native读数，再保存实际body pose/omega诊断；没有读取native orientation。bodyZ保持worldZ，交替激励保证前后命令不同。步前后时钟来自实际调用，不用真值拟合最佳偏移。

排除事前规定两个启动步后，完整48步全部有效/有限，以下门槛均事前固定：time1e−5s、currentbodygyro/已知command1e−5rad/s、clockstep1e−6s。

|独立检查|实际结果|
|---|---:|
|sensorTime与本步开始时钟的最大差|2.98023e−8s|
|sensorTime与本步结束时钟的最大差|.02000002936s|
|原生Z角速度与本步已知请求最大差|1.20401e−6rad/s|
|原生角速度与本步实际body角速度最大差|2.22e−16rad/s|
|原生Z角速度对前一步已知请求平均绝对差|.76666693rad/s|

本独立fixture的**begin-label/current-content**假设通过，end-label假设拒绝。当前SDK这个采样路径在步尾读取的内容响应本步激励，时间标签对应步首；不能只因20ms标签差称整帧内容延迟。源码SimulationContext.step把步前current_time传入PhysicsContext._step，timer callback再累加dt，与见证一致。[官方对应代际IMU文档](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/sensors/isaacsim_sensors_physics_imu.html)说明latest读取当前物理步，但未明确步首/尾；本实验补充的是本地版本实测，不移植最新6.x API。

范围限制：只验证零重力、自由立方体、Z轴激励和固定SDKgetter；没有认证受约束机器人所有轴残差、真实硬件时钟、任意rate/filter组合。旧机器人接口的1e−5 current-end-time与轴门槛及其REJECT保持，旧1536 packet仍null、物理预测成绩仍null。没有对旧B0日志加dt后跑滤波、也没有事后宣称旧gate通过。

下一项若继续，只能建立独立显式区间时间契约，用实际步前/后时钟登记原生样本的区间，保持单一未评分的固定重力辅助结构；不能盲目添加拟合偏移或用GT旋转代替nativegyro。受约束body的微小gyro差异仍须单独测量解释，不认证动态姿态门槛。回到视觉来源/盲区记忆与35秒闭环目标仍必要，整体任务未完成。该fixture无相机/视频，不制造导航视频证据。
