# 理想陀螺仪与球壳编码器：历史预测、在线只读验证与失败定位

本阶段真正运行了一条完整的 Oracle B0+只读里程计导航。原 Oracle 在30.7秒到达目标，最终距离0.195485623m；17项非raw物理/执行字段与原B0逐项精确相同。里程计没有改变控制动作。但其在线位姿预测未通过事先冻结的门槛，不能接入视觉导航。整项研究尚未完成，正式E0/E1/E2与学习记忆增量均未启动。

## 1. 传感器、刚体与估计器身份

静态USD核验：joint1父为base_link、子为球壳link1，转轴Y，两端pivot同中心；link1球形碰撞体半径0.40000000596m，球体到link1为单位变换。没有用旧V1的0.3m半径替代实际模型，也没有拟合尺度。USD层SHA与球体直达路径见 `rolling_odometry_asset_identity_v2.json`。球体为instance proxy，默认Traverse遗漏它，直接GetPrim核验后才记录半径。

历史active18/bank32各自未保存DOF名称，最早只复用了最后可用的真实runtime读回joint1/2/3，限制已明确。新在线运行首次实际读回dof_names=['joint1','joint2','joint3']，main encoder index=0，见 `rolling_online_run/rolling_live_identity.json`。不再把历史读回冒充新现场认证。

估计器只能读取当前时间、body-frame gyro和实际主轴编码器速度。初次重力法向是t0标定；半径/轴方向是静态几何。仿真“理想陀螺仪”由CURRENT的root quaternion与世界角速度合成body gyro，GT quaternion仅用于传感器模拟及初次法向，不作为逐帧姿态纠正；GT位置不进入估计器。零噪声、零偏置假设没有验证真实IMU，bodygyro本身来自仿真真值，因此这不是纯RGB位姿结果。

V1使用前一采样gyro的SO(3)指数积分当前姿态；球壳角速度为R_est*(gyro+axisY*actual_qdot_joint1)，平面无滑动假设速度为r*(omega_shell×normal)。以前一已知速度积分位移。输入错误/倒序时间在变更状态前拒绝；无未来反馈、拟合、晚期对齐、重置、GT落地过滤。

源码 `rolling_odometry.py`、`rolling_shadow.py`，白名单与未来后缀/位置真值污染fixture见 `test_rolling_odometry.py`、`test_rolling_shadow.py`。真实在线hook在每次正式physics diagnostic之后读取当前row，shadow返回None，policy不消费它。

## 2. 冻结门槛及历史完整序列

spec/plan于两条历史预测前冻结，全部预测先保存再读取评估位置/姿态标签；只有t0 gauge对齐。完整t>=1s分母，3D位置RMS<=0.05m、peak<=0.10m、姿态geodesic p90<=2deg，任何失败拒绝。没有看完在线数据把3D改成XY，也没有去除初始下落。

| 完整序列 | 所有/评估采样 | 3D RMS / peak(m) | 姿态p90(deg) | 原门槛 |
|---|---:|---:|---:|---|
| active18 |901/850|0.046126 / 0.046278|1.270046|通过历史诊断|
| bank32 |1601/1550|0.047353 / 0.049652|1.605321|通过历史诊断|

这些都是用于开发的旧数据，不是fresh。协议 `rolling_odometry_protocol.json`，独立预测 `rolling_odometry_predictions`，原评估 `rolling_odometry_audit.json` 原字节保留。历史通过仅允许下面的一条在线只读核验，不代表导航或实传感器通过。

## 3. 唯一真实在线 B0+shadow

原golden E1同场景、相同完整已知全局参考、PID、速度、相机 `ball_stereo_front020_up005_v2`、固定光照、原35秒终止规则。新runner为旧 `run_oracle_rgb_shadow.py` 的机械复制加独立physics hook；差异见 `rolling_online_runner.diff`。原RGB ORB shadow也保持原样。控制仍是全图Oracle，不能将成功写成视觉导航或新成功率。

事前协议 `rolling_online_protocol.json`：2026-10-05T17:41:13.940992UTC，67源码/5原B0产物，SHA4bb407f06262bf74f87b3f2f0b41aaab4660f8aa257ad621169f514d5b8c4835。实际运行17:41:56.330911→17:44:44.207202UTC，退出0，仅一次；1536物理采样、308capture，308帧10fps双视频，无额外物理尾帧。真实DOF名称本次读回。原raw初始化时钟单列，不能把非raw一致称所有raw一致。

在线预测在physics hook当时产生并在运行结束保存，后续auditor只评估已有预测，没有事后重放冒充在线。全部1536时间/白名单对应，t>=1s评估1485采样。与原B0比较，17项非raw数组shape与内容精确相同，包括请求、协议、smoothed/applied、关节与世界状态。详见 `rolling_online_audit.json`；其auditor是post-seal新评估文件，单独SHA记录，不暗改冻结runner协议。

| 原冻结在线门槛 | 实测 | 结果 |
|---|---:|---|
|3D位置RMS<=.05m|.057997866m|失败|
|3D位置peak<=.10m|.080070635m|通过|
|姿态p90<=2deg|3.823861deg|失败|

**拒绝在线V1准入。** 原B0到达终点只证明控制未被只读shadow改变，没有证明估计位姿闭环导航。

视频实际逐帧解码各308帧，并查看0/153/307原始帧拼图；相机/照明未改变，机器人自遮挡仍明显。证据 `rolling_online_video_check.json`、`stereo_view_rolling_online_frames.png`、`third_person_rolling_online_frames.png`。

## 4. 有界原因核验：不能靠换积分口径救分

在线失败后冻结独立诊断协议 `rolling_diagnostic_protocol.json`，仅对同一已用序列检查三种标准采样积分，0新仿真、0fit、0控制改变。它不是新候选资格或fresh验证，原V1拒绝不追改。

| 离线诊断积分口径 | 姿态p90(deg) |
|---|---:|
| 前一角速度 |3.823861|
| 当前终点角速度 |3.754024|
| 两端平均 |3.789434|

三者均不能消除方向漂移，不认定简单“交错一个采样”修复已成立。实际Log(Rprev^T Rcurrent)/dt与begin gyro误差RMS.011510rad/s、endpoint .008279rad/s；endpoint较近却仍有残差。当前SDK articulation有效handle下pose取physics_view.get_root_transforms，角速度取同view.get_root_velocities；不是通常USD/Fabric pose fallback，尚无确定缓存或时间戳bug。姿态求解修正/离散物理积分可能贡献残差，但未直接记录solver姿态校正，不能写成确凿原因或拟合gyro bias。

同一在线数据：3D RMS.057998m中XY RMS.035218m、XY peak.065481m；竖直RMS.046081m。实际初始球心高.446081668m、最低.399993956m、最后.399999291m，平面无滑动位移模型本身不模拟这段初始下落，不能用late reset消除它。XY诊断不替代原3D门槛。

另一个严格标注为GT-attitude的离线运动学上限诊断：提供实测姿态后，球壳无滑动公式对实测XY速度残差RMS.004037m/s，积分XY位置RMS.005099m、peak.009269m。它支持“当前水平位移偏差大部分与姿态积分有关”这一有限解释；不是无真值预测结果，也不是整个运行无滑动的证明（没有独立shell实际角速度/接触滑速读回）。world-z球壳自旋本身不产生平移，不能用root yaw速度替代编码器与完整旋转。

上述实际诊断、输入/source/API SHA保存在 `rolling_gyro_time_alignment.json`、`rolling_temporal_diagnosis.json` 与npz。图 `rolling_online_diagnosis.png` 已查看，门槛虚线明确是RMS门槛，不是每帧门槛。

## 5. 实现、测试、限制与接续

主估计器RED4fail/1pass .20s→GREEN5pass .13s。在线边界最初三个helper fixture与实现并行写，首次运行3pass，**不称完整RED→GREEN**；坏初始sensor会锁存错误标定的问题有真实RED1fail/3pass .17s，修正为成功update后才提交状态；第一GREEN夹带fixture名错误保留日志，修正测试文件后4pass .13s。上线前整个H277pass9.51s，没有测试排除。源码/原失败/实际运行日志保留。

本阶段交付分别是：历史理想传感器诊断通过；在线只读接口与动作不干扰通过；在线完整位姿门槛失败；Oracle原行为成功复现；RGB源深度/空间记忆自由证据仍不足。没有新的控制A/B、学习记忆训练或正式E0/E1/E2。前一层通过不代表后一层。

继续工作不能无边界拟合姿态/扫积分/以GT姿态救准入；先关闭此唯一在线估计候选。回到视觉源层，核验目前双RGB中可观测地面/障碍证据，区分观察像素与平面假设，考虑在可验证观察证据上建立自由地面来源，而不是给未看见区域填自由。已有稀疏ORB对地面零证据是明确消费者阻塞；只有可靠源测量后才继续空间记忆闭环及封存学习增量。固定相机、自遮挡、历史全部拒绝和35秒规则保留。整个任务不宣称完成，自动续做不停止。


## 独立审查与作者范围澄清（新追加，不修改上述8553字节正文）

ONE fresh gpt-6-astra/high审查0Critical/0Important/2Minor，亲自9fixture .21s、全部预封存来源/输入、2483清单中19根产物SHA、两历史与在线门槛、17非raw字段及拼图。未重跑完整视频/仿真/277suite/三个积分或GT-attitude诊断。原文见rolling_online_review_transcript.md；七项未评范围作者逐项接受为明确限制，裁决及风险另存rolling_online_review.json。

两Minor独立保留：测试on/off、future名称超过真实局部纯函数/相同前缀确定性验证范围，不能称通用集成隔离证明；本次真实B0数组精确对照与hook源码补充证据。SO3 norm/inverse、非共线动态bank解析fixture未完整覆盖，后续修改估计器时补；不为本次已拒绝候选重跑或变更冻结来源。原正文fixture描述按该限制理解。在线V1仍拒绝，任务尚未完成。
