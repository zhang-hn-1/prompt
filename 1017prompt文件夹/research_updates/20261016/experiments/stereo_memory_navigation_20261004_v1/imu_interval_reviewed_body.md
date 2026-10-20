# 显式IMU区间交付与独立完整序列姿态验证

本次新交付接口通过，唯一固定重力辅助滚动结构在独立完整E2+20°初态32秒序列达到原位置/姿态门槛：完整1550/1550评估步，3D RMS **.0469695m**、peak **.0483718m**、姿态p90 **.808587°**。仅允许该测量域的姿态层结论；不是RGB深度、地图、控制或正式导航通过。旧原生V1接口REJECT及其null物理成绩保留，没有修改旧日志或重放来生成新分数。

## 接口问题与唯一固定结构

独立SDK已知激励见证表明native读数标签是步首、内容响应本步；随后仅分解已拒绝机器人旧数据的body basis：native gyro对R_begin^T*omega_end RMS1.04925e−8rad/s，而对R_end^T*omega_end为2.78183e−5。它不是坐标轴旋转的大错误，也不是整个前一帧内容；旧current-end-time/当前body比较资格假设不匹配该SDK。该分析在旧拒绝之后、使用GT只作标签，不调用滤波器或产生旧评分。

新版本从原World.step实际调用前/后采集区间时钟，原函数只调用一次；原生stamp须匹配实际begin<=1e−5s、end−begin须.02±1e−6s，不是由GT误差选最佳偏移。模型packet只有delivery-relative-end时间、native begin-basis gyro/acc和实际主轴编码器。没有GT gyro参与资格、没有sensor orientation/dynamic GT pose输入；body区间语义和一物理步的近似滞后明确保留。初始normal只在t0明确仿真标定提供，非完全自主硬件姿态初始化。

GravityRolling源码与先前封存SHA精确相同：前一步gyro SO3、前一步速度积分、当前gyro用于滚动速度、acc模长9.81±.5时k=1/s校正重力；不改gain/半径/时间积分/模型结构、不学习bias或拟合网络。原结构此前没有任何有效物理评分，修正的是传感器交付，不是看过差成绩后改模型救分。旧接口gate保留，新的区间资格是独立版本。

## 实际独立whole-sequence

188source/5输入19:36:43.203353UTC封存后，唯一完整运行19:37:14.441087→19:39:11.991773退出0。已知E2场景和旧运动域作为development/context，新的+20°初态整条physical/sensor运行在冻结参数之前不存在、未拟合或随机拆帧；不称fresh环境泛化。单reset、内部不reset，zero6→main+.5/rawaux−30°20→zero6，320个5×2请求块、1601physics/321capture/65RGBpairs/1600PID反馈、两321帧视频。不是正式E2导航计分或新成功率。

当前PID(.4,.1,.7)、.02s、mainaccel.5、副轴target slew30deg/s/actual velocity.3、相机x.2/y±.3/z.05和fixed_camera_fill_v1均不变。独立初态20°是预声明验证条件，不能与不同初态旧E2要求17物理数组全同；原始1600raw请求与sealed完整schedule逐项精确，同一shadow从不输出action。已有同初态nativeV1-B0的17nonraw精确只作为历史只读证据，不冒充本次新条件A/B。

## 分层验收与完整数值

区间资格1601/1601通过、无缺失；begin stamp误差峰1.72481e−6s、actualdt误差峰4.47e−10s。所有t>=1s的1550预测完整、在当步生成后才进入下一physics/request，不读取后续真值。完整原gate为3D RMS<=.05/peak<=.1/attitudep90<=2°，全部通过；没有去掉高度/较差阶段或用settled子集救分。

XY位置RMS.00909061m/peak.0147074；XYZ分量RMS(.00807840,.00416877,.0460814)m。3D主要误差仍是初始机体下落与平面滚动Z恒定假设，未修正或删除，consumer不得自动把本结果视为地面高度/相机投影全域准入。姿态极值、瞬态、逆向/不同地形/真实噪声和偏置尚无证据；此序列唯一原参数假设通过，不授权全域模型或模型替代planner。

326suite10.40s；6interval fixture RED6fail.10s→GREEN6pass.12s，覆盖begin stamp/actualinterval/finite/valid/复制/orientation不可读取。914来源/输入/721postclose产物SHA通过；实际双视频逐帧解码各321帧10fps，0/160/320拼图与误差图已打开。首版行政auditor误以requests形状320×2，真实格式是320×10（每块五个二维physics请求）；退出1原代码与记录保留，独立v2仅修正shape并reshape1600×2，没有改模型/原始数据/gate或重跑物理。首plot因该measurement未生成退出1；成功图在v2audit之后生成，失败范围保留。

## 下一项与尚未完成

接下来仅作同一已冻结区间接口/结构在完整复杂原B0上的只读在线集成验证，与同初态旧B0核验17执行/物理字段、原30.7s行为；不改控制/gain/参考/相机，不将已有B0场景称fresh held-out。目的核验这次恒bank序列的姿态通过能否覆盖导航中的复杂激励，而非继续拟合新模型。若失败关闭部署准入，不扫参数。

随后必须回到双RGB来源/显式记忆/盲区与可执行局部规划目标；位姿通过不能修复已有floor分类、匹配覆盖及局部35秒超时。RGB+估计位姿闭环、唯一学习记忆/世界模型增量及正式35秒E0/E1/E2仍未完成，automation ACTIVE。没有因本报告小阶段结束停止研究。
