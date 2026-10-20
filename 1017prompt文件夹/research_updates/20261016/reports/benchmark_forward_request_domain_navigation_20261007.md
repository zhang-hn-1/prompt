# 固定相机受控绕障任务：前进请求域候选实际到达

**PASS_CONDITIONAL_FORWARD_CAP_TASK_ONLY。** 唯一D2真实闭环在43.3秒进入目标，真实终距.187850974m、零接触/碰撞；实际通过障碍段，连续球体代理最小净空.463903m。未完成原35秒E0/E1/E2、纯RGB估计位姿或世界模型增量。43.3秒不能计成原35秒成功。

## 唯一干预与适用域

D1仅规划裕量实验仍REJECT（60秒/.807551m）。D2保留D1规划连续.8m/栅格.870710678m、原参考guard.6m，唯一改变未知区**正向主轴请求上限+.3→+.5rad/s**；负向仍-.3，副轴/PID.4/.1/.7、slew/supervisor、换向五次低速/反弹清零、光照、相机、网络与不确定半径.20m不变。当前已有+.5/-22.5°单点稳态实测只用于动机；没有认证全BankMap、瞬态预测或转弯全域。未扫描上限，未重新标定，候选最多这一条。旧直线2米cap1.0原终距gate拒绝保留；此次不同单障碍/不可观测空间任务不是重评分救分。

固定robot/camera x=.20,y=±.30,z=.05/native640×480，flat ground/safe start .7m是明确先验。受控独立fixture为(0,0)半径.35m单柱，机器人代理球半径.4m。起点(-2.2,0)，目标(2.2,0)，任务最大60秒，与旧重叠柱长墙/正式E1不是同场景。双RGB冻结Fast→当前CAD过滤/事件资格→四个过去曝光的部分外观/射线可能占据集合，初生固定hypothesis作为60秒模型记忆→局部A*/曲线→现有roll-PID真正闭环；后续移动几何不被消费。它**不是可靠完整metric depth/观测自由图**；当前/过去模型标签6/7与测量时间分开，测量自由/占据格始终0。依赖当前真位姿及同场景开发的uncertainty标量，不能称通用视觉导航。

## 实测与冻结门槛

|门槛|实际|判定|
|---|---:|---|
|原生到达且终距≤.4m，≤60s|43.3s/.187851m|通过|
|实际目标轴进度≥4m/实际绕过柱|4.557700m/是|通过|
|零接触，连续body代理净空≥0|0/.463903m|通过|
|所有安装名义参考到模型barrier净空≥.8m|7次，最小.884760m|通过该项|
|出生集合足迹覆盖≥95%、部分分类精度≥95%|185/185=100%、所有8view通过|仅局部静态来源通过|
|实际前进请求改变|336帧|通过|

初生模型162XY格/1.62m²；站立观察资格1.5秒，原跟踪器仍6秒才起步。不是强制早起步。连续proxy净空以事后真实XY折线到柱面、球体半径计算；只证实本次代理，不证明完整CAD三维安全。目标轴进度是最终x减初始x，超过4.4m是实际终点轻微过冲；距目标改善4.212149m是另一口径。真实路程6.752433m。冻结D1完整几何参考独立投影终进度5.911270m，横向RMS.156372/peak.284532m；不读取path_index。

与历史D1实际轨迹比较时，97帧物理/请求前缀一致，之后336请求帧/1650applied ticks不同（首applied tick516）。初生hypothesis packet不是逐字段一致，实际后续参考也不同；因此不宣称完整参考/同源严格因果A/B、TTL收益或时间差的唯一原因。实际当次uncapped→旧cap→新nominal→bootstrap最终request逐层精确复算，副轴和负向保持旧cap，才支持请求接口真的改动。D1原失败、C2guard失败和旧局部控制拒绝保留。

继承metrics.reference_length8.83675/SPL1和minimum_clearance3.83792属于旧元数据范围，不用作此fixture成绩。正式35秒到达未通过、来源全域/移动覆盖拒绝保留。

## 作者实际证据

预封存计划2026-10-07-single-command-domain-cap-validation.md/protocol1353身份。4RED→4GREEN后新增倒车约束真实1RED4PASS→5GREEN；596完整CPU测试37.90s。一次独立首次review C0/I0/M0、个人5fixture.21s/限定源码，未重跑GPU或最终填充协议；作者核验final review/closure/suite/current完整hashbinding后才启动。前景stage15190,parent1710622/child1710623，UTC12:23:47.941→12:30:42.564，exit0/wall414.622s，没有重试或补跑。

433真实请求/434capture/2166physics，87双目source更新；实际433请求/2165dispatch精确，2160 measured-feedback执行克隆及432snapshot精确，初始5未克隆且不是递归预测。4366postseal文件/3219来源字段/174future来源检查/10future事件检查/3031typed地图字段精确；前缀truncate6次来源和26动作不变，没有全文件库future poisoning声明。两视频逐帧解码各434/10fps，眼时差0，轨迹图作者实际查看。模型birth分类真值仅postseal核验，不反馈actor。作者birth/cause/assessment均exit0；与评审范围分开。

source计算wall中位1.856s/p952.407s，capture p50.252/p952.149，仿真暂停处理，因此不是实时系统通过。原始数据目录command_domain_cap_D2_run_v1，视频stereo_view.mp4/third_person.mp4，图command_domain_cap_actual_v1.png；独立causal_verification/actual_assessment/birth_source_check JSON保存。

## 接续边界

成功的条件化knownpose栈现在封存，不再继续扫速度/增益/规划裕量。此前冻结位姿D1压力测试3D RMS.051351>.05仍拒绝，不能直接把旧滚动估计器放入导航。下一项若验证平坦地面高度约束，须明确新任务floor/radius是先验、旧模型不倒改、先独立接口测试和无动态真姿态输入的整序列预测，再最多一个fresh估计位姿闭环；不把原.046m高度偏差事后裁掉称旧pass。学习记忆旧候选拒绝也保留；只有相应基线/数据层前提具备才开始限定学习增量。整体任务未完成。
