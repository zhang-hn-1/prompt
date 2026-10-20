# 平坦地面先验下的估计位姿绕障闭环

**PASS_CONDITIONAL_ESTIMATED_POSE_TASK_ONLY。** 唯一 fresh D3 实际运行42.9秒到达，真实终距.190624432m，零接触/碰撞，连续球体代理净空.462167m，实际通过障碍段。整序列 t≥1s 的3D位姿RMS .010011m、peak .016496m、姿态p90 1.771204°，分别通过原.05m/.10m/2°门槛。相机安装和光照未改。不是原35秒成功率、纯RGB或真实带噪IMU验证。

## 原拒绝与唯一新模型

原冻结滚动估计器D1 3D RMS .051351>.05的拒绝保留。它保持初始高度，而模拟机器人从.446082落到约.4m；没有改旧输出或把旧3D评分改成XY。

新独立 FlatGroundPoseBridge 将已授权任务的floor z=0/CAD代理球r=.4000000059604645作为高度先验，root/camera高度取floor+r，垂直速度0；XY和姿态仍使用原RollingOdometry、原陀螺积分与编码器规则。保留未约束height并显式标注prior≠observed/contact/full3Dsafety。这只适用于这里的平坦地面任务。初始XY/R/重力一次真实标定、理想bodygyro模拟以及底层roll-PID实测roll仍享有仿真特权；理想gyro模拟器内部当前真姿态负责世界角速度→机体系转换。预测器只读time/bodygyro/encoder/初始规范，不读后续真实位置或线速度。

新结构在历史D1/D2整序列预封存预测后再评分，RMS .022660/.009708m通过；它们均历史开发，不称fresh。两条各1001传感器前缀的2002字段精确，未来位姿请求拒绝。然后才实际运行一条新完整D3，没有扫描另一高度、积分器、网络或控制参数。

## 真正使用估计状态的接口

实际50Hz记录推进滚动预测，10Hz曝光位姿严格按当时已到达时间取值。双目端点注册、外观重投影、partial ray-set/root packet使用估计pose；source机会判断取估计速度。导航observation重建位置/yaw/速度/角速度/重力/四元数历史，点目标从已知goal与估计位置重算。原17D真值运动0:9及动态root位姿不进入actor；source CAD接口只给当前关节/过去动作，0:9清零。当前真姿态/renderer depth单独保存诊断，不能流回actor。底层PID、protocol/smoother/supervisor、速度+.5/反向-.3、D2规划裕量及五次低速/反弹规则保持。

初生source资格2.0秒（D2是1.5秒），仍由原policy6秒起步；没有修改资格阈值。出生hypothesis163XY格/1.63m²、185/185物理足迹覆盖，8个partial-class view最低精度.992456，只准入局部静态可能占据集合。后续moving possible sets不消费；测量free/occupied格始终0。source packet pose_sigma_m=0是继承的未认证假设字段，不是估计位姿零误差或已校准统计协方差；模型barrier固定.20m假设和原全球depth拒绝保留。

## 实测分层验收

|层次|实际|范围|
|---|---:|---|
|物理导航|42.9s/.190624m/0碰撞|60秒独立单柱任务通过|
|真实进度/路径|目标轴4.486695m/路程6.442273m|不是path_index|
|连续body proxy|净空.462167m|本次代理，不认证完整CAD|
|安装名义曲线|7次，最小到modelbarrier .893387m|原≥.8通过，不等于动态安全证明|
|位姿|3D RMS .010011/peak .016496m/姿态p90 1.771204°|2095采样/原门槛通过|
|静态初生hypothesis|100%足迹/最低partial precision .992456|与全球/移动depth拒绝分开|
|冻结D2完整参考误差|RMS .153463/peak .310041m|描述值，不宣称通过旧其他实验.30门槛|

不同初生集合、source事件、实际参考存在：与历史D2仅60帧物理/请求前缀一致，206帧请求和1621applied ticks不同（首tick326）。不能从42.9 vs43.3秒或路程差证明估计位姿更好、记忆TTL增益或单一因果收益。继承metrics SPL/reference-length/clearance字段不作为此fixture成绩。42.9仍大于35秒，正式E0/E1/E2未触发。

## 运行、测试与作者核验

4真实missing-module RED→4GREEN；600完整CPU37.90s通过。一次FIRST审查C0/I0/M0、个人4flat+2observation-poison+4oldbridge=10tests.62s及限定源码/历史seals，没有GPU或最终填充协议认证。审查人hash对应保留的pre-metadata driver；作者另关闭一行pose_privileged/estimated/prior元数据覆写。错误版本assert及缺协议的stage读取失败保留，二者均在物理启动前且没有状态文件，不是科学重试。最终1378身份/closure/suite/current绑定由作者核验后才启动。

前景session91570,parent1734892/child1734893，UTC12:44:56.153→12:52:03.056，exit0/wall426.902s，唯一429请求/430capture/2146physics/86网络source。作者sensor/observation、causal、birth、assessment四程序均exit0；4747封存文件、2146sensor与滚动prediction精确、6450pose/860source/2574actor字段精确；501传感器截断前缀和future pose拒绝；3182source字段/172future source traps/10future events/429request/2145dispatch/2140 measured-feedback clones/428snapshot/3003typed地图字段精确。初始5tick未克隆，不是递归物理预测。两视频逐帧解码各430/10fps、眼时间差0、轨迹图作者实际查看。审查与作者实际证据分别限定，没有第二次GPU/复审声明。

source wall中位2.070s/p952.147s，capture p50.250/p952.359s；仿真暂停处理，不是实时系统。原始目录flat_ground_pose_D3_run_v1，视频stereo_view.mp4/third_person.mp4，图flat_ground_pose_actual_v1.png，审计JSON各自保存。无额外计分尾帧。

## 继续核验盲区与记忆角色

随后作者复用D3封存数据的8个固定曝光做真值诊断：0/6/12秒两眼柱面可见，18/24/30/36/42秒两眼当前深度中柱面像素均0（在CAD mask之前即0）。这证明这些采样时刻柱面不在当前所见表面，不是mask删除了可见柱；不能仅据零像素把原因都归为近场机体遮挡。18秒仍保留2秒出生的163 model cells，参考版本1来自6秒；之后地图仍用同一出生几何重规划。未把新图像读到的先验/未知写成观测free。

raw查询在0/6/12秒柱面精度仅.784694/.840537/.977035，不能以最后一个较好时刻升级全球源。盲时无raw柱面queries。空间hypothesis记忆和已缓存路径都是历史信息；因此“current-only地图”也可能通过保留路径拥有隐含记忆，不能直接拿它作完全无记忆对照或宣称学习收益。诊断v1误把birth时间网格当标量，原错误code/log保留；v2只从网格提唯一有限birth，exit0，未改模型/gate/旧结果。细节conditional_blind_region_audit_v2.json。

目前通过的是带显式先验、静态belief、理想传感器的条件化导航链。世界模型/学习记忆增量、动态/多障碍通用源、原35秒E0/E1/E2尚未完成，旧学习head和源拒绝不倒改。先核验冻结学习模块能否适用于这一来源/记忆接口，再决定有界新验证，不能把缓存曲线或手工先验冒充学习改善。
