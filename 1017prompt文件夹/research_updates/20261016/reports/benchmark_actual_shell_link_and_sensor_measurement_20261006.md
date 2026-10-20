# 实际球壳与机架物理量：滚动残差和传感器测量身份

实际新增一条原Oracle B0+只读刚体测量，30.7s到达、终态距离.195485623m。17项非raw执行与物理字段与此前B0全部精确一致。此次只增加每个50Hz物理步的base_link/link1等link姿态、线/角速度以及初次COM读回，不改变控制、planner、参考、gain、速度、相机、光照或原35秒计分。

本次补上“球壳真实角速度未单独读回”的测量缺口。结果支持此序列主要表现为滚动；没有地面接触力/实际接触点证据，不能宣称所有帧完全无滑动。旧gyro-only里程计、视觉来源、M2/导航拒绝保持；没有新预测模型、控制A/B或正式E0/E1/E2成功率。

## 实际接口与运行

预封存protocol18:56:46.072544UTC，74来源。SDK当前articulation有效handle/shared_metatype，每次正式physics diagnostic之后，只调用get_link_transforms/get_link_velocities、初次get_coms，没有setter、kinematic刷新、额外物理推进或policy.act。原RGB/rolling只读shadow也保持。新运行18:57:02.378943→18:59:48.500076退出0，一次实际B0，1536physics/308capture/1535反馈和完整双视频；已核验feedback JSON实际1535行。内部不reset。

实际link_names=['base_link','link1','link2','link3']，DOFjoint1/2/3。link1球壳半径.40000000596m、轴Y、球体到link1单位变换沿用静态USD身份，没有使用V1的.3m替换半径。link pose为xyz/xyzw、速度为linearxyz/angularxyz。实际base_link的四元数、位置、线速度、角速度与原root诊断最大差均0。不是读了另一个刚体或按名字猜测。

原B0所有17非raw数组shape/数值精确相同，含raw请求/protocol/smoothed/applied/关节/世界状态。当前raw时钟身份独立保留，不将非raw全同写成所有raw全同。新测量每行与原physics timestamp精确一致、全1536有限且完整。303项preflight suite10.07s；4小fixture真实RED4fail.18s→GREEN4pass.14s，覆盖完整命名/array复制/shape/非有限拒绝和滚动方向/绕worldZ不平移。

## 球心与质心必须分开

现场get_coms的球壳局部质心偏移为(.010809,.020684,.012501)m，距离几何球心.026475m；base_link COM偏移(-.013516,-.0079104,-.028555)m。SDK返回线速度的参考点不能自动认作球心。官方[PhysX ArticulationLink文档](https://nvidia-omniverse.github.io/PhysX/physx/5.1.1/_build/physx/latest/class_px_articulation_link.html)明确COM线速度与actor原点不同；[IsaacLab源码](https://isaac-sim.github.io/IsaacLab/main/_modules/isaaclab/assets/rigid_object/rigid_object_data.html)也区分link pose与COM velocity。本地tensorAPI源码封存，COM实测与差分诊断作为本版本补充支持，不仅凭远程最新文档认证本地版本。

球壳球心速度按刚体点转换：v_center = v_COM − omega_shell × (R_shell * com_offset)。再计算假想最下接触点切向速度 v_bottom = v_center + omega_shell × (0,0,−radius)。球壳绕竖直轴自旋不能直接解释成平移。两种速度点口径都原样报告，没有看完数据挑最好者救gate。

全1535相邻步：SDK线速度对COM位置差分RMS.005952m/s，对球心位置差分RMS.014824m/s；存在位置求解残差，差分不是严格的传感器标签。均值endpoint速度定义只作测量诊断，不是新的在线预测。

## 实际链与条件滚动残差

评估t>=1s且球心高度距半径<=.01m，共1485/1536步。只是高度资格，不把它叫力支持接触判定；所有原始步都保留。

|测量关系|RMS|p90|峰值|
|---|---:|---:|---:|
|实际shell omega − (base omega + R_base*Y*qdot_joint1)|1.9717e-7rad/s|3.1846e-7|7.0895e-7|
|COM→球心转换后的假想底点XY速度|.0008291m/s|.0007828|.0130344|
|误把SDK线速度直接当球心速度|.0104426m/s|.0185688|.0253379|

本次实际编码器/rigid-link角速度链在浮点精度内吻合，之前球壳角速度合成不是主要测量误差来源。近地面条件底点速度大多很小，不能把未转换COM造成的厘米每秒残差都叫滑动。仍有约.013m/s瞬态峰值且无接触力读回，所以不声称严格零滑动或全域验证。

## 姿态积分为何仍会漂

同一实际base_link，1484条件相邻步Log(Rprev^T*Rnext)/dt与begin body gyro残差RMS.011494rad/s、p90.019747、peak.029844；终点gyro换到begin frame仍RMS.008269rad/s。没有发现getter选择错误，SDK读到的body pose和post-solve angular velocity不是每步严格微分关系。

官方[PhysX求解说明](https://nvidia-omniverse.github.io/PhysX/physx/5.4.0/docs/RigidBodyDynamics.html)解释位置几何偏置与速度求解分离，报告速度未必精确复现相邻姿态差。该机制与本次残差一致，**是有文档支持的解释，不是已经记录求解器内部偏置并完成唯一因果证明**。不改solver迭代或physics救结果，不把GT四元数差分直接作为“更好的gyro”递给估计器。

另外仅分解已拒绝旧online姿态误差，body相对旋转向量绝对分量p90 XYZ=3.25466/2.78092/1.36585deg，最终向量−2.66075/−2.13216/−.47726deg。不能把geodesic p90的3.82386deg都称yaw漂移。旧3D/姿态gate不倒改、没有新估计器成绩。

## 实际图与下一项明确工作

[shell/base/encoder与底点速度图](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/shell_link_measurement.png)和[三时刻实际视频拼图](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/shell_link_actual_video_snapshots.png)已打开检查。实际第三/双目视频逐帧解码，各308帧10fps。没有新视觉模型推理或重新标定矩阵；新原始link JSON、COM身份、物理/feedback/两视频、source和postcloseSHA全部保留。

下一项只选择一个传统、物理可解释的姿态消费候选：真实Isaac IMU的gyro+accelerometer补充gyro-only的倾侧/俯仰漂移，禁止读取sensor orientation/GT quaternion作估计器输入。先验证IMU实际输出坐标/时间/重力和只读身份，再预封存唯一固定参数的重力辅助互补滤波；不fit/扫增益/扩大网络。所有预测只读当前及过去IMU+编码器，old数据开发与独立完整fresh序列分开。尚未实现或准入，不能把本报告当改善姿态或完成导航。

视觉地面来源已有明确拒绝和原mask足迹支持上限，仍是单独阻塞。改进位姿不等于修好深度或盲区记忆；下一项完成后仍需回到可证来源和闭环。整体研究未完成，自动保持ACTIVE。

## 独立复核与作者范围裁决

一次fresh只读第二人审查C0/I0/M0；亲自核验4fixture、74source/6baseline/2485postclose/4USD、1536link、17nonraw、COM转换与残差算术、308capture/1535feedback、两完整视频与图。实际进程、RED4fail和303全suite是检查作者日志，未亲自重演。原已审正文按字节封存。作者保留高度资格而非力支持接触、求解器解释非唯一内部因果、原生IMU尚未实现、完整视觉导航未完成四项范围；不再复审或重跑此序列。
