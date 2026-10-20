### Strengths

本次仅审查独立理想滚动里程计阶段。完整阅读了审查包、对应spec/plan/ledger/report及八个实现文件；以归档SHA边界核对，没有构造不存在的Git提交区间，也没有重审旧空间记忆/VO阶段。

- 传感器模拟与估计器边界清晰。逐帧GT quaternion只参与理想body gyro合成；估计器只接受时间、gyro、实际joint1速度，初始法向只提交一次。更新前验证和输出复制避免了检查到的状态污染。
- 在线hook先调用父类物理记录，再读取当前末行；源码中的policy调用一次，估计器返回值不进入控制。记录有1536组一一对应的sensor/prediction/physics样本。
- 原三维门槛保持不变。在线位置RMS与姿态p90失败，XY和GT-attitude结果明确限于诊断，没有借其恢复准入，也没有将Oracle成功表述为估计位姿闭环成功。
- TDD偏离如实记录：前三个shadow fixture首次GREEN，后续坏初始输入有独立RED→GREEN；没有把全部实现包装成完整TDD。
- root pose和angular velocity的SDK路径核对符合报告：有效physics handle时均从physics view读取；没有充分证据把漂移归因于缓存或时间戳错误，报告保留了这项不确定性。

### Issues

#### Critical — 无

#### Important — 无

#### Minor

1. **部分测试名称承诺了超出实际检查的边界。**  
   位置：[test_rolling_shadow.py:16](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/test_rolling_shadow.py:16)、[test_rolling_shadow.py:26](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/test_rolling_shadow.py:26)。  
   “shadow on/off”测试没有运行父diagnostic或on/off两条调用路径，requests数组也未传给被测对象；“future stream”测试对两个实例提供相同prefix，没有可访问的不同未来后缀。它们实际验证的是局部输入不变性、确定性及白名单拒绝，不能单独证明名称所述的完整集成性质。当前实际B0数组对照和源码检查补足了本次运行证据，因此不阻塞负结果归档。后续复用时应准确改名，或添加真实父hook和不同后缀场景。

2. **预期的SO(3)及bank投影fixture覆盖不完整。**  
   位置：[test_rolling_odometry.py:10](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/test_rolling_odometry.py:10)。  
   现有测试覆盖单轴yaw和更换初始normal，未直接覆盖spec所列SO(3) norm/inverse及动态bank后编码器轴投影。未来若改动矩阵乘法方向、旋转轴或归一化，现有fixture可能漏检。建议在下一次修改该估计器时补充非共线旋转、逆旋转及非零bank的解析期望值；不必为本次已拒绝候选追加物理运行。

### Evidence

**我亲自完成的检查：**

- 仅运行指定9项局部fixture：**9 passed in 0.21s**，关闭字节码及pytest缓存写入。
- 历史协议8项source、4项prediction input、2项evaluation input；在线协议67项source、5项baseline；4个USD层、3项post-seal来源及诊断来源/输入哈希全部匹配。
- 核对2483项post-close清单数量，并抽查其中19个运行根目录产物哈希，全部匹配；未宣称亲自重算全部2483项。
- 报告SHA匹配，长度8553字节。
- 独立核算历史sensor与GT当前行的合成关系、实际编码器、初始法向，以及两条历史评分：850/1550评估样本，均复现报告。
- 独立核算在线1485评估样本：位置RMS **0.057997866 m**、peak **0.080070635 m**、姿态p90 **3.823861°**；确认原门槛失败。
- 17项B0非初始化时钟字段数组逐项精确一致；归档identity为307次policy调用、308次capture、一次reset。
- 查看两张视频帧拼图及诊断图，所示自遮挡和失败趋势与报告一致。

**仅核对作者归档证据、未重新执行的事项：**

实际仿真、唯一运行预算与运行时间、277项全套测试、全部视频逐帧解码、三种完整积分诊断、GT-attitude上限诊断和静态USD结构读回。相关日志/源码/记录支持报告；本次没有重跑这些过程。

### Recommendations

将两项Minor作为后续测试维护项单独记录。保持现有失败结果和冻结来源，不为修饰测试覆盖而重新运行候选或更改门槛。

### Declined to judge

- 旧空间记忆、ORB/VO阶段正确性：明确排除在本次范围之外。
- 真实IMU噪声、偏置及标定表现：当前仅是理想仿真传感器。
- 每条历史序列各自的现场DOF名称：原记录不存在；接受报告明示的历史身份限制，不追认为现场认证。
- 精确物理漂移原因及全程无滑动：缺少solver校正、独立shell角速度和接触滑速证据。
- 全部视频逐帧内容及全部2483项产物重验：沿用作者归档验证，本次只查看拼图并选择性核对哈希。
- 导航准入、跨环境泛化及正式E0/E1/E2：本阶段未实施，不能从Oracle复现推导。
- 完整研究任务完成状态：明确仍未完成。

### Assessment

**Ready to merge? Yes — 仅限作为本阶段负结果与诊断证据归档。**

未发现需要阻止归档的Critical/Important问题。**在线V1仍被拒绝，不能接入导航；本次审查通过不构成候选准入或整体研究完成。**
