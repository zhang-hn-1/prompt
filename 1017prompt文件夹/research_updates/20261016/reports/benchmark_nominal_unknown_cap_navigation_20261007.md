# 单一主轴未知区限速恢复：实际局部到达与冻结 gate 拒绝

真实双 RGB + CAD 自遮挡过滤 + 10 秒显式空间记忆 + 已知位姿，在同一 2 米局部任务达到目标：12.1 秒，终点距离 0.195381984 米，零接触；原 .3 上限基线为 22.6 秒、0.193028772 米。**局部任务通过，但候选准入拒绝**：原先明确冻结的终点距离不退步条件失败，差 0.002353212 米。没有触发完整 6.6 米导航、正式 E0/E1/E2、新成功率或 RGB 估计位姿导航。整体研究未完成。

## 唯一改变与身份

spec/plan `2026-10-07-nominal-unknown-cap-design.md` / `2026-10-07-nominal-unknown-cap.md` 在实际运行前冻结。未知区 final main request 上限由 .3 改为现有调度器的默认 1.0 rad/s；不扫另一个值。父控制器只调用一次，从当次保存的 uncapped request 取值，只改主轴五项，副轴、零请求、观察、五次低速确认/反弹清零、参考、BankMap、PID、smoother/supervisor、增益、运动时限、相机与光照均未改。网络仍 frozen Fast，模型输入没有 CAD 改图，CAD 仅过滤网络输出端点。

此前有限预算核验：166 运动请求原名义 .5/1/1.5 全被外层裁为 .3；161/166 实际 speed 查询低于 BankMap 旧实测范围。这个 evidence 支持测试一个执行消费环节，不证明全域动力学、导航或安全。地面/安全起步区仍明确 task prior；未知风险探索是用户授权的新任务条件，不冒充已观察 free。

代码 `nominal_unknown_cap.py`、机械派生 `build_nominal_cap_runner.py` / `run_nominal_cap_navigation.py`，1272 项源/配置/继承文件预封存，旧仓库只读。driver `run_nominal_cap_navigation_stage.py`，UTC 2026-10-06 20:55:34.076495→21:00:40.136742 exit0，唯一物理序列，没有重试。实际 121 act、122 capture、606 physics、25 推理，两部完整 122 帧/10fps 视频。网络推理及 CPU provenance 重建使 wall-clock 约 306 秒；这不是实时性能通过。

## 同流只读对照与实际动作

primary 使用 cap1/history；独立 slow shadow 使用 cap.3/history；两者同一实时 masked source、同一真实反馈。每次在线断言父控制器 uncapped request、方向/跟踪状态、参考几何、地图 labels/floor/hazard 时刻一致，共121次。shadow 后实际 policy request、执行状态与物理读回不变；没有完整物理 shadow-off 实验。shadow 没有执行，不能把实际 fast 轨迹作为 slow 计划的未来真值。

同流 shadow 原始请求差异54次，首帧60，即6秒正常 tracking 事件开始；同流父控制器/参考/地图一致支撑“最终 cap 是这个源流上的唯一请求差异”。与之前独立实际 .3/history 基线比较，首请求分叉也是60帧，前301物理采样×17非raw字段和61执行快照精确；共同121 blocks内61请求不同、275 applied ticks不同。因此是真实执行改变。两个重置运行的 RGB 不保证逐字相同，不能从整体时间差证明没有渲染混杂；同流 shadow 只证明消费环节归因，不替代慢速另类闭环。

## 独立几何评估及原准入裁决

以冻结 start→goal 直线评价真实 XY，不读取候选 path_index。真实沿线进度1.813267630米，实际路程1.888447747米。运动期从首非零请求帧60到目标：横向 RMS .023872693米、peak .057490362米，通过原 .12/.30 门槛；包括观察期的 RMS .017673828 另列，不稀释运动期 gate。空间进度 .5/1/1.5米首次通过时间8.6/9.9/11.3秒。

| 冻结条件 | 实测 | 裁决 |
|---|---:|---|
| ≤35秒进入 .2米目标、零接触/无运行错 | 12.1秒/.195381984/0 | 局部任务通过 |
| 时间不慢于22.6秒 | 12.1秒 | 通过 |
| 终距不大于.193028772米（仅数值1e-6容差） | .195381984米 | **失败** |
| 运动期 RMS≤.12、peak≤.30米 | .023873/.057490米 | 通过 |
| 同流请求与实际 applied 真正改变 | 54请求/275 actual ticks | 通过 |

**拒绝这个候选的完整目标准入，保留 gate，不更换 cap、不放宽终距、不延时救分。** 区分局部任务 pass 与候选 no-regression fail。source、旧 M2/世界模型和 estimated-pose gate 不倒改。

有限终点采样原因核验：两条真实序列均在第一10Hz采样进入 .2米后结束。新序列末两点 .227557935→.195381984，单block减距 .032175951；旧序列 .204139584→.193028772，减距 .011110811。末五点均向目标接近。它支持“首入目标离散采样造成终距排序差异”这一解释，不能确定唯一原因，也不证明新控制在完整障碍参考上更好。没有插值重评分、继续执行或事后改目标资格；原 no-regression 仍失败。

## 来源、测试与作者证据

原网络输出旧mask66082点，CAD源64114点；去1968，其中1355≤.1米误差、456有限错误、157真值无效。不能称删点全错误或 depth gate 通过。renderer 深度/位姿标签只在 source 包确定后用于 offline 验证；预测/决策没有未来实测量。记忆峰31历史 cells，不宣称记忆改善。

4新增 cap fixture真实 import RED→GREEN，另2记忆/1预调用无突变回归共7pass2.62秒（本阶段旧日志）；继承完整379 suite是前一阶段证据，没有重称本次重跑。author auditor `assess_nominal_cap_navigation.py` 实际复算550包/mask字段、726地图数组、1452同流slow-history诊断字段，future-payload trap通过；1272 preseal+910 postclose=2182 SHA，两视频逐帧解码各122帧，曝光delta0。完整 `nominal_cap_navigation_assessment.json`、raw目录 `nominal_cap_history_run/`、状态/log保留，图 `nominal_cap_navigation_actual_and_shadow.png` 已实际查看。

本段是作者个人实际 audit/看图；GPU/7tests是本作者先前执行证据。独立评审范围另署，不把只读评审称第二次运行。严格来源拒绝与初始prior区别保持。没有 SPL/global-reference 评估，旧metrics里的reference_length2/SPL仅局部harness metadata，不能作为导航比较成绩。

## 接续

不绕过失败准入去跑 fullgoal。同源控制链证据已足够，接续一个独立 RGB-D 位姿候选：复用本次完整移动数据、冻结 Fast 深度与 CAD mask、固定 ORB/旧PnP gates，预测只读像素和深度/当前关节生成mask，无世界pose输入。先历史整序列诊断，不称fresh/在线；通过才另封存实际在线只读整序列。旧 stereo-ORB/rolling/IMU 拒绝保留，无 M3/网络/标定矩阵或阈值扫。spec/plan `2026-10-07-cad-masked-rgbd-odometry*` 已冻结，TDD11（4新增+7旧PnP）pass.29秒，实际CPU预测入口下一项。

## ONE 独立终审与作者附注
新 gpt-6-astra/high 只读终审 C0/I0/M1：个人4fixture1.74秒、逆派生runner/stage精确、27源+10产物SHA、121请求/cap/时间/parentraw/shadow标志、605真实raw targets、301×17物理前缀/61exec、54shadow/61actual request/275applied差异、运动分母62与gate拒绝。未重跑GPU/全部2182SHA/全地图或视频；原作者证据和评审个人证据分开。Minor：sealed protocol继承 unknown_request_main_cap_rad_s=.3字段含义模糊；它表示旧基线/slowshadow，唯一candidate显式1.0字段与代码/实际数据正确。本附注明确，不修改原1272封存协议字节或重跑。完整审查保存nominal_cap_review*.md/json。冻结候选拒绝保留。
