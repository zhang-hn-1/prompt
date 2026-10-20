# 局部规划初始方向代价：单点受控对照

2026-10-05 06:44UTC heartbeat。**当前方向确实改变名义路线选择，但没有消除前后换向。** 本轮完成一次已归档17.5s输入的几何对照，无新物理运行、控制A/B、视觉推理或训练；原35s导航失败与所有旧拒绝结论保持。全研究未完成，automation ACTIVE。

## 对照结果

机器人当时正前进，旧规划器同时以零成本播种forward/reverse根；请求对象没有当前方向。本轮独立版本让与当前接受方向相反的根承担原有.5m切换代价，保留两根、原reverse multiplier1.25及所有其他规划参数。不是强制forward，不提高惩罚，不把.5m解释为真实制动时间。

| 同地图、目标、XY/yaw、原3000节点预算 | 原规划 | 当前方向根代价候选 |
|---|---:|---:|
| 结果 | success | success |
| 首primitive | reverse | forward |
| 前三个primitive方向 | R→F→F | F→R→F |
| nominal primitive距离 m | 3.3 | 3.0 |
| primitive数 | 11 | 10 |
| 原结果内部换向数 | 1 | 2 |
| 含当前forward→首次primitive的总换向数 | **2** | **2** |
| 统一当前方向代价 m | 4.375 | 4.075 |
| 原始无初始方向代价 m | 3.875 | 4.075 |
| 节点展开 | 1821 | 2434 |
| 终点距同local target m | .174093 | .018737 |
| nominal .05m采样原栅格最小净距 m | .632456 | .600000 |
| 当前整installed polyline连续净距 m | .642272 | .614018 |
| 原guard整polyline栅格最小净距 m | .583095（不足.6） | .600000（通过） |

统一目标函数是各primitive距离（reverse乘1.25）加初始及内部方向切换各.5m。原参考10前向+1反向：距离成本3.375，初始1次+.5、内部1次+.5=4.375。候选9前向+1反向：3.075+内部2次×.5=4.075。没有把原direction_changes=1倒改成2；新total separately包括当前方向交接。结果是已运行的确定性搜索选择，未证明Hybrid全局最优。

旧初始反向.3m，候选先前向.3m、反向.3m再前向；**两条仍包含两次潜在制动交接**。拒绝“补初始方向就消除换向/已解决超时”的解释，不推断制动耗时已减少。候选曲线更短且当前polyline guard通过，足以作为下一项独立单次物理对照的名义候选；尚不接受为导航改进或正式baseline。

![同输入名义路线](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/direction_root_cost_comparison.png)

图已实际检查。两线均为名义规划曲线，箭头为primitive几何运动方向；B1/B2、C1/C2是第一/第二primitive终点。紫格仅当时来源地图中的hazard格中心，不是全场景真值。没有候选实际轨迹。原真实视频仍见 [上一轮第三视角](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/guarded_reference_run/third_person.mp4) 与 [双目视角](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/guarded_reference_run/stereo_view.mp4)，它们不作为这条候选的真实标签。

## 五层方向分别是什么

固定frame175，实际clock17.49999960884452s；当前XY=(-.8794614,-2.3945336)、root yaw=-1.21161146rad。

- **当前接受的执行方向**：规划前tracker latch为forward，pending=None、low count0；前一实际记录也forward。这是唯一新增模型输入。
- **参考希望方向**：原新参考首primitive为reverse，随后tracker当前call desired=reverse；该值仅离线报告，不反馈给候选搜索。
- **请求方向**：当前原policy发双字段0，source=direction_change_braking。零请求是制动请求，不叫已实际倒车。
- **当前applied主轴**：+.69999999rad/s，副轴-.24036598rad。它是当前capture已有执行状态，不是后续标签。
- **实际运动**：主轴joint1实测+.87669498rad/s；世界XY速度(.11611456,-.31868365)m/s，course=-1.22138917rad，沿当前root朝向投影+.339162m/s。此刻前进证据与latch一致；不把latch当通用物理方向测量，近零/侧向状态下一般不能由它代表所有运动。

没有用未来applied、关节/速度或后续planner输出作为模型输入。小型snapshot只有当前来源map七字段、当前frame/time/XY/yaw/pointgoal/accepted_direction；源json/episode中其他当前描述量单独写provenance，预测接口不接收它们。既有整个episode用于归档身份，不向搜索提供未来数组。

## 实际运行与版本

完整读取mission317行、主计划276行、ledger426行、最新guarded报告84行及其冻结spec/plan。核验前轮86源/2834旧输入/2790阶段产物/5文档SHA均一致，唯一旧实际run finished/exit0，没有待续仿真。原项目只读，已有H实验隔离/unborn master保留；用户独立执行授权取代重复审批，保留产物，不作无关Git提交或清理。

新spec/plan为 `docs/superpowers/{specs,plans}/2026-10-05-direction-root-cost*.md`。`direction_root_planning.py` 是原kinodynamic_planning完整独立副本：机械相对import→绝对benchmark.drive_model；仅增加initial_direction参数/枚举校验、root g和frontier priority。完整差异在 `direction_root_cost_source_diff.patch`，原dataclass字段/defaults、edge costs、42模板、heuristic、碰撞采样、重建和nodebudget规则保持。None分支精确回归旧行为；两根仍允许，零原switchcost不改变选择。

冻结地图来自双方rendererdepth+knownpose来源记忆，仍是特权unknown-risk诊断，不是RGB/估计位姿。无脚下free seed、全图/全局参考/未来真反馈输入，原相机X.20/Y±.30/Z.05、灯光、PID、调度/速度、protocol/smoother、原五低速与反弹不变。本轮根本没有调用policy.act、安装候选参考或改变实际动作。

**封存06:55:21.321515UTC，94源/5630旧与当前输入身份；实际06:55:35.435283—06:55:41.269461UTC完成，exit0。** 一个当前frame问题：1次原grid A*构造同route/2.4m target及±4m crop；2次Hybrid分别原baseline/唯一候选。原baseline完整pose/direction/command、route/target及1821节点逐值精确归档；候选2434≤3000。没有额外帧、Hybrid/grid扫描、矩阵重做、额外物理序列/模型或新视频。fixture中的玩具地图搜索单独属于CPU测试，不能计作新场景实验。

## 核验、门槛和范围

事前接口gate要求原baseline精确、当前方向来源/净化/源身份、地图不改和原预算一致；均通过。事前nominal eligibility要求两条success、首实际primitive方向改变及原nominal栅格/终点容差；**通过**。这是未来单次控制试验的必要名义证据，不是物理性能gate。原真实35s结果仍timeout/距离4.156980m/0contact；没有新的导航成功率。

作者06:59:34.658791UTC无search核验94源/5630输入、dataclass字段/defaults和已封存source diff；21个primitive/147个含重复边界采样点以原V1曲率重算，最大端点差<1e-15，原inflated raster全部通过、同goal .2m内。统一目标成本独立按正负command手算，初始与内部换向分开。额外future_feedback payload陷阱被严格字段检查拒绝、未解引用。不是未来完整日志替换控制回放/在线接口证明，没有把未运行的在线消费者写成通过。

独立polyline guard描述核验与nominal primitive不同：旧原primitive栅格通过，整折线guard的rint采样却到.583095m；候选为.6m。连续两者通过；没有借此认定旧planner实现bug、真机危险或改变任何.6口径。作者验证和画图分别保存两组guard测量，guard只测当前时刻，后续地图/运动可令其失效。nominal V1仍未验证任意当前速度/bank/PID的有限物理响应，候选距hazard更近也不认证动态/3D或unknown安全。

16项真实行为fixture先全部缺功能RED/1.18s→GREEN/1.36s；完整H **178测试通过/7.01s**。包括手算初始/内部代价、None完整旧规划对照、零代价、必要倒车不被禁、坏方向提前拒绝、当前snapshot拷贝/严格字段/未来时间与payload拒绝。两个作者命令错误把工作目录和相对路径重复，均在文件创建前失败，file-not-found日志分别保留，纠正后真正16项RED和完整科学运行只执行一次；无数据后改模型/gate/重搜。绘图缓存只在/tmp自动回退，图正常生成，不是实验失败。

所有新代码、协议、同输入快照、两完整曲线、请求、旧输入来源、实际status/log、测试/失败、核验和图均在H；assessment保留当时pending字段，verification单列最新门槛，不倒改旧数组/历史报告。最后只读独立审查及作者身份/文档封存另记录。

## 接续边界

下一项若开展控制，仅实现一个以当下tracker accepted_direction调用新root-cost搜索的独立局部consumer，保留上一轮guarded-reference策略的所有其他行为。每次原边界rebuild使用当前latch，retain时不搜索/不reset；不按episode ID/17.5s强制触发、不删除倒车、不同时改PID/速度/goal/map/heuristic/primitive。先CPU核验真实类/请求转换/保留状态/五次资格与反弹，再预封存最多一条同起点原35s goldenE1对归档guarded B0。验收仍真实目标、独立固定参考进度/横向/空间检查点/接触与完整动作链；不能以换向数内部口径、polyline短或首方向一致作实际成功。

这是仍有两次换向的有限候选，失败直接拒绝，不追加penalty/nodebudget/目标距离/速度扫描。新动作前不存在候选实际轨迹，不能以原B0的17.5—23s物理记录充当其标签。来源可靠性、RGB+估计位姿、唯一学习记忆/世界模型增量及正式35sE0/E1/E2仍待分层完成，研究ACTIVE。

## 独立审查与作者收尾

以下是作者在独立审查完成后追加的收尾，不属于已审正文。ONE只读审查 `/root/direction_root_cost_review`：Critical 0、Important 0、Minor 1，接受本次有界名义阶段；审查正文SHA `71400a7d86431fd5e4212b0528d31ed8cad44f4053f990b7af4cd53d5a8fcd92` 保持不变。审查原文、独立与作者检查的区分、每条范围裁决完整保存在 `direction_root_cost_review.json`。

审查者亲自核验94源/5630输入SHA、精确diff、三个dataclass字段/类型/defaults、frame175输入及规划前latch、整条原baseline归档一致，独立重建21primitive/147采样及代价/终点/栅格/连续几何。仅运行获准的无search fixtures：11通过、5未选、0.97s；看过图，无文件/Git改动，无新增搜索/回放/仿真/推理/训练。作者178项全量测试与原一次科学运行属于被审查的作者证据，不能冒称审查者重跑。

Minor已接受：`objective_summary` 对合法的“起点已在目标容差内、只有终态waypoint”success会抛异常，需在一般复用前返回零代价/零换向/无首方向，并保留真实primitive中零指令的拒绝。当前两条非空参考不受影响；本轮保持封存源码，下一次一般复用前用literal零运动fixture和独立版本修复，不为此重复本次搜索或修改历史结果。

对全部 declined 范围逐项裁决：实际候选导航/制动/PID/有限执行与动态3D安全没有数据，保持未测和旧35s拒绝；在线consumer/状态保持尚未实现，下一阶段先CPU验证；RGB/估计位姿/学习增量/正式E0E1E2仍未准入；不声明一般planner正确或全局最优，仅限本问题和root diff；全研究未完成、automation ACTIVE；作者收尾与文档同步以随后新鲜hash/gate核验负责，不算独立审查追加内容。以上均不阻塞本次名义接口/几何阶段，但不能被省略为后续导航通过。

最终封存包含源码/协议/两曲线/快照/实际运行状态/测试和失败日志/独立与作者核验/图/报告身份，更新mission、主计划和ledger。下一项仅为事前独立封存的控制consumer与至多一条原35s同条件实际对照。本heartbeat不再执行新的科学查询或控制运行。
