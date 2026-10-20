# 来源筛选与旧RGB障碍记忆接入导航：单候选实际实验

## 本次结论

候选拒绝。原B0观察交接后35秒超时、无接触、距离2.834363m；新来源消费者在29.7秒发生真实接触、距离2.043201m，没有通过障碍X=0。真实目标方向进度−.232717→+.560704m，但不能把前进更多且撞上障碍写成改善或成功。

这是已知仿真位姿、双RGB估计深度、显式平地/起始区域先验和未知探索风险的局部任务。没有准入估计位姿/原生IMU/纯RGB，也没有触发正式E0/E1/E2或修改35秒。原世界模型、来源深度、速度跟踪及位姿拒绝均保留。

## 唯一改变及输入边界

只改变current模式grid的障碍来源消费：当前hazard必须与过去已接受点在.10m内、保留10秒原年龄；额外插入当前双目RGB外观可佐证的旧种子几何。非hazard来源保持；旧点的frame/query/partner/sigma/几何时间保持，当前外观不是新深度或自由空间。before-seed输入透传，18秒由真实五次低速/当前100hazards资格自行种子，单次reset/no内部reset。相机、光照、Fast、遮挡、PID、smoother、supervisor、主轴.3未知风险上限、规划/参考/控制增益和换向低速规则不变。原source另行归档，shadow读取原source不执行。未安装被拒绝的velocity-pursuit。

actual run_qualified_hazard_navigation_v1.py由上一轮observe recovery代码机械派生，逆替换与父代码原字节一致，1283来源SHA运行前核验。status显示02:08:58至02:14:27UTC，退出0、collision不是runtime failure。297请求、298capture、1486物理、60source更新、两路298帧10fps视频。18秒实际交接。原B0复用，未重复；只有一个新科学候选。

## 开发与接口核验

四项RED→GREEN测试覆盖前缀透传、来源筛选、旧sigma/编号/时间、过期/未来输入及实际历史occupied/UNKNOWN语义；整个477测试22.48秒通过。此前已关闭observe源数据71次开发回放先生成所有输出再标签评分；旧外观支持最差283/289=.97924，无有限证明错点，10条标签不足计入分母。不是fresh held-out。首次仅行政封存的SHA函数str类型出错，发生在71次输出和records全部生成后；独立补封存，原脚本/错误保留，没有科学重跑。

新实际输出60次因果重建2220字段精确；120个未来source/appearance请求拒绝，40次截断前缀1480字段精确，不读取renderer标签/不二次调用有状态policy。不是全文件库未来替换。实际请求297块/1485dispatch逐值一致，PID+smoother用实测反馈1480tick、296snapshot精确，最初5tick未克隆；这不是递归物理模型预测。

完整实际原CAD/点云/遮挡重建1320字段、当前+旧点地图1782数组、旧原source-history只读shadow均一致；3746来源/产物身份核验，两路视频均完整。第一次posthoc验证把npz的frame/time零维数组当hash key，报错；v2仅转换Python scalar，保留v1来源/失败记录，无runtime修改或物理重跑。

新插入旧外观点在18.5/19/19.5/20/20.5/22/22.5秒分别300/279/270/272/263/112/6，双目世界正确299/279/270/272/262/112/6；1500/1502=.998668，两条不足计分母，零有限证明错误，每个非空帧≥.95。当前已筛选hazards25713/25770正确，但这不是全部原source或新区域准入，逐帧记录独立保存。60次来源中22.5秒后旧外观输出均空，不能用高conditional精度隐藏消失。

## 对照的因果范围限制

物理和实际请求直到18秒交接均逐值相同（901×17字段）；但原RGB重置序列的来源点早已不同，地图首次差异8.5秒，controller nominal参考状态受其影响。首次实际请求差异18秒，实际556个applied tick改变。以首张不同地图为分叉边界，426×17物理、86执行state、原policy状态与请求一致，但原source数值不同，预注册完整source/control前缀gate失败。

因此动作干预真实存在，但不能把全部进度变化或碰撞归因于唯一来源消费者。物理前缀一致不等于感知或规划前缀一致，不重复基线来救分。原接口记录control_variant=observe指调度前缀，完整候选身份需结合hazard_source_qualification=True、派生代码和新manifest中的qualified_hazard_source_only；旧字节保留，不能只凭名称认控制器身份。

## 有界失败分解

种子1501个hazard仅落在5个grid cells，XY跨度.460×.191m，局部表面窄。23秒起actual map无hazard，原reference guard把零已见hazard的必要筛查当作通过，仍保留参考；它明确certified_safe=False，且原任务允许未知风险探索，所以尚无确定实现bug或安全保证。对本次被拒绝hazard cells核验，没有与floor自由标签重叠（0），但类不应被概括成所有未来数据都无此可能。

在18/20.5/23/28/28.5/29.5秒，实际保留参考的真实occupancy代理suffix最小clearance均.04386m，远低于原.6屏幕规则。实际source grid却均通过。只读把18秒全部未过期旧点加回，18至28秒仍只有5个局部格，而且guard依然通过。因此不能仅把失败归因于23秒丢弃记忆，保留这批旧点也不足以阻止这条路线。28.5秒原10秒TTL才到期，但参考在18秒已会穿过未观测的墙段。闭环没有执行这条假设overlay策略，不能把它称作实际A/B。

实际post-handoff投影误差peak.12376m，临碰撞前.06167m；机器人基本沿已经不安全的参考前进。首接触29.66秒；最终root[-.73930,.12628,.40000]。因此本次最直接的未解决层是局部障碍空间覆盖和未知区域规划，不是单凭终态再认定PID跟不上。网格真值只是事后诊断代理，实际接触由物理contact计分；没有把代理clearance当精确壳接触距离。

预算也应明确：已知静态.6裕量point-grid绕行约7.0627m，观察交接消耗18秒后只剩17秒，名义所需约.41545m/s；这次实测post-handoff XY均值.12595、peak.20550m/s。实测peak或R×.3=.12不是全域速度上界，不能数学证明所有策略不可能；但当前这套观察调度和请求策略缺乏35秒任务可行证据。不得靠延长计分或加速阈值扫描救分。

## 交付与下一项

真实轨迹图qualified_hazard_navigation_result_v1.png已个人查看，原数据、mask、原/消费点云、来源编号、物理/请求/PID、两路视频和版本均保留。source/geometry/interface层通过的局部检查与完整控制/导航拒绝分开；本新候选尚未独立首次审查。不要反复仅改TTL、换成history或继续PID调整来重复这条已知不完整路线。

下一项针对新观测如何准入地图：用因果双目/多时刻RGB确认来允许新增区域，不再永久限制于18秒旧种子的邻域。先在已关闭数据进行一个冻结、来源可追溯的4-view一致性诊断，空间新覆盖和保守精度同时核验，失败不调阈值；输出不能用全图/renderer/未来位姿或模型先验冒充观测。若来源不能形成增量，拒绝该路并明确需要的观测/任务可行条件。估计位姿层仍拒绝，原35秒正式评测未开始，整个研究未完成。


## 2026-10-07T02:37:03.413674+00:00 first independent qualified-hazard review CLOSED
C0/I2/M1: no newly identified Critical issue; I1 collision/navigation rejection and I2 strict source-prefix rejection remain admission blockers. Reviewer personally reproduced60source/2220fields/120future guards/40prefix1480fields;297requests1485dispatch,1480PID ticks296snapshots;native297maps1782arrays/150120age checks; geometry1500/1502 and25713/25770. Same901x17 physical/actionprefixthrough18s does NOT establish sourceprefix: leftRGB andoriginalsource mismatch alreadyframe0. AuthorCAD/fullvideo verification was inspected, not personally rerun byreviewer. Unconditionalunsupported_hazards_unknown flag is INTENT not universalcellinvariant: actual23rejected-cellinstances include0floor and1occupied withindependent surviving support. Rejection creates no newfree observation; do not erase legitimate other supports. No codefix/install/simrepeat/reviewloop; oldbytes/manifests/scores immutable. Independentreport qualified_hazard_independent_review_v1.md. WholeNOTcomplete/formalE0E1E2NOTtriggered.

OldprefixSHA 5ac5c86d42920f3dfb1f7c2f5370e5e690e4e64eba45ddd7600a0814d30e70a6
