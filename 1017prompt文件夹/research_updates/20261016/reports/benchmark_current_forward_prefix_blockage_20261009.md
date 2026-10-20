# 当前前向段阻塞与参考消费候选的拒绝

本轮接续封存E1 continuous-tube运行，未重跑上轮仿真、相机/光照/PID/速度/地图/裕量未改。完成真实执行数据上的CPU因果复现，以及一个局部候选的实现、开发预检查和拒绝。整体视觉导航研究仍未完成，原35秒E0/E1/E2未启动。

## 不可达原因的代码和数据复现

实际frame506/50.6s，原minimum_prefix_transition planner要求真实measured-forward eligibility成立时先连续两条.3m forward primitive，再允许reverse。原四个命令、支持曲率、.05m采样、全图膨胀/crop、节点预算3000都保留。namespace clone只记录原collision函数输入/输出，不修改返回值、不召回有状态policy。

同一当前实际XY/yaw、实际map和局部目标复现unreachable、expanded4：三个首forward .3m全通过；九个第二段全部失败。首次阻塞格中心主要(-1.2,.3)/(-1.2,.2)，最近既有风险格(-.4,.1)，栅格距离.800–.8246m，低于原.870710678m。这不是“第一步就被方向锁存卡住”的证据。

随后frame511/51.1s使用其当时实际状态与eligibility，复现success80 nodes；waypoints与实际保存值逐值一致。这不是从506预测511，也不是许可提前反转。

v2历史地图对照最初同时改变了map-dependent local target(-.9,2.1)→(-.8,2.0)，独立审查指出Important，原v2保留。v3显式固定两组query相同局部目标(-.9,2.1)，仅占据图换旧501，结果仍unreachable4。实际query完全相同和单纯status/expanded一致分开标记。独立复核Important关闭，C0/I0。

对两张图相同9条第二段做解析连接折线的连续必要筛查，最小距离范围.62251–.80118m、全部FAIL。这不是精确曲线扫掠或瞬态轨迹预测，但排除了“仅路径栅格二次量化导致本次前向阻塞”的充分解释。来源点不能因挡路删除，旧margin不放宽。

作者实际查看 `E1_continuous_tube_navigation_run_v1/first_prefix_blockage_v1.png`。图中绿色首段、红色续段、黑色风险格与灰色名义裕量可追溯；不是隐藏真值全图。

## 三个预先选定时刻的必要筛查

固定471/486/501三次原规划边界，当前已知XY/yaw/map分别生成原3×3两段forward命令，不读未来真实反馈。原sampled几何可行对数9/5/0；连续连接折线9/4/0；三次原参考仍retain。这提示旧参考几何与当前任意前向续走职责不同，不能直接认定传统跟踪器bug。

这些时刻属于开发数据。当前真实eligibility也要满足：471、486、501的原measured-first-direction均None，不能把全局三时刻描述直接变成可准入控制动作。

## 唯一候选：实现通过，语义准入拒绝

新增 `current_prefix_feasibility_v1.py` 和 `current_prefix_reference_consumer_v1.py`。仅在原测得forward eligibility成立、已armed且旧管通过时，新增“两段前进至少一对可行”保留条件；失败走原重新规划，不改锁存、五次停稳/反弹清零、动作范围、planner要求或速度/PID。来源与先验语义保留。

测试先出现missing-module RED exit2，随后3个几何/只读/协变测试通过；再验证实际wrapper组合、原planner注入和direction状态不被强改，合计4 passed in1.49s。组合测试明确人工给定eligibility，不能称其501实际eligible或真实控制通过。

历史<=506共有71已armed规划边界，按当前实际状态复原原eligibility（存在真实logged query时逐值匹配），9个边界符合新screen前提；新条件在221/371/376改变保留决定。此前501并不eligible，未改bank/速度域或强制标forward救效果。

进一步核验三个已知参考的实际意图：221原forward前段尚余.443247m，随后已规划reverse；新条件额外要求未请求的.6m forward，因而可能阻止合理换向。371/376的当前同方向已知参考仍约2.0963m，但不能用这两点选择性跳过221或按episode/time加例外。

决定 **REJECT_CURRENT_PREFIX_CONSUMER_SEMANTIC_MISMATCH**。实现4测试通过≠条件合适≠动态/安全/导航通过。prepared driver/stage仅草稿；协议execution_admitted=false，stage在写status/启动子进程前硬拒绝。没有新实际仿真、视频或导航成功率，也未换第二候选/扫阈值。

## 接续

下一项应针对已知参考实际要求的动作前段，而非任意延长的两条forward：在封存221与371时刻，构造当前可知的outgoing参考段/换向意图/当前roll-PID执行状态，检查需要验证的前段在哪里结束、哪些请求仍未知。只读实际接口核验要落成代码和原始输出，不能仅再写“建议定义接口”；不读取未来实际B0请求，不把曲线几何当瞬态执行保证。保留本候选拒绝及先前来源/模型拒绝，不重跑旧Oracle/NN/参数矩阵。

本轮没有必须用户介入的阻塞；自动研究接续保持ACTIVE，整个任务未完成。

## 同轮继续：参考意图接口与当前首tick执行已实际实现

`known_reference_intent_context_v1.py` 把已安装参考的outgoing方向、当前accepted方向、当前投影及该方向段结束点分开；返回已知几何与名义carrier，而future_actual_request_plan明确None。不把nominal waypoint副轴值当roll-PID最终请求，不把剩余参考长度当停止预测。三个RED→GREEN测试通过(.33s)，包括cusp截断、末点无outgoing、输入只读。

实际CPU构建221/371两个Boundary记录：只读当前capture/执行snapshot、当前act之前的policy前缀、过去已安装参考。前缀末act状态与原boundary_before逐值相同，已安装坐标逐值相同；未来policy日志替换/截断结果相同。221输出.443247m后reverse；371为2.096319m到reference终点，next_direction=None。之前审计v1的末点forward字段由独立v2修正，v1保留。这个接口是封存数据重建，未声称真实在线shadow。

进一步区分时刻：current raw request只有当前act完成后才已知。用这两个时刻已给定的当前请求、当时实际执行snapshot和当前bank/关节测量，正式ExecutionClone重建第一tick smoothed/applied；先seal预测，再与dispatch标签对比，两个首tick完全一致。没有使用下一tick物理反馈、未来实际请求或预测另一计划的真实轨迹；不能升级为多步物理终态或导航模型通过。

下一具体入口是参考意图和请求计划之间的缺口：针对冻结known reference/current accepted状态实现独立controller状态克隆，构造当前时刻可知的请求计划，先做当前请求精确重建及future日志poison核验。不能为了构造计划直接喂未来真实关节/动作；若后续physical反馈未知，明确限定预测适用期或拒绝。旧M1/M2及学习候选保持冻结拒绝，不再拟合网络或扩大标定矩阵。


## 同轮继续：完整跟踪器状态克隆与请求语义

实际执行独立CPU前缀重建。v1因临时SimpleNamespace不是正式PolicyObservation退出1，日志/原代码保留；v2仅换正式观察类型后，221前106次、371前256次跟踪器调用的uncapped请求和方向/计数/游标状态全部精确匹配。没有二次调用实际运行policy。

最终v3将当前policy行完全移出构造输入：过去安装参考、过去调用记录、当前测量进入独立跟踪器；deepcopy包含整个tracker.__dict__（映射、drive_model、参考和全部锁存/计数），数组变更隔离核验通过。当前clone请求和状态一致；future policy日志替换/截断一致。预测文件和prelabel seal写完后，才读当前原输出作验证，221/371两点请求/状态精确，实际两点retain使固定已知参考假设成立。这里只重建固定参考tracker，**没有重建整套policy规划边界**，不得把当前不重规划假设推广全域。v3/v4均实际退出0，无新物理运行。

v4只增加当前clone真实中间量，保持v3原字节。221 waypoint的名义aux carrier为0，但实际反馈合成请求曲率+.836289/m；371为+1.732311/m。两个当前speed为.164213/.160440m/s，BankMap都标记speed_clamped和curvature_clamped，最终当前请求为主轴+.5rad/s、副轴−30°。这证明参考carrier不是实际roll-PID请求；并不新证明映射有bug或获得可行物理轨迹。

构造的因果计划仅限当前action block已知的五组请求（.1秒）；没有自动把它延长成后续B0请求。下一块的速度/位置/关节、停稳资格与可能的重新规划未知，旧物理模型仍拒绝。因此**多步参考跟随请求计划及物理轨迹预测尚未准入**，不能继续用任意.6m前进或名义圆弧来拦截控制。已拒绝current-prefix候选保持execution_admitted=false，未产生新的导航结果。


独立复核controller clone：Critical/Important 0。计数澄清：221为105次过去调用+当前一次=106；371为255+当前一次=256。完整policy JSON预先加载，但当前label不进入producer，seal后才取当前label比较；这是历史数据流隔离，不能称独立在线文件读取隔离。v3/v4源码和输出保持原字节。

随后实际CPU完成唯一必要支持域筛查 `current_block_response_support_v1.json`：两个当前状态对既有positive/negative完整响应都不适用，原inner_config身份、速度/角速度/关节/倾侧/PID/smoother等存在拒绝项，当前+.5/−30°请求也与响应库首请求不相同。原response是不可切开的12秒段，当前仅已知.1秒，不可截取响应或拼不同初态。原门槛/库SHA没有修改，无新标定、拟合或仿真。

具体后续入口：对已封存221/371当前五请求block，以当前snapshot进行正式执行链重建（仅当前已知请求；实测后续反馈只能作为验证标签），然后把V1名义扫掠与这一个block实测XY分开报告预测偏差及适用限制。没有准入物理模型前不把名义轨迹加到控制veto，不重新启动已拒绝候选，不重跑54.6秒。
