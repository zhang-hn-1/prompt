# 球型机器人：已有时间规划来源与唯一代价候选（2026-10-05 14:47UTC）

本轮完成源码身份核验、真正消耗时间权重的规划器实现、CPU回归和一次封存快照搜索。**候选在原3000节点预算内未找到完整局部路径，拒绝准入；没有触发新的物理导航。整个双目记忆导航研究未完成，自动续做保持ACTIVE。**

## 1. 原始结论与研究层次

固定ball_stereo_front020_up005_v2相机、照明、现有roll PID/速度策略、来源地图、方向五次低速确认/反弹清零、guard及原35秒规则。所有旧实验、原RGB/来源消费/M2/停止包络拒绝、13:47共同域曲率裁剪候选拒绝均保留。此次未拟合网络、未重采标定矩阵、未改变任何实际动作。没有新的导航成功率。

已有endpoint导航实际35秒仍timeout：goal剩3.961658m，独立进度5.028426m，RMS.171278/peak.384271m，0接触。那条实际曲线和视频只作历史上下文，不能给本次另一条名义计划当未来物理标签。本轮用原frame175的17.4999996088s状态作为当前，局部输入仅截至该时刻的来源map、XY/yaw/pointgoal/accepted-forward。完整execution175身份已归档，但本候选不读取未来execution/applied/XY。

## 2. 已有成功Oracle到底用了什么

直接阅读v9/hybrid/time_planning.py、hybrid_planning.py、run_navigation_worker.py与run_hybrid_golden.py。独立time_cost_source_audit.json的历史manifest实际为14项：12项当前SHA相同，isaac_runtime.py与run_pointgoal_benchmark.py两核心当前不同；9个hybrid wrapper全部同历史身份。不能凭“Oracle/B0”名称认证整套历史runtime可直接移植。未import执行仿真worker。

历史worker在episode初始以完整占据图求原计划和time plan，修改runtime规划入口；和当前来源局部map→2.4m目标→±4m裁图→direction_root planner的在线consumer不同。历史time planner把edge变成L/max(.05,V1speed)，heuristic除以V1最大速度、switch成本也除以最大速度；另有**abs(V1曲率)>.75直接排除template**，且没有当前accepted-root参数。直接搬来会同时改变代价、运动曲段和方向接口。

历史chooser计算waypoint弦长/V1速度+每内部换向1秒，以35秒减固定10秒reserve得25秒门槛。原计划满足25秒时优先原计划；否则任何success的time plan都会被选中，未再次要求它也满足25秒。用实际函数的字面单位速度fixture：原26/候选28秒会选28；原20/候选18会保留20。只是来源语义见证，不是物理数据。该函数运行在episode初始化，本轮没有把它的剩余预算缺口称作当前runtime bug。

V1静态payload当前身份明确，历史2026-09-12测量生产者源码身份仍未认证。BankMap的body命名曲率是XY几何曲率乘主轴方向符号，非Euler yaw/arc；low/high共同数值域不是动态可执行证明。13:47的生产者定义审计继续保留，不倒改。

## 3. 唯一候选与测试

nominal_time_cost.py只计算一个基边项：

`base_m = L * vmax / max(0.05, V1speed(main, aux))`

vmax为冻结V1 steering表最大速度0.58097705090496m/s。目标函数保持归一化米单位；reverse×1.25、换向/起始方向不同的.5m惩罚原样。总成本除vmax是一个加权名义时间代理，而非实际截止时间预测。实际42template速度均>.05；.05是历史实现floor，不是新调参值。

time_cost_planning.py是冻结direction_root_planning的机械副本：移除重复dataclass定义并alias原request/waypoint/result类型，加入vmax及基边调用。42运动曲段、原V1曲率、采样、heuristic、closed-set、优先队列顺序、反向与方向成本、节点预算完全保持。没有安装上一轮曲率cap、历史.75筛选、新的speed策略，未改变tracker。

字面成本/非法输入以及真实两段直线CPUfixture先RED：距离-only scaffold 9fail/1pass .31s；正确消费后10pass .26s。fixture原tie选择main1.5，新cost选择名义更快的main.5，同时端点仍(0,0)→(.3,0)→(.6,0)。这是小型人工模型上的实际搜索行为，不是物理快慢标签。完整实验区237pass 9.44s。fixture搜索与本次唯一实际快照搜索分开计数。

## 4. 封存与唯一实际快照结果

162sources/14200历史及当前inputs先封存于time_cost_protocol.json；旧可变主文档SHA解析到前阶段既有归档，当前mission/plan/ledger单独存pre-seal原字节。协议接受条件先定：success/终点.2/0raster坏段，路径及原距离objective各至多B0+.3m，内部换向不增加，所有名义曲段在两层BankMap共同域且比原7个域外少，加权时间代理至少快.1秒。均为以后物理候选的必要CPU条件，不是safe/RGB/navigation准入。

一次审计14:59:56.519387→15:00:02.241984UTC exit0；唯一candidate Hybrid搜索14:59:58.491629→15:00:02.239808。1 shared grid request、1 new candidate search、0 original target replay、0 policy.act、0物理序列、0拟合。归档原计划与实际reference175 pose/command/direction/2434节点精确核验。

| 项目 | 归档原计划 | 唯一时间代价候选 |
|---|---:|---:|
| 返回状态 | success | node_budget_exhausted |
| 展开节点 | 2434 | 3000 |
| 完整primitive数 | 10 | 不可报告 |
| 路径长度 | 3.0m | 无完整路径 |
| 原距离objective | 4.075m | 不可估计 |
| 内部换向 | 2 | 不可估计 |
| 共同域外曲段 | 7 | 不可估计 |
| 到相同局部目标误差 | .018737m | 不可估计 |
| 加权名义arc-time | 13.693638s | 不可估计 |

候选空结果中的程序占位path_length=0/direction_changes=0不是成功指标。success相关所有条件不成立，**CPU资格REJECT**。不是证明不存在路径，也未证明更多节点会通过；不增加预算重搜、不换heuristic救分、不扫代价或安装到runtime。

原计划另有未加权arc代理11.580005s、历史chooser弦长+换向代理13.549481s；三者职责不同，不能互换。当前episode剩17.500000391s；这里比较的仅2.4m局部目标，不是完整全球终点，甚至代理小于剩余时间也不能宣称整条路线可在35秒完成。真实制动、初态速度/bank/执行延迟、PID跟踪偏差均未由代理预测。因此“原失败主要因为缺时间代价”本轮仍没有证据成立。

![唯一当前快照与归档原名义路径，候选无完整曲线](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/time_cost_comparison.png)

图为已观测hazard/同一起点/同目标/原名义计划。红色图例表示候选失败，不画不存在的路线或实际未来轨迹。无新视频；旧endpoint实际双视频仍在endpoint_navigation_run作为上下文。

## 5. 作者验证、失败记录与范围

作者15:01:54.224984UTC重新核验162源/14200inputs SHA、封存/开始/完成时序、原参考/42template几何、原类型alias、机械diff、三种时间代理代数和测试日志。验证没有重新搜索、推理、物理或完整测试矩阵。assessment SHA4ef9e4797863e6455643927be2196d1976dac678a571ac0b6ffec7499152cadb。PNG已逐像素显示检查布局与图例。

作者verifier v1导入不存在的未使用load_drive_model而退出1；保留verify_time_cost.py/time_cost_verify_v1_failure.log，独立v2仅删该未用import，实际PASS。这不是候选科学失败的第二次搜索，也没有修改sealed来源/协议/结果/门槛。matplotlib因host配置目录只读自动使用/tmp缓存，图正常生成，未改源码或数据。ledger临时将历史manifest写为15/13，下一行校正为实际14/12并保留原记录。

本层实现/CPU测试PASS；当前目标搜索/CPU资格REJECT；实际控制、完整参考、正式35秒导航均**未触发**。前层测试通过不代表后层通过。全研究NOTcomplete，automation ACTIVE，不通知项目已完成。

## 6. 后续工作边界

关闭这次time-cost候选，不继续在同一175案例增加节点/改heuristic救分。下一项应核验**原V1名义曲率/速度与实际roll-PID请求链所代表的运动是否同一系统**：以已有BankMap完整生产者和实际endpoint/开放平面数据为主，区分原副轴角命令、编码bank参考、真正测得bank、XY速度与稳态条件，确定一项具体未覆盖的闭环响应。如果确有缺口，只预封存一条必要的完整roll-PID运动序列；不重做整矩阵，不以Euler yaw替代XY几何。随后只选一个有测量依据的名义响应来源消费改动，而非继续静态代价/速度阈值扫描；不能把旧cap拒绝改为接受，也不把静态BankMap域当动态物理上限。

可靠RGB来源消费、估计位姿、唯一因果学习记忆/世界模型增量及原35秒E0/E1/E2仍待分层验证。既有Fast直接融合、因果多视角、ORB来源拒绝不倒改；不得重复把同一旧RGB矩阵重新累积当新成果，也不拿先验冒充观测。


## 作者收尾附录：只读审查与逐项范围裁决

审查正文原SHA e4dc994b4427aa0ea8a3836f8d193e395fde0664c96b1c42e345305e153c0874。以下为作者在审查完成后追加，不冒称二次审查。执行计划要求的fresh-context dispatch实际被agent thread limit拒绝；本次实际ONE第二人审查使用前阶段nominal_support_review的保留上下文，明确不称fresh。原请求package、dispatch失败记录、源码/数据均保留，不静默改写。

审查0Critical/0Important/0Minor；个人核162source/14200inputs/7review-source/20artifact与正文SHA、全部旧alias与3文档原字节、14历史身份/12匹配/9wrapper、时序/机械diff/v2删未用import/42相同曲段/原B0/3代理与图。仅允许10fixture实际重跑，10pass.25s，/tmp/no bytecode/cache；RED、237全suite、原目标搜索、chooser与作者verify只检查已有证据。没有science/search/act/sim/fit/inference/新矩阵重跑。

作者对每项declined scope裁决（如误判的成本附后）：

1. Fresh-context independence: Fresh spawn failed agent thread limit; retained-context second-person reviewer explicitly disclosed. Cost if wrong: anchoring; no scientific acceptance promoted.
2. Complete historical runtime equivalence: Two changed core hashes preclude complete historical certification. Cost if wrong: unsafe historical portability assumption; preserve limited matched wrapper claims.
3. Historical V1 measurement-producer identity: Current payload/consumer identity is insufficient; report refuses old producer certification. Cost if wrong: misnamed nominal geometry.
4. Physical equivalence of V1 and roll-PID/BankMap: Cost consumption established, physical equivalence untested; targeted source/coverage investigation remains pending. Cost if wrong: wrong motion source; no runtime deployment.
5. Candidate physical tracking/actions: No complete candidate reference or rollout, no claim of physical change/improvement. Cost if wrong: imaginary labels; candidate rejected.
6. Actual35s feasibility: Three proxies are local/static; no dynamics/globalgoal deadline admission. Cost if wrong: misleading navigation claim, explicitly disallowed.
7. Missing time cost as cause: No actual causal navigation intervention; remains unproved. Cost if wrong: researching wrong layer.
8. Path beyond node budget: Exhaustion is bounded algorithm failure, not impossibility; no new budget rescue. Cost if wrong: overlook potential solution but retain clear search scope.
9. Global optimality/heuristic admissibility: Inherited rounded-grid search preserved, not certified; literal relative edge-unit statement not global guarantee. Cost if wrong: suboptimal solution, not promoted to production.
10. Budget/heuristic/parameter rescue: Sealed single-target search forbids further rescue currentstage. Cost if wrong: unbounded posthoc selection.
11. Future drive profiles including vmax<.05: Only frozenV1 tested; all42speeds .190848..580977>.05, no generalmodel admissibility claim. Cost if wrong: future API use requires new validation.
12. Continuous collision/blind/stopping safety: Raster screen only observed nominal samples; unknown-risk and earlier stopping refusal preserved. Cost if wrong: false safety certificate.
13. Beyond snapshot175: Single developmental snapshot; no whole-sequence/fresh generalization. Cost if wrong: case-specific benefit assumed broad.
14. Navigation rates/baseline admission: No new physicalnavigation or successful baseline gate; rejectedCPU cannot enter E0E1E2. Cost if wrong: fabricate successrate.
15. RGB-only/estimatedpose: Privileged current data remain; earlierRGB consumer rejections untouched. Cost if wrong: hiddentruth leak in visualbaseline.
16. Learned memory/worldmodel: No fit or learning increment; mission outstanding. Cost if wrong: misrepresent conventional proxy as worldmodel.
17. Next response measurement: Future bounded source/coverage task, not remedy or committed new sequence; establish gap first. Cost if wrong: duplicate old calibration.
18. Historical execution-count attestation: Logs/source/SHA inspected; reviewer did not monitor original targetprocess. Scope stated, no independent rerunclaim. Cost if wrong: infer unseen events from authorrecord.
19. Live scheduler/subsequent closure: Author read-only ACTIVE check and finalidentity check separately attributed; reviewer did not inspect futureclose. Cost if wrong: stale/admin identity mismatch.
20. Wholemission completion: Perception/estimatedpose/learning/formalnavigation remain unfinished ACTIVE. Cost if wrong: stop automation prematurely.

行政记录补充：一次只读scheduler探针在Python3.10缺tomllib，尚未读取配置即失败，工具输出另存明确为转录；随后只解析唯一quoted status行，15:10:56.946153UTC确认ACTIVE/SHA6e7f66086a80997e9cca1be69d15689d85312c912b6f6e85f381e248c82bf890，未改automation。一次附带rg把root的run_hybrid_golden误写到hybrid子目录，返回missingfile；科学SOURCE已在前置完整读取，后续无运行或输入改变。作者当前known-stage进程检查无运行中science。

最终阶段完成以time_cost_final_check.json实际PASS为准；review与作者finalidentity/document同步分开。旧report/科学结果/gate不变，全部代码/数据/失败/图/SHA保存，无新视频/物理/上传/Gitcommit/delete。整体研究NOTcomplete，保持ACTIVE，不通知任务完成。


作者最终行政收尾补充：首次identity builder误在已含尾换行的审查正文后再加一换行，精确prefix SHA断言失败（尚未生成identity）；随后checker报告missing identity，raw失败log保留。改为用原审查SHA精确匹配appendix附近的原字节前缀长度，无报告正文/源/协议/科学数据/gate改动、无science重跑。新finalcheck实际PASS才算阶段关闭。
