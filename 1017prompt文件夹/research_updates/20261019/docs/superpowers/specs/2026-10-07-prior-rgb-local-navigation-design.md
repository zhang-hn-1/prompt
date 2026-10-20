# 固定双目与显式先验的首个实际局部闭环

用户已在2026-10-07批准原建议并要求现在执行、每小时接续。目标是首先完成一个2m局部pointgoal的真正RGB动作闭环，而不是再做观测审计；之后按结果推进完整目标。原strict benchmark/gates不改。本阶段最多2条35s实际序列，golden_e1_0001原场景/起态，唯一任务变更为沿原start→goal方向截取2m局部目标（只用任务目标，不用Oracle路径挑选）。不是完整E1完成或正式成功率。

选择：复用冻结Fast B0双目而不再fit，native几何/旧mask/1.5pxLR原样。候选只有空间hazard/floor来源记忆current-only→10s-history；两臂同时采用同一flat-ground任务假设和起始安全区、受控未知探索、局部planner/现有Endpoint tracker/roll-PID。禁止同时改planner/mapping/PID/速度/参考规则。不使用完整地图或Oracle路径在线；global goal可以读取。本层knownpose实测state明确privileged，RGB深度函数无GTdepth/pose/action输入。

同原相机/光照/.3auxvelocity/smoother/protocol，初始6s零请求。35s含初始观察，未知遍历主轴raw限制±.3rad/s（两臂同一策略，来源为旧limit_action）；原reference/方向五低速规则保持，0future真值。旧source gateREJECT不能转称可用深度证书，新任务为有显式风险的仿真实验。已观测hazard旧.6m空间screen，unknown可参与探索但labels仍unknown，floor/taskprior不证明完整3D自由体积。

每10Hz同步采集左右RGB；每5帧(0.5s)才执行同一Fast网络，最多71推理。最新来源端点使用产生时刻的实测camera pose，缓存在world坐标；重复消费原query/source时刻不刷新。旧point_packet仅在严格finitebool confidence guard后调用，未改封存源码；误差敏感性明确非实测bound。PointSupportMap原样TTL10s，包括hazards；current臂仅latest source packet，history臂叠加past<=now。未知/历史/当前/taskprior分离存盘，不用GT标签删坏点。flat-ground假设独立全grid bool层，起始safe-zone只初始.7mfootprint，不跟随robot，不写observedfloor。两臂局部路径来自现有local_curve+EndpointLookaheadPolicy；cap只在未知段由共享策略执行。

物理sim暂停进行网络/CPU计划，报告真实wall latency；35sim秒不冒称实时性能。GTdepth/occupancy仅capture后的offline_truth与评价，不能packet/policy读取。source只读currentRGB，knownpose独立。保存全部RGB/source predictions/worldpacket/plan/map/provenance/requests/执行状态/50Hzphysics/contacts/视频/identity。

事前验收：每条窄局部任务必须<=35s实际goal<=.2m、physicalcontact0、无runtime_error、完整同步/因果/身份核验；未知exploration不是安全认证。唯一history候选比同初态同目标current臂不得增加contact，终距与实际到达时间不退化；必须有来源记忆导致的真实请求/applied变化及独立实际进度证据，若两臂行为等价就拒绝“记忆改善”说法。各自baseline通过和memory增量通过分别报告；任何source/3D/estimatedpose/完整nav准入仍不能由此继承。

先literal RED/GREEN：cachedsource不能刷新/未来不读、boolconfidence拒绝NaN、prior不污染observed且hazard优先、同目标2m截断、history与current真正差异、unknown cap公平、拒绝globalOracle context。实现后source/input预seal，再两臂按固定次序actualrun current→history；不因第一臂失败改参数，runtime失败只允许行政/输入契约修复且保留失败，不启动第二科学候选。

阶段最后一次fresh readonly review，原字节保留、无无关Git提交/删除。自主授权优先于skill逐阶段审批。阶段结束继续有证据的下一工作，不因报告停止；whole任务NOTcomplete。
