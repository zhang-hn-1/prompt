# 独立安全起步条件：完美可见来源与 prior 隔离

目的：稠密来源/记忆拒绝后，不再换模型。复用同一 E2 yaw−20 完整65帧实测序列，用明确 privileged renderer-depth/current knownpose 的零误差端点来源，比较严格 coldstart 与仅初始 footprint 已知安全的必要支持条件。这是条件诊断，不是 RGB 观测、在线实验、全3D证书或导航成功；该数据已开发，不能 fresh。

替代方案：继续网络/阈值扫描不允许；改安装/光照/场景不允许；原方案明确允许另列 safe-start prior，所以只隔离这个条件，不改变旧 coldstart/source gates。

沿用 PointSupportMap/.1格/.7扫掠/10sTTL/高度带，不增添插值、raycarving或扩大prior。只使用实际左眼 true axial depth 的双目可见端点：.5..3m、stride2、固定机体 mask、当前 root距离>=.55、当前右眼真实 depth 投影/FOV/mask及同表面一致性 .03+.01z。显式 sigma0仅完美测量假设；双目 GT 伙伴是理想来源的特权条件，不是视觉估计器。每个点真实frame/leftpixel/rightpixel/time可追溯。poses/label读取必须只当前/过去，不读取未来真实trajectory来形成consumer。

独立 ImmutableStartupPrior 只记录 t6 当前 root .7m footprint 的185 cells，且必须外部明确 source='task_assumption'，trusted=True，non-observation=True。该185格是静态原点条件，不跟随机器人或扩大；到 t6 之前禁用prior。任何 current/history hazard 仍优先阻断；prior永不进入 PointSupportMap records、floor/provenance/timestamps。proxy-supported/certifiedfree/3D/action分开，最终action始终false。仅 floor/support 条件也不能认证穿越上方障碍。

每帧保存唯一perfect-source history map与“没有prior/固定prior”必要支持记录；先完整consumer输出落盘，后独立评估器读取真实6..32s轨迹，报告 full498格、其中prior185与剩余313、当前 footprint未知/冲突、真实可见floor是否落在需要区域。全65地图，不补新物理序列；旧模型输出无读取、不重fit/infer。此perfect-source新counterfactual不是可以获准部署的候选。

通过/拒绝：纯类型/因果/prior隔离literal测试通过；严格t6 footprint unknown0/hazard0方满足必要支持。prior臂当前disk通过只能证明提供了起始假设，不是观测改善。延后运动/整条扫掠缺任一unknown/hazard则不能称仅起步prior解决导航。若perfectsource仍缺支持，明确source算法错误与观测缺口分别存在；不推广为所有RGB方法永久不可用。下一执行方向必须避免重复网络/姿态/Oracle矩阵。

自主授权优先于逐阶段审批，无删除/上传/无关Gitcommit。保存旧字节；唯一阶段最终readonly审查。整体任务NOTcomplete。
