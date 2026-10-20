## 12:46UTC tracking capacity and guard-source audit — final stage closure

先完整读 reports/benchmark_same_direction_capacity_and_guard_source_20261005.md 与 docs/superpowers/specs/2026-10-05-tracking-capacity-audit-design.md、plans/2026-10-05-tracking-capacity-audit.md。本轮只完成一个预封存CPU测量/来源审计，不新增物理导航、planner search、policy act、模型拟合或控制候选。144sources/14148旧与实际inputs于12:56:02.888173UTC冻结，一次完整审计12:56:18.580804→12:56:19.612774退出0；RED4fail2pass.16s→GREEN6pass.12s。首次cwd写入失败无文件/无science，保留日志。原35秒结果、rig、mapper/PID/speed/guard及旧RGB/M2/停止拒绝不改。

237..334的98个有效同方向tracking：名义main+.5/aux-30°的V1机体曲率+.880052，而当前速度插值BankMap正端点.796201..796603；98/98名义超过加权端点、98/98正式map夹边、98/98协议夹边，80/98速度超出表域被夹边。加权端点不是各层逆映射顺序，也不认证动态物理上限。490真实PIDtick377clip/5supervisoractive；早期倾侧滞后明显，334实际bank-.393705已接近目标-.392699，仍请求k1.660542/离参考.425525m。不能单凭PID饱和认定增益不足，不能说限制无作用或删supervisor。

10个不重叠测量窗全部可估计。真实XY三点几何曲率与root Euler yaw/arc、world-z积分/arc分别报告；早期XY曲率负并不自动证明打滑/符号bug，后期bank达到不证明恒曲率稳态。窗口后续实测只作离线描述，不输入在线预测。数据、唯一行/分母、图、PID与当前reference字段源均保存；禁止使用legacy全零heading数组。

33.5s原guard的4个past/current位置330/335×map330/335组合，位置330均保留，335均因原rasterclearance拒绝；只说明这个局部screen，不证明map全程无影响。原连续suffix/bridge距离.640678893>=.6，原rounding EDT.583095189<.6，规则如实触发重规划而非碰撞终止。两失败sample是同一投影点，round位移.058137568m；最近hazard为历史code4、timestamp13.099999707/frame131，来自3个原实测depth点，高度.528327..643834。hazard不受floor10秒TTL删除，不是335当前视觉/脚下先验。本层是特权depth+knownpose，不能改写为纯双RGB性能。

作者12:59:10.556788UTC核144/14148及全部保存计数/来源时序，无science重跑；两PNG视觉核验。本轮无新视频，旧endpoint两条实际视频作为上下文。ONE readonly第二人审查见tracking_capacity_review.json。新fresh-context dispatch因agent thread limit被拒绝，实际复用上一阶段审查者并公开保留上下文；不称fresh或二次独立重跑。正文与审查/作者追加分开，作者append/hashcheck未二次review；最终审查发现与逐项scope裁决以附录为准。保存所有代码/失败/数据/图/SHA，无无关Gitcommit/delete/upload。阶段完成以tracking_capacity_final_check为准；整个研究仍未完成、automation ACTIVE。

下一项尚未实施：ONE有界CPU规划名义响应/控制consumer一致性实验。先审查原V1/BankMap曲率定义、命令域与steady nominal适用性；在一个完整已知当前快照（可用175首次重建前，不取未来335结果设计输入）上只改变一个名义曲率响应来源或必要screen，原搜索/mapper/PID/gains/speed/rig/guard不动。比较同起态原计划与唯一候选的几何/goal/clearance/长度/换向数/当前请求域支持，冻结接受门槛后最多一次搜索对照；不把当前速度预测全部未来、不用旧实际轨迹当候选label、不强制禁reverse或循环等待。CPU资格通过才另阶段最多一条原35秒同初态实际对照。当前不再science/search/sim，RGB+estimatedpose、唯一学习记忆/世界模型与正式E0/E1/E2仍未准入，不宣称导航成功率。
