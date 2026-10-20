from common import *
r=json.loads((D/'result.json').read_text());dis=json.loads((D/'display_metrics.json').read_text());fmt=lambda x:'N/A' if x is None else f'{x:.6f}'
lines=['# 固定双目查询域的度量残差资格（2026-10-13）','','**完整五组905对与4500地图时钟资格失败：关闭固定查询残差目标，fit0、physical0，不拟合。**','', '历史标量+3.455px残差、二元误差>15cm置信过滤、地面锚点/竞争测距与RGB类别距离先验均保持拒绝。本次不重新拟合它们，仅检验任意逐查询完美距离校正的原查询域是否值得新的独立学习。','', '冻结原sensor_packets左眼query IDs，不新增对应或补空洞。只在渲染有限.5–3m时给出完美距离；无命中/域外未知，不能当自由。原生理想IMU与标定保留，双眼可见语义目标取v62冻结作者兼容类别。两臂：完美距离+原.05–1.5m高度的零不确定性代理；额外完美类别的更宽松诊断。均GT_ONLY，未作为在线源。','', '|已见开发组|有限可校正/原查询|高度代理精确率/覆盖|理想类别精确率/覆盖|原生投影点误差P90 m|','|---|---|---|---|---:|']
for a in r['rows']:
 x=a['metrics']['range_height'];y=a['metrics']['range_category'];lines.append(f"|{a['group']}|{a['finite_correctable']}/{a['queries']} ({100*a['finite_fraction']:.2f}%)|{100*x['precision']:.2f}% / {100*x['recall']:.2f}%|{100*y['precision']:.2f}% / {100*y['recall']:.2f}%|{fmt(a['point_error_p50_p90_p95'][1])}|")
lines+=['','五组均因可见目标障碍格覆盖低于80%失败；精确率>=90%、地面误报<=5%、无支持<=10%、定位P90<=.15m的固定门槛未放宽。覆盖是冻结可见目标格、允许相邻1格匹配的指标，不是场景全障碍召回。完美距离的条件光轴误差为0，点位置仍受原生姿态/高度先验影响。','', '查询分母、有限标签分母与机体排除后的入图分母分别核验，当前数据额外机体排除数量0；未知作者标签另计。局部地图>=4点，永久记忆任一点入格，二者口径不同。高度代理未重跑原纹理/不确定性全部筛选，不是原完整流水线或所有学习方法的理论上界。它不能证明校正距离毫无导航价值；结论限本次完整可见目标资格不成立。','', '|组/诊断|地面误报格/目标地面格|无支持/预测风险格|最终风险/偏离>.15m/旧10s/旧偏离|','|---|---|---|---|']
for a in r['rows']:
 for k,m in a['metrics'].items():
  mm=a['memory_metrics'][k];cells='/'.join(str(mm[x]) for x in ['risk','far015','older10s','old_far015']);lines.append(f"|{a['group']}/{k}|{m['ground_false_cells']}/{m['ground_cells']}|{m['unsupported_predictions']}/{m['predicted_risk_cells']}|{cells}|")
lines+=['','同轨迹重新构建的离线图不是对真实历史风险在线删除，未认证自由/完整安全或新增导航。理想类别能减少长期偏离，但覆盖仍不成立；GT不得用于消费者救分。','', '## 同刻直接图','']
for a in dis:
 name=a['group'];i=a['frame'];q=' / '.join(fmt(x) for x in a['original_query_error_p50_p90_p95']);lines +=[f"### {name} {i/10:.1f}s",'',f"选择{a['selection']}；原查询{a['queries']}、有限校正{a['finite_queries']}（全图分母{a['full_pixels']}）。原查询深度误差P50/P90/P95 {q}m；oracle光轴距离误差0仅理想诊断。",'',f"![RGB/原深度/理想固定查询/真实深度/原误差]({D}/figures/{name}_depth_{i:03d}.png)",'',f"![同刻真实与风险图]({D}/figures/{name}_memory_{i:03d}.png)",'',f"![长期记忆]({D}/figures/{name}_memory_899.png)",'']
lines+=['## 执行边界','','durable qualify finished0，905源/1810局部图/4500时钟/40记忆快照独立重算一致；预审后仅补occupancy SHA，原审查版本保留。fit0、physical0、recovery0。原v62模型与全部旧预算不重开。可靠导航、学习导航增量和整体均未完成。','']
(W/'reports/metric_residual_domain_results_20261013.md').write_text('\n'.join(lines)+'\n')
