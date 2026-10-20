from common import *

r=json.loads((D/'result.json').read_text());co=json.loads((D/'prior_blocker_cohort.json').read_text())
fmt=lambda x:'N/A' if x is None else f'{x:.6f}'
qs=lambda a:' / '.join(map(fmt,a))
lines=['# 双目扫描线联合对应完整开发资格（2026-10-13）','','五条完整开发轨迹905对当前双RGB与4500步原轨迹地图更新完成。'+('源资格通过，仍需新鲜实际90秒对照。' if r['source_pass'] else '**源资格失败，拒绝接入导航，physical0/4未用4次关闭、fit0。**'),'','该机制保持v56的7×7 Census48与3×3汉明代价、完整9×9双眼CAD支持及108代价上限，只把独立逐像素最低代价选择替换为整条扫描线顺序、一对一匹配和显式未匹配缺口的联合优化。支持像素缺口成本54（继承108上限的一半），不支持像素缺口0；完整前缀与后缀动态规划求解，只接受所有全局最优路径必须包含的同一对应。整数视差，不在求解后移动亚像素位置。算法与参数运行前冻结，无调窗、扫参、训练或GT在线输入。','', '项目区别：原SGBM使用视差跳变惩罚及LR/局部唯一性；已关闭归属候选在原抽样源中拒绝重复右眼声明。此次生成新的全图联合对应。不是文献原创方法。参考：[OpenCV StereoSGBM](https://docs.opencv.org/4.10.0/d2/d85/classcv_1_1StereoSGBM.html)、[Cox等扫描线对应研究](https://scholars.duke.edu/individual/pub785998)。两者不保证本场景有效。','', '## 五完整序列测距','','|开发组|原深度P50/P90/P95 m|新深度P50/P90/P95 m|新误差>.3m|新测量数/原测量数|原真实障碍查询保留|灾难帧|','|---|---|---|---:|---:|---:|---:|']
for v in r['groups']:
 b,c=v['arms'];lines.append(f"|{v['group']}|{qs(b['depth_p50_p90_p95'])}|{qs(c['depth_p50_p90_p95'])}|{100*c['above030_fraction']:.2f}%|{100*v['dense_count_ratio']:.2f}%|{100*v['obstacle_query_retention']:.2f}%|{len(v['catastrophic_frames'])}/181|")
lines+=['','所有组均需有限评分>=1000、深度P90<=.15m、>.3m比例<=10%、总测量保留>=70%、每帧原测量>=1000时保留>=50%、新有限评分>=1000帧P90<=.3m、原真实障碍查询保留>=95%。最后一项是原入图查询像素的保留率，不是整幅图/所有可见障碍召回；即使保留也不认证深度正确。未知和无渲染命中不当自由。','', '## 像素评分与入图点','','|组/方法|有限评分/预测像素|评分/整图|有限入图点/全部入图点|地面误报/有限入图点|入图位置P50/P90/P95 m|','|---|---|---:|---|---|---|']
for v in r['groups']:
 for c in v['arms']:
  lines.append(f"|{v['group']}/{c['arm']}|{c['finite']}/{c['predicted']} ({100*c['finite_scoring_coverage']:.2f}%)|{100*c['full_image_scored_fraction']:.2f}%|{c['finite_accepted_points']}/{c['accepted_points']}|{c['ground_false_points']}/{c['finite_accepted_points']} ({100*c['ground_false_fraction']:.3f}%)|{qs(c['point_error_p50_p90_p95'])}|")
lines+=['','整图分母各181×640×480。深度指标只对有输出且渲染有限正深度的像素评分，无命中预测数量在result.json单列。入图点使用原CAD/双眼纹理/高度与不确定性筛选，地面误报按GT世界高度绝对值<=.03m。像素、点和格分母不可互换。','', '## 同轨迹长期地图（89.9秒）','','|组|原风险/偏离>.15m/旧10s/旧偏离|新风险/偏离>.15m/旧10s/旧偏离|','|---|---|---|']
for v in r['groups']:
 b,c=v['maps'][-1]['arms'];cells=lambda a:'/'.join(str(a[k]) for k in ['risk_cells','far015','older10s','old_far015']);lines.append(f"|{v['group']}|{cells(b)}|{cells(c)}|")
lines+=['','灰色参照为机体高度带实际障碍；绿色是风险格中心距参照<=.15m，红色>.15m，紫圈为最近支持超过10秒，白色预测为未知。固定原轨迹的离线反事实地图，不是新物理运行或新到达证据。源评分包括90s末帧，动作地图最后89.9s；H用原重复曝光记忆，其余组原视角确认记忆，两方法各组保持一致。','', '## 已固定阻塞像素集合','']
for a in co['rows']:
 lines.append(f"- {a['frame']/10:.1f}s原{a['queries']}点：新有测量{a['joint_measured']}、未知{a['joint_unknown']}；误差<=.15m {a['error_le015']}、>.3m {a['error_gt030']}。新点深度P50/P90/P95 {qs(a['new_error_quantiles'])}m。")
lines+=['','原阻塞格(-1.1,-.1)最终状态：'+json.dumps(co['final_original_blocker_cell'],ensure_ascii=False)+'。此集合和49.5s图因上次已知阻塞来源事后选定，并在本候选输出前固定；不是随机代表或本次搜索最差帧。','', '## 同刻直接图','']
for name,i in [('C54',495),('E0',450)]:
 v=next(v for v in r['groups'] if v['group']==name);f=next(f for f in v['frames'] if f['frame']==i);c=f['arms'][1]
 lines+= [f"### {name} {i/10:.1f}s",'',f"新深度P50/P90/P95 {qs(c['p50_p90_p95'])}m；有限{c['finite']}/{c['predicted']}，占整图{100*c['finite']/(640*480):.2f}%；有限入图{c['finite_points']}/{c['points']}，地面误报{c['ground_false']}；点位置P50/P90/P95 {qs(c['point_p50_p90_p95'])}m。",'',f"![RGB/预测/真值/误差]({D}/figures/{name}_depth_{i:03d}.png)",'',f"![同刻风险与真图]({D}/figures/{name}_maps_{i:03d}.png)",'',f"![最终长期记忆]({D}/figures/{name}_maps_899.png)",'']
lines+=['## 范围、执行与核验','','五组H/E0/E1/B54/C54全部是已见开发，未作为新独立留出。理想原生IMU、编码器估计、特权初始规范、固定.4m高度先验沿用并披露；当前双RGB/原生标定与静态CAD输入生成源，动态CAD和位姿只在原投影消费者。GT在所有输出封存后才评分，未知/缺失不能在线清旧风险。','', '启动前独立审查发现旧文件路径引用会造成空ledger的问题，派发前修复并保存旧脚本和审查。首次CPU用例dtype不符属于测试接口，旧测试保留，实际估计器未因此改变。durable记录在generation_status.json、score_status.json和visuals_status.json，预算ledger.json；中断/接口失败不记性能失败。','', '只读身份命令：使用既有 `/home/an/miniconda3/envs/ball_wm/bin/python` 执行本目录 `verify_readonly.py`。该核对只确认封存身份和判据聚合，科学复算另见独立审查；不对封存目录再生成。','']
if (D/'independent_closure.json').exists():
 a=json.loads((D/'independent_closure.json').read_text());lines+=['## 独立闭合','',''+json.dumps({k:a[k] for k in ['decision','hash_checks','numerical_fields_compared','max_float_difference','independently_scored_pairs','selected_map_metric_sets'] if k in a},ensure_ascii=False),'','具体独立回放范围及局限见independent_closure.json。完成的是这一候选的源资格阶段；可靠导航、学习导航增量和整体目标仍未完成。','']
(W/'reports/ordered_scanline_depth_results_20261013.md').write_text('\n'.join(lines)+'\n')

with (W/'reports/ordered_scanline_depth_results_20261013.md').open('a',encoding='utf8') as f:
 f.write('\n## 局部作用与下一研究方向\n\n旧46个阻塞来源像素中，44个变为未知，2个仍有测量：1个误差<=.15m，另1个在.15–.3m。新地图从原轨迹起点重放时未形成旧阻塞格，未用真值在线删除它，也未证明该区域自由。完整C54深度P90从原.133587m升至.533280m；虽低于已拒绝的独立局部Census 1.158500m，仍不满足.15m准入。原真实障碍查询仅保留30.86%，最终新增地图偏离参照60/284，旧错误60，不能因单个阻塞格不再出现保留整个源。\n\n这条联合机制闭合：不调缺口成本、歧义条件、视差域、窗口或描述子复跑。下一优先接续此前因规划研究暂缓的像素表面类别与距离分开重建资格：先有限核对旧高度/地面竞争/BEV模型与新类别消费者的实质区别；用理想像素类别及深度、实际原生位姿完成整序列投影诊断，检查地面误报、位置误差和可见障碍覆盖。GT仅离线诊断/监督，不作在线语义；资格通过才冻结独立像素学习目标、完整分组数据与一次拟合/fresh验证，分别标注双眼测量与单眼先验。不能换网络救旧v49模型或恢复旧预算。该下一目标尚未实现、尚未准入，也不承诺导航增量。\n\n## 编码执行故障及唯一恢复\n\n首次生成在H/E0/E1三组完成后，宿主Python默认ASCII无法解码B54的UTF8配置，于08:58:09.985579 UTC结束；原runtime_failed/child1与1653份已生成输出均保留。唯一恢复设置PYTHONUTF8=1，五个科学源文件及协议哈希不变，前三组输出复用且packet/地图重新比对不覆盖。恢复09:07:45–09:29:50 UTC finished0；physical0、fit0、recovery1/1。该编码接口退出不是性能失败；源拒绝来自随后完整评分。详细身份见recovery_registration.json/recovery_review.json与两个durable状态。\n')
