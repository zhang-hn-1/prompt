from pathlib import Path
import json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
S=Path(__file__).resolve().parent;read=lambda f:json.loads(Path(f).read_text());p=read(S/'protocol.json');R=Path(p['run_dir']);A=Path(p['original_run']);B=Path(p['baseline_expanded_run']);D=R/'C_balanced';assess=read(R/'assessment.json');dis=read(R/'disposition.json');sel=read(D/'selection.json');scope=read(R/'sampling_scope.json');exp=read(R/'selected_exposure.json')
reports={label:{s:read(path/f'{s}_prediction.json') for s in ['val','test']} for label,path in [('A_original',A),('B_expanded',B),('C_balanced',D)]}
fig,axs=plt.subplots(2,2,figsize=(11,7),layout='constrained')
for col,split in enumerate(['val','test']):
 for row,(key,scale,label) in enumerate([('displacement_error_m_mean',100,'XY error (cm)'),('yaw_abs_error_deg_mean',1,'Yaw error (deg)')]):
  for model,color in [('A_original','#2178ad'),('B_expanded','#d47722'),('C_balanced','#16866b')]:axs[row,col].plot([.1,.3,.5],[reports[model][split]['aggregate']['per_horizon'][h]['model'][key]*scale for h in ['0.1','0.3','0.5']],'-o',label=model,color=color)
  axs[row,col].set(title=f'Historical {split} | 94 windows',xlabel='Horizon (s)',ylabel=label,xticks=[.1,.3,.5]);axs[row,col].grid(alpha=.2)
axs[0,0].legend(fontsize=8);fig.savefig(S/'state_sampling_results.png',dpi=160);plt.close(fig)
L=lambda f,label:f'[{label}]({R/f})'
lines=['# 实际状态均衡采样：单候选对照（2026-09-27）','',f'完成一个固定比例的训练候选C。原28项通过 **{assess["original28_passed"]}/28**；加入四项上一版位置收益保留要求后共 **{assess["passed"]}/32**。'+('历史验收通过，仍需fresh验证。' if dis['historical_passed'] else '**拒绝候选，fresh未启用。**'),'','## 唯一改动','','B为上轮12条数据均匀遍历的扩充模型；C复用同一12条数据、2772训练窗口、初始权重、84,931参数网络、损失、AdamW配置与27,720次更新。每924次更新验证一次，共30个选模点，按历史验证motion loss最低选择，不按测试分数或通过项数选权重。本轮只训练C，B沿用已完整运行并核验的冻结结果。','','唯一改变是训练样本频率和顺序：较快倒车、大副轴角的正/负角子组各占5%，其余90%。总10%接近原8条训练数据9.79%的联合状态占比，正负对称；比例在训练前固定，没有扫描。只有单一随机种子，不把这一次结果当作统计显著结论。','','分类只读取训练窗口锚点的实测状态：XY速度≥0.15m/s、yaw前向速度<−0.05m/s、副轴角≥15°或≤−15°；不读取未来真值、请求计划或测试误差。所有推理模型仍使用原输入，没有按测试序列名、时间或状态切换动作。','','| 训练子组 | 不同窗口数 | 整个预算出现次数 | 固定比例 |','|---|---:|---:|---:|',f'| 其余状态 | {scope["group_counts"]["0"]} | 49896 | 90% |',f'| 正副轴角倒车转弯 | {scope["group_counts"]["1"]} | 2772 | 5% |',f'| 负副轴角倒车转弯 | {scope["group_counts"]["2"]} | 2772 | 5% |','','固定配额打乱排列，子组内部反复随机无放回遍历；跨遍历边界可能在同一batch出现重复样本，属于事前披露的采样语义。不存在均匀“20轮”概念，不能将加权重复称为新增独立数据。','','新增四条仍属于训练数据；其中speed_histories原正式高角度覆盖失败记录继续保留，研究训练适用范围沿用上轮，其余完整性检查未放宽。原val/test仍是已用于开发的历史回归，不是fresh。','','## 实际执行与选模','',f'实际训练耗时{sel["training_seconds"]:.2f}秒；选中第{sel["selected"]["milestone"]+1}个验证点，累计{sel["selected"]["optimizer_steps"]}次更新。到该权重为止，正/负倒车转弯子组实际出现{exp["group_exposures"]["1"]}/{exp["group_exposures"]["2"]}次，其余{exp["group_exposures"]["0"]}次。完整训练预算与选中权重之前的实际曝光分开报告。','','## 相同历史窗口的结果','','A=原8条训练模型，B=12条均匀采样模型，C=本轮状态采样模型。位置为平均欧氏误差，朝向为平均包角绝对误差。','','| 集合 | 时域 | A XY cm | B XY cm | C XY cm | A yaw° | B yaw° | C yaw° |','|---|---:|---:|---:|---:|---:|---:|---:|']
for split in ['val','test']:
 for h in ['0.1','0.3','0.5']:
  v=[reports[label][split]['aggregate']['per_horizon'][h]['model'] for label in reports];lines.append('| '+split+' | '+h+'s | '+' | '.join([f'{x["displacement_error_m_mean"]*100:.4f}' for x in v]+[f'{x["yaw_abs_error_deg_mean"]:.4f}' for x in v])+' |')
lines+=['',f'![相同历史窗口的预测]({R/"state_sampling_results.png"})','','原请求变化分组含未来半秒的变化，不等于预测起点正在实际换向。','','| 集合 | 时域 | 匀速 XY cm | B XY cm | C XY cm |','|---|---:|---:|---:|---:|']
for split in ['val','test']:
 for h in ['0.1','0.5']:
  b=reports['B_expanded'][split]['categories']['request_change']['per_horizon'][h];c=reports['C_balanced'][split]['categories']['request_change']['per_horizon'][h];lines.append(f'| {split} | {h}s | {b["constant_velocity"]["displacement_error_m_mean"]*100:.4f} | {b["model"]["displacement_error_m_mean"]*100:.4f} | {c["model"]["displacement_error_m_mean"]*100:.4f} |')
lines+=['','## 未通过项目与处置','','原28项的数值门槛和比较对象全部保留；额外四项为val/test的0.1/0.5秒XY误差不得超过冻结B的1.05倍，防止以牺牲位置收益换朝向收益。','','| 检查 | 实际比值 | 上限 |','|---|---:|---:|']
for x in assess['checks']:
 if not x['passed']:lines.append(f'| {x["split"]}/{x["check"]} | {x.get("ratio",x.get("delta",x.get("count"))):.6f} | {x.get("limit",x.get("minimum",1e-5))} |')
lines+=['','判定：'+('历史通过，fresh尚待独立验证。' if dis['historical_passed'] else '拒绝，不接入导航，封存fresh保持未生成。')+' 没有调整阈值、重选测试上更好的轮次或再训练第二个比例。','','## 测试与复现','','六项采样器测试先失败、实现后通过；本轮相关运动模型与采样器测试 **20 passed、3 warnings，1.96秒**，不是完整仓库套件。训练函数由上轮脚本逐字提取，仅替换minibatch索引迭代器；源码身份和差异记录见implementation_provenance.json。训练前后核验原缓存、数据状态、比较模型和189份源码指纹，未改planner、Oracle或限速。','','完整采样索引提前保存并固定SHA；没有训练/验证/测试完整序列混用。预测移除target或将其换为NaN后不变，原窗口/匀速基线与请求前缀因果性保持原检验。','',L('protocol.json','训练前协议')+'；'+L('schedule.npy','完整采样计划')+'；'+L('selected_exposure.json','选中权重实际曝光')+'；'+L('assessment.json','全部32项验收')+'；'+L('C_balanced/selection.json','选模记录')+'；'+L('implementation_provenance.json','唯一代码差异')+'；'+L('final_verification.json','最终核验')+'。','']
lines+=['## 局部收益与整体代价','',
'按上轮已冻结的实测状态标签复核，test中“较快倒车＋大副轴角”的11个窗口，半秒yaw由B的1.1307°降到C的0.8657°，改善约23.4%；但val中同类6个窗口由0.4897°升到0.6420°。目标状态在test有局部收益，却未跨序列稳定复现。', '',
'全体test半秒XY由0.6906cm升到0.7452cm（+7.91%），yaw由0.4317°升到0.4806°（+11.33%）；全体val也退步。不能用目标小组的改善抵消整体代价，或事后按episode选择不同模型。', '',
'选模规则自动选中C的第14个验证点，而B选在第23个验证点。C选中时正/负目标子组曝光1317/1309次，B为1495/1646次；虽然C的采样比例更高，选中权重的累计次数更少。保留真实选模结果，不用后续轮次的完整预算曝光来解释已选模型，也不改为测试上更好的权重。', '',
'本轮只能拒绝这个固定比例、单种子的采样候选，不能证明所有状态采样都无效，也不能证明唯一根因。停止继续扫描比例，保留B作为位置预测较好的研究对照，其原拒绝结论仍在。世界模型导航仍缺可靠的跨状态预测、候选动作排序与闭环验证。','']
(S/'world_model_aux30_state_sampling_20260927.md').write_text('\n'.join(lines));print(json.dumps({'historical_passed':assess['all_passed'],'original28':assess['original28_passed'],'total_passed':assess['passed']}))
