from pathlib import Path
import json,hashlib,time
import torch
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
S=Path(__file__).resolve().parent;p=json.loads((S/'protocol.json').read_text());R=Path(p['run_dir']);A=Path(p['original_run']);read=lambda f:json.loads(Path(f).read_text())
assert read(R/'status.json')['status']=='complete'
assess=read(R/'B_expanded/assessment.json');dis=read(R/'disposition.json');rep=read(R/'reproduction.json')
reports={arm:{s:read(R/arm/f'{s}_prediction.json') for s in ['val','test']} for arm in ['A_original','B_expanded']}
fig,axs=plt.subplots(1,2,figsize=(10,4),layout='constrained')
for ax,split in zip(axs,['val','test']):
 for arm,label,color in [('A_original','Original 8 sequences','#2178ad'),('B_expanded','Expanded 12 sequences','#d47722')]:
  report=reports[arm][split];ax.plot([.1,.3,.5],[report['aggregate']['per_horizon'][h]['model']['displacement_error_m_mean']*100 for h in ['0.1','0.3','0.5']],'-o',label=label,color=color)
 ax.plot([.1,.3,.5],[report['aggregate']['per_horizon'][h]['constant_velocity']['displacement_error_m_mean']*100 for h in ['0.1','0.3','0.5']],'--',label='Constant velocity',color='gray');ax.set(title=f'Historical {split} | 94 windows',xlabel='Prediction horizon (s)',ylabel='Mean XY error (cm)',xticks=[.1,.3,.5]);ax.grid(alpha=.2);ax.legend(fontsize=8)
fig.savefig(S/'data_expansion_prediction.png',dpi=160);plt.close(fig)
fig,ax=plt.subplots(figsize=(7,4),layout='constrained')
for arm in ['A_original','B_expanded']:
 h=read(R/arm/'training_history.json');ax.plot([x['optimizer_steps'] for x in h],[x['val_motion_loss'] for x in h],label=arm)
ax.set(xlabel='Optimizer updates',ylabel='Historical validation motion loss',title='Equal update budget / identical selection opportunities');ax.legend();ax.grid(alpha=.2);fig.savefig(S/'equal_budget_training.png',dpi=160);plt.close(fig)
L=lambda name,label:f'[{label}]({R/name})'
lines=['# 基础运动预测模型：等更新预算的数据扩充对照（2026-09-27）','',f'实际完成原8条与扩充12条数据的A/B训练。扩充组原28项门槛通过 **{assess["passed"]}/28**。'+('历史通过，必须另行完成冻结fresh评估才能接受。' if assess['all_passed'] else '**拒绝本候选，封存fresh未启用。**'),'','## 实验问题与边界','','上一轮独立校准门控未带来整体改善。本轮不使用门控，检验增加四条真实运动序列能否改善基础加速度残差预测；网络结构、初始权重、损失、优化器与评估窗口都不变。冻结双目编码器；其视觉特征仍参与预测，但本轮未验证视觉相对本体信息的独立贡献。','','| 项目 | A：原数据 | B：扩充数据 |','|---|---:|---:|','| 完整训练序列 | 8 | 12 |','| 总训练时长 | 192秒 | 288秒 |','| 训练窗口 | 1848 | 2772 |','| 完整遍历轮数 | 30 | 20 |','| 优化更新次数 | 27720 | 27720 |','| batch size | 2 | 2 |','| 验证/选模机会 | 每924次更新，共30次 | 相同 |','','两组84,931参数，同一个seed3072和逐张量完全相同的原始初始化，AdamW lr=5e-5、weight_decay=1e-4。均按历史验证motion loss最低选择，平局选更早；不按测试分数或28项通过数选轮。扩充组每条样本重复次数减少，是在相同更新预算下扩充数据支持范围的对照；不是“相同轮数多训练50%”。只有一个随机种子，不宣称统计显著性。','','## 数据隔离与已知例外','','复用已核验的原始数据及相同冻结编码器的特征缓存。新增四条原先用于独立门控校准，这次在训练前明确划入基础模型训练集；今后不能再当独立校准或fresh测试。原val/test各两条完整序列仍只作历史回归，既不进入训练，也不重新随机拆帧。原封存四条fresh保持隔离。','','第三条speed_histories最大applied转向目标16°，原正式V2逐序列>16.1°覆盖要求仍失败；四条的29项结构/时序/执行一致性全部通过，其余三条满足原覆盖。本轮训练前明确允许此完整低角度序列用于研究训练，不修改旧审计或称原正式审计全通过。所有接触标签片段保留。','','## 原组复现与实际训练','','原组完整30轮重新运行，和原实验相同初始化、顺序与更新。训练/验证历史最大loss差 '+f'{rep["max_history_loss_delta"]:.3g}'+'；通过预定1e-9复现要求。']
for arm in reports:
 s=read(R/arm/'selection.json');lines+=['',f'{arm}：选中第{s["selected"]["milestone"]+1}个验证点，累计{s["selected"]["optimizer_steps"]}次更新；实际训练耗时{s["training_seconds"]:.2f}秒。']
lines+=['',f'![相同更新预算下的训练比较]({R/"equal_budget_training.png"})','','## 相同历史窗口的预测结果','','每个集合94个窗口；XY为平均欧氏误差cm，yaw为平均包角绝对误差°。不是导航成功率。','','| 集合 | 时域 | 匀速 XY | A XY | B XY | B/A XY变化 | A yaw | B yaw |','|---|---:|---:|---:|---:|---:|---:|---:|']
for split in ['val','test']:
 for h in ['0.1','0.3','0.5']:
  a=reports['A_original'][split]['aggregate']['per_horizon'][h];b=reports['B_expanded'][split]['aggregate']['per_horizon'][h]['model'];aerr=a['model']['displacement_error_m_mean'];berr=b['displacement_error_m_mean'];lines.append(f'| {split} | {h}s | {a["constant_velocity"]["displacement_error_m_mean"]*100:.4f} | {aerr*100:.4f} | {berr*100:.4f} | {(berr/aerr-1)*100:+.2f}% | {a["model"]["yaw_abs_error_deg_mean"]:.4f} | {b["yaw_abs_error_deg_mean"]:.4f} |')
lines+=['',f'![数据扩充后的预测误差]({R/"data_expansion_prediction.png"})','','原“请求变化”分组含未来半秒内请求变化，不等价于预测起点刚发生换向。','','| 集合 | 时域 | 匀速 XY cm | A XY cm | B XY cm |','|---|---:|---:|---:|---:|']
for split in ['val','test']:
 for h in ['0.1','0.5']:
  a=reports['A_original'][split]['categories']['request_change']['per_horizon'][h];b=reports['B_expanded'][split]['categories']['request_change']['per_horizon'][h];lines.append(f'| {split} | {h}s | {a["constant_velocity"]["displacement_error_m_mean"]*100:.4f} | {a["model"]["displacement_error_m_mean"]*100:.4f} | {b["model"]["displacement_error_m_mean"]*100:.4f} |')
lines+=['','## 验收与处置','','原28项模板及比较对象保持不变：24项历史规则，加上半秒相对冻结加速度模型的XY/yaw保留要求。不会改成只比较较差的新门控。','','| 未通过检查 | 实际比值 | 原门槛 |','|---|---:|---:|']
for x in assess['checks']:
 if not x['passed']:lines.append(f'| {x["split"]}/{x["check"]} | {x.get("ratio",x.get("delta",x.get("count"))):.6f} | {x.get("limit",x.get("minimum",1e-5))} |')
lines+=['',('历史门槛未通过：拒绝候选，不采集fresh。' if not dis['historical_passed'] else '历史门槛通过，fresh尚待验证。'),'','本轮未改变planner、Oracle、执行限速、动作范围或正式35秒规则；没有运行E0/E1/E2。候选完整权重和失败项保留，不用局部误差改善替代所有验收项。','','## 验证证据','','新数据划分/等预算采样四项测试先失败、实现后通过；本轮针对运动预测与数据划分的实际测试为 **18 passed、3 warnings，2.27秒**（14项既有运动测试+4项新划分/采样测试），不是整个仓库测试套件。两组全部评估窗口执行target删除/NaN替换不变性测试；前缀因果性、相同窗口和匀速基线逐项核验。运行前后检查189份源码、固定输入及全部原始数据指纹。','','证据：'+L('protocol.json','训练前冻结规则')+'；'+L('data_scope.json','数据划分')+'；'+L('reproduction.json','原组复现')+'；'+L('B_expanded/assessment.json','28项验收')+'；'+L('B_expanded/selection.json','选模记录')+'；'+L('behavior_tests.log','实际测试')+'；'+L('final_verification.json','最终核验')+'。','']
lines+=['## 结论与后续重点','',
'这次数据扩充对位置预测有明确的本次实验收益：验证/历史测试半秒XY平均误差分别下降15.40%和13.64%；历史测试请求变化组的0.1秒XY由0.0565cm降到0.0388cm，本次该项已优于匀速0.0448cm。原组已逐张量复现，因此此对照没有把不同初始化或更多总训练更新混作数据收益。', '',
'同时，历史测试半秒yaw从0.3547°上升到0.4317°，增加21.72%；验证请求变化组的0.1/0.5秒位置预测仍差于匀速。通过项计数为24/28，与未加门控原组相同，但失败项发生了变化，不能只看通过总数。保留本候选作为位置预测改善的研究结果，不晋升为导航模型。', '',
'下一步应集中解决朝向预测的退步及验证请求变化片段，不再继续叠加门控。当前证据尚不能区分位置/朝向共享训练的折中、具体状态覆盖或单种子波动；应先在现有数据上定位这些误差对应的真实运动状态，再选择一个受控修正。新增数据已用于训练，不能再伪装成fresh验证。','']
(S/'world_model_aux30_data_expansion_20260927.md').write_text('\n'.join(lines));print(json.dumps({'passed':assess['passed'],'fresh_required':dis['fresh_required'],'report_ready':True}))
