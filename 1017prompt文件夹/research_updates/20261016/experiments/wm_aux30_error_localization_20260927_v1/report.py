from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
S=Path(__file__).resolve().parent;R=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs')/S.name;read=lambda f:json.loads(Path(f).read_text());d=read(S/'results.json');rows=read(S/'rows.json');ex=read(S/'training_exposure.json');test=[r for r in rows if r['split']=='test' and r['primary']]
fig,axs=plt.subplots(2,2,figsize=(12,8),layout='constrained')
for ax,ep in zip(axs[0],sorted({r['episode'] for r in test})):
 rs=sorted([r for r in test if r['episode']==ep],key=lambda r:r['anchor_s'])
 for model,color in [('A','#2178ad'),('B','#d47722')]:ax.plot([r['anchor_s'] for r in rs],[r['h']['0.5'][model]['yaw_deg'] for r in rs],'-o',ms=3,label=model,color=color)
 ax.set(title=Path(ep).parent.name,xlabel='Anchor time (s)',ylabel='0.5 s absolute yaw error (deg)');ax.legend();ax.grid(alpha=.2)
worst=max(test,key=lambda r:r['h']['0.5']['B']['yaw_deg']-r['h']['0.5']['A']['yaw_deg']);rs=sorted([r for r in test if r['episode']==worst['episode'] and abs(r['anchor_s']-worst['anchor_s'])<=2],key=lambda r:r['anchor_s']);ax=axs[1,0]
for label,model,key,color in [('Needed correction','B','yaw_needed_deg','black'),('A correction','A','yaw_correction_deg','#2178ad'),('B correction','B','yaw_correction_deg','#d47722')]:ax.plot([r['anchor_s'] for r in rs],[r['h']['0.5'][model][key] for r in rs],'-o',ms=4,label=label,color=color)
ax.axhline(0,color='gray',lw=.8);ax.set(title='Largest regression neighborhood (post-hoc)',xlabel='Anchor time (s)',ylabel='Yaw correction relative to CV (deg)');ax.grid(alpha=.2);ax.legend(fontsize=8)
ax=axs[1,1];cats=[('motion','reverse','Reverse'),('motion','forward','Forward'),('aux','large','Large aux'),('turning','turning','Turning'),('stop','qualified','Stopped')];x=np.arange(len(cats))
for model,color,shift in [('A','#2178ad',-.18),('B','#d47722',.18)]:ax.bar(x+shift,[d['summary']['test']['groups'][key][v]['horizons']['0.5'][model]['yaw_deg'] for key,v,_ in cats],width=.36,label=model,color=color)
ax.set_xticks(x,[label+'\nn='+str(d['summary']['test']['groups'][key][v]['count']) for key,v,label in cats]);ax.set(title='Measured-state groups (overlap)',ylabel='Mean 0.5 s yaw error (deg)');ax.legend();ax.grid(axis='y',alpha=.2)
fig.savefig(S/'yaw_error_localization.png',dpi=160);plt.close(fig)
(S/'figure_selection.json').write_text(json.dumps({'selection':'Largest primary-test paired half-second yaw regression; descriptive only, no model selection','episode':worst['episode'],'anchor_s':worst['anchor_s'],'radius_s':2},indent=2)+'\n')
L=lambda f,label:f'[{label}]({R/f})'
lines=['# 数据扩充后：朝向退步与请求变化片段定位（2026-09-27）','','**本轮完成只读诊断，没有重新训练、改控制器或启用fresh。** 先前24/28验收与拒绝结论不变。A为原8条数据的加速度残差，B为新增4条数据后的同结构模型。','','## 1. 已经查清的现象','','半秒朝向退步主要来自历史测试的reverse_brake_sweep，贡献约92.3%的总净退步。不是所有场景都变差，也不是主要发生在停稳后或首次换向。两个验证序列的朝向均改善。','','| 历史集合/序列 | 窗口数 | A半秒yaw误差° | B半秒yaw误差° | 净误差和变化° |','|---|---:|---:|---:|---:|']
for split in ['val','test']:
 for ep,x in d['per_episode'][split].items():
  h=x['horizons']['0.5'];lines.append(f'| {split}/{Path(ep).parent.name} | {x["count"]} | {h["A"]["yaw_deg"]:.4f} | {h["B"]["yaw_deg"]:.4f} | {h["B_minus_A_yaw_error_sum_deg"]:+.4f} |')
lines+=['','分组使用实际XY速度、当前root yaw过去0.1秒变化率、实测副轴角和瞬时yaw前向速度；未用请求角冒充实际角。fast阈值为0.15m/s，转弯阈值5°/s，大副轴角阈值15°。不同分组重叠，贡献不能跨维度相加。root-yaw差分与世界z角速度分别保存，不把二者当作同一个量。','','| test实际状态 | n | A半秒yaw° | B半秒yaw° |','|---|---:|---:|---:|']
for key,v,label in [('motion','reverse','倒车'),('motion','forward','前进'),('speed','fast','速度≥0.15m/s'),('aux','large','副轴角绝对值≥15°'),('turning','turning','yaw变化率≥5°/s'),('stop','qualified','连续5次低速已停稳'),('request_timing','no_future_change','未来半秒请求不变')]:
 x=d['summary']['test']['groups'][key][v];h=x['horizons']['0.5'];lines.append(f'| {label} | {x["count"]} | {h["A"]["yaw_deg"]:.4f} | {h["B"]["yaw_deg"]:.4f} |')
lines+=['',f'![朝向误差与实际状态]({R/"yaw_error_localization.png"})','','## 2. 错的是方向，还是幅度','','两种错误都有。9.0秒锚点处：实际速度约0.375m/s、副轴角16.45°，已距上次请求变化4.4秒，未来半秒请求不变。相对匀速先验，真实需要+0.725°修正，A给−0.500°，B给−1.724°，因此半秒误差由1.225°扩大到2.449°。8.5秒处真实需要−0.502°，B给−2.309°，则是同方向但过度修正。','','整体test半秒朝向“修正方向相反”的窗口数反而从24/94下降到16/94，但少数窗口错误幅度显著增大，导致均值退步。因此不能只看错误个数，也不能简单给全部修正统一缩放。这些修正方向仅用真值做事后分析，不进入模型输入。','','## 3. 新数据覆盖了什么，漏弱了什么','','单看“有倒车”“有大角度”不够：联合状态“实际倒车＋XY速度≥0.15m/s＋实测副轴绝对角≥15°”在新增数据里较少。下面按全部训练窗口计算，避免每0.5秒评分采样与整秒命令边界的相位造成误导；窗口相互重叠，不是独立试验。联合分组是对预先定义状态分组的事后交集，不是新增验收门槛。','','| 数据 | 总窗口 | 联合状态 | 比例 | 其中正副轴角 | 其中负副轴角 |','|---|---:|---:|---:|---:|---:|']
for split,label in [('old_fit','原训练'),('new_fit','新增训练'),('val','历史验证'),('test','历史测试')]:
 x=ex['all_window_coverage'][split];lines.append(f'| {label} | {x["total"]} | {x["reverse_fast_large"]} | {x["reverse_fast_large"]/x["total"]*100:.2f}% | {x["positive_aux"]} | {x["negative_aux"]} |')
lines+=['','合并后该联合状态比例由181/1848=9.79%降到204/2772=7.36%。这提示新增数据对这一状态的补充较弱，但**不能直接说选中模型看过的这类样本更少**。按真实随机顺序重算到实际选中权重：A见过2715次，B见过3141次；正副轴子组为1365次与1495次。B选在更晚的更新点，绝对曝光次数反而增加。只比较完整30/20轮的总曝光会错误解释实际被评估的模型。','','因此目前支持的是相对状态分布变化与局部预测失配的关联，不足以证明“曝光数量减少”是唯一原因。','','## 4. 请求变化片段仍失败的原因','','验证集原请求变化组11个窗口，其中10个的命令变化发生在第一步0.1秒预测之后。B在0.1秒仍有7/11个XY修正方向相反，8/11比匀速差；半秒分别5/11与7/11。因此不是给命令统一加延迟就能解释的错误。','','按真实请求开始执行时间重新对齐同一批11个事件，下表位置误差单位cm。每行对应相同事件的不同时间，行间相关，不是55个独立事件。','','| 锚点相对请求切换 | n | 0.1s匀速 | 0.1s B | 0.5s匀速 | 0.5s B |','|---|---:|---:|---:|---:|---:|']
for off,x in d['event_aligned']['val'].items():
 h=x['horizons'];lines.append(f'| {off}s | {x["count"]} | {h["0.1"]["CV"]["xy_cm"]:.4f} | {h["0.1"]["B"]["xy_cm"]:.4f} | {h["0.5"]["CV"]["xy_cm"]:.4f} | {h["0.5"]["B"]["xy_cm"]:.4f} |')
lines+=['','切换前0.5秒锚点的半秒预测结束时，新命令还未执行，B仍差于匀速；切换开始处半秒B则优于匀速。这进一步定位到某些已有运动状态下的残差修正，而非所有换向都不行。test的10个事件也已同样对齐，完整数据保存。','','严格按“当前前向速度超过±0.05m/s，未来半秒达到相反±0.05m/s”定义，原188个评分窗口没有实际完成这种换向；不能将请求变化组叫作真实换向成功/失败组。数据含倒车与完整长时换向过程，但当前短时评分不能据此宣称快速实际换向覆盖充分。停稳资格始终是过去连续五次10Hz的XY速度<0.05m/s，保留反弹清零。','','## 5. 是否位置损失压制了朝向','','只在固定A/B权重上分别求XY与yaw对共享context/GRU的梯度，没有优化器更新。数值属于当前损失定义（m²与rad²的系数均为1），不代表物理量天然可直接比较。','','| 权重/数据 | XY损失 | yaw损失 | yaw占总损失 | 共享梯度夹角余弦 |','|---|---:|---:|---:|---:|']
for model in ['A','B']:
 for split,x in d['gradient_diagnostic'][model].items():lines.append(f'| {model}/{split} | {x["xy_loss_m2"]:.6g} | {x["yaw_loss_rad2"]:.6g} | {x["yaw_fraction_of_loss"]*100:.1f}% | {x["shared_xy_yaw_cosine"]:.4f} |')
lines+=['','B的yaw项占原/新训练损失约59.5%/73.3%，共享梯度余弦为+0.033/+0.171。当前检查点没有呈现“yaw被很小的损失权重完全忽略”或强负向梯度冲突的直接证据；但单个检查点的梯度不能重建整个训练过程，也不能证明共享网络没有容量折中。现有证据不支持盲目加大yaw损失权重。','','## 6. 结论与下一项可执行实验','','已经把泛泛的“朝向变差”定位到高速倒车、大副轴角状态中的少数大幅错误修正；验证请求变化组仍包含命令尚未变化时的错误修正。本轮复核的请求块与yaw标签未发现对齐错误，没有证据把它统一归为执行延迟。','','下一项建议只改**训练采样比例**：保持12条数据、网络、初始化、损失和总更新预算不变，按训练数据的实际状态平衡倒车转弯与普通运动，并对正负副轴角对称处理。与本轮B按相同初值/预算比较，目标是减少这类大幅朝向错误，同时保住已获得的位置收益。分桶、比例和验收必须在训练前冻结；历史评估明确作为开发回归，fresh仍保留到候选全部通过后使用。不要按测试episode ID、具体秒数或未来真值触发模型行为。','','这只是证据支持的一个待验证改动，不保证通过；本轮没有训练它，也没有修改24/28拒绝结论。','','## 7. 实际验证与复现','','16条原始序列的1200步编号及全部3696缓存窗口的请求块与yaw标签对齐检查通过；原188评估窗口两模型的XY/yaw重算与既有记录最大差 '+f'{d["max_metric_reproduction_delta"]:.3g}'+'。复用旧定义对齐21个请求事件、五个相对时间点，共105个窗口。7项手算自检通过；实际运动/姿态基础测试 **16 passed、3 warnings，1.87秒**，不是完整仓库测试套件。本轮没有新训练、仿真或导航成功率。','','原始图像完整性沿用上轮已核验记录；本轮重新检查实际读取的states、缓存、checkpoint和189份源码指纹，不冒称重新审计全部图像。完整逐窗口结果含实测状态、修正分解、原分组及事件对齐。','','证据：'+L('protocol.json','诊断口径')+'；'+L('results.json','完整分组与梯度结果')+'；'+L('training_exposure.json','真实训练曝光')+'；'+L('rows.json','逐窗口数据')+'；'+L('event_rows.json','事件对齐')+'；'+L('behavior_tests.log','测试日志')+'；'+L('final_verification.json','最终核验')+'。','']
(S/'world_model_aux30_error_localization_20260927.md').write_text('\n'.join(lines));print('Report and figure written')
