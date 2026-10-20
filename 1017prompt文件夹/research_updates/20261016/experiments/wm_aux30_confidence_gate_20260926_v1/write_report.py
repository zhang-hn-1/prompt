from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).resolve().parent;ROOT=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark');DEST=ROOT/'research/ball_wm/runs'/R.name
read=lambda p:json.loads(Path(p).read_text());p=read(R/'protocol.json');A=Path(p['frozen_acceleration_run']);a=read(R/'assessment.json');usage=read(R/'gate_usage.json');diag=read(R/'calibration_diagnostics.json');history=read(R/'calibration_history.json');status=read(R/'status.json')
d={s:read(R/f'{s}_prediction.json') for s in ['val','test']};old={s:read(A/f'{s}_prediction.json') for s in d}
plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False})
fig,axes=plt.subplots(1,3,figsize=(13,4),layout='constrained')
for col,s in enumerate(['val','test']):
 for name,label,color in [('constant_velocity','Constant velocity','#666666'),('old','Acceleration residual','#2178ad'),('model','With confidence scaling','#d5832b')]:
  vals=[(old[s] if name=='old' else d[s])['aggregate']['per_horizon'][h]['model' if name=='old' else name]['displacement_error_m_mean']*100 for h in ['0.1','0.3','0.5']]
  axes[col].plot([.1,.3,.5],vals,'-o',label=label,color=color)
 axes[col].set(title=f'Historical {s} | 94 windows',xlabel='Prediction horizon (s)',ylabel='Mean XY error (cm)',xticks=[.1,.3,.5]);axes[col].grid(alpha=.2)
axes[0].legend(fontsize=8)
for s,color in [('val','#2178ad'),('test','#d5832b')]:
 axes[2].plot([.1,.2,.3,.4,.5],[usage[s]['overall'][f'{h:.1f}']['mean_coefficient'] for h in [.1,.2,.3,.4,.5]],'-o',label=s,color=color)
axes[2].axhline(.5,color='#777777',ls=':',label='Fallback cutoff');axes[2].set(title='No exact fallback on either split',xlabel='Prediction horizon (s)',ylabel='Mean applied coefficient',ylim=(0,1));axes[2].legend();axes[2].grid(alpha=.2)
fig.savefig(R/'confidence_gate_results.png',dpi=160);plt.close(fig)
L=lambda f,label:f'[{label}]({DEST/f})'
text=['# 保守运动残差门控：校准与受控评估（2026-09-26）','',
'**唯一候选已完成，仍拒绝使用。** 28 项检查通过 24 项；短时位置误差略有改善，但半秒测试位置误差增加 5.22%，超过事前 5% 容差。更关键的是：验证与测试全部时域均没有触发一次精确匀速回退，尚未证明学会识别“不应修正”的状态。没有追加阈值扫描、重训基础网络或导航测试。','',
'## 1. 本次假设与唯一实现','',
'上一轮已经排除了明显的时间错位，并发现部分短时修正方向相反。本次检验：只根据当前与过去运动状态，估计该保留多少网络残差；不足时回到匀速先验，能否减少错误修正并保留中期收益。','',
'冻结已训练的加速度残差模型及其视觉编码器，新增 **175 参数**的小门控层：当前 17 维本体状态，以及当前减去 0.4 秒前状态的 17 维趋势，共 34 维；线性层输出五个时域的 sigmoid 系数。跨帧机体系速度差只是状态趋势，不能称为惯性加速度。基础模型仍读取历史双目特征和给定请求；门控自身只读取本体历史，不读取未来实际状态或未来请求。','',
'系数不是已校准的“成功概率”。部署规则训练前固定：系数 **<0.5 则置零，完全回到匀速；否则按该系数缩放残差**。XY 与 yaw 共用同一时域系数，yaw 按最短角差插值后恢复 sin/cos；零和一端点保持原预测逐位不变。','',
'训练监督来自八条原训练序列：`c=冻结模型−匀速`、`r=真值−匀速`，最优受限缩放目标为 `clip(c·r/(c·c),0,1)`，使用原 XY 米与 yaw 弧度的单位权重。以 `c·c` 为权重、各时域内归一化；零能量不参与拟合。此处真实未来只生成训练监督，不进入部署输入。','',
'固定 CPU float32、完整训练批次、Adam lr=0.01、500 次更新、L2=0.01×权重平方均值，零初始化；只用训练集统计归一化均值和总体标准差，标准差下限 1e-4。取第 500 步，没有验证选轮、阈值搜索或第二种结构。训练拟合连续系数，评估应用固定的 0.5 截断，因此存在训练/使用规则差异；裁剪缩放目标的加权拟合也不能称为严格等价于共享参数下的原位姿损失优化。','',
'该独立研究模块只验证 CPU float32 用法，未实现通用 CUDA/float64 接口；没有移植进正式导航。','',
'## 2. 数据隔离与校准局限','',
'沿用 **8/2/2 条完整训练/验证/测试序列**，窗口 1848/462/462。评估每五帧取点，验证和历史测试各 94 窗口。不同集合的完整序列互不交叉，集合内部的相邻窗口仍相关。现有验证/测试已参与前期设计，是历史回归，不能作为全新 held-out。','',
'基础网络已经见过这八条训练序列，所以本轮用到的是其训练内残差；这不是独立的置信度校准数据。即使门控只在训练集拟合，也不能据此声称已经学到可泛化的不确定度。这一限制在训练前协议中写明。','',
f'完整校准、评估和指纹核验实际完成，返回码 0，运行约 {status["ended_epoch"]-status["started_epoch"]:.2f} 秒（复用缓存特征，只训练 175 参数）。校准目标从 {history[0]["loss"]:.6f} 降至 {history[-1]["loss"]:.6f}；500 次更新已保存历史与最终权重，基础网络权重不变。','',
'## 3. 相同窗口的实际预测','',
'XY 是平均欧氏误差（cm）；yaw 是平均包角绝对误差（度），不是 RMSE。旧模型指冻结的加速度残差模型。','',
'| 集合 | 时域 s | 匀速 XY | 旧 XY | 门控后 XY | 旧 yaw | 门控后 yaw |','|---|---:|---:|---:|---:|---:|---:|']
for s in d:
 for h in ['0.1','0.3','0.5']:
  new=d[s]['aggregate']['per_horizon'][h];before=old[s]['aggregate']['per_horizon'][h]['model'];vals=[new['constant_velocity']['displacement_error_m_mean']*100,before['displacement_error_m_mean']*100,new['model']['displacement_error_m_mean']*100,before['yaw_abs_error_deg_mean'],new['model']['yaw_abs_error_deg_mean']]
  text.append('| '+s+' | '+h+' | '+' | '.join(f'{v:.4f}' for v in vals)+' |')
text+=['',f'![预测与实际门控使用]({DEST/"confidence_gate_results.png"})','',
'历史测试 0.1 秒 XY 从 **0.643 mm → 0.627 mm**，约改善 2.5%；半秒从 **0.800 cm → 0.841 cm**，反而增加约 5.22%。验证半秒有改善（1.116 → 1.066 cm），但不能用验证的收益掩盖测试的退步。','',
'原请求变化组：','',
'| 集合 | 时域 s | 匀速 XY cm | 旧 XY cm | 门控后 XY cm |','|---|---:|---:|---:|---:|']
for s in d:
 for h in ['0.1','0.5']:
  new=d[s]['categories']['request_change']['per_horizon'][h];before=old[s]['categories']['request_change']['per_horizon'][h]['model'];text.append(f'| {s} | {h} | {new["constant_velocity"]["displacement_error_m_mean"]*100:.4f} | {before["displacement_error_m_mean"]*100:.4f} | {new["model"]["displacement_error_m_mean"]*100:.4f} |')
text+=['',
'“请求变化”仍指未来半秒内变化，不等于预测起点已经切换。门控后这些组的位置误差有所下降，但验证半秒及两组短时仍差于匀速。旧定义和门槛未修改。','',
'## 4. 是否真的学会了回退','',
'| 集合 | 时域 s | 精确匀速回退数 | 平均保留系数 |','|---|---:|---:|---:|']
for s in ['train','val','test']:
 for h in ['0.1','0.3','0.5']:
  u=usage[s]['overall'][h];text.append(f'| {s} | {h} | {u["exact_cv_count"]}/{u["count"]} | {u["mean_coefficient"]:.4f} |')
text+=['',
'四条评估序列分别统计也全部为零回退，五个时域都如此；每条序列、每个时域的完整均值与计数保留在 gate_usage.json。因此本轮实际行为主要是**根据状态缩小残差**，不能写成“已经能在不可信时可靠退回匀速”。','',
'事后诊断进一步显示，在 0.1 秒，验证 94 窗口有 17 个、测试 94 窗口有 18 个，其事后最优联合 XY/yaw 缩放为零，门控却全部保留了修正。验证/测试的 XY 修正方向相反窗口有 29/27 个，也全部保留。事后最优系数只用于诊断，未回灌模型或修改阈值。','',
'训练评估窗口的能量加权最优缩放均值约 0.831，验证约 0.679，而门控验证平均输出约 0.837。验证上偏乐观；测试加权最优均值约 0.828、预测均值约 0.819，看起来接近，仍漏掉全部应回退窗口。这说明“平均缩放接近”不能替代逐状态的可靠选择。训练内校准与低阶表达都是可能限制，目前没有把两者的影响因果分开。','',
'## 5. 验收与处置','',
'保留旧 24 项要求，其中半秒相对旧 30 轮**速度残差**模型最多退步 5% 的比较对象没有更换。额外增加四项：两集合的半秒 XY/yaw 相对本次冻结加速度模型也不得增加超过 5%。旧检查 **21/24 通过**；总计 **24/28 通过**。门控使用率是独立行为诊断，没有在事后追加数值 gate。','',
'| 未通过检查 | 实际比值 | 上限 |','|---|---:|---:|']
for x in a['checks']:
 if not x['passed']:text.append(f'| {x["split"]} / {x["check"]} | {x["ratio"]:.6f} | {x["limit"]:.2f} |')
text+=['',
'**拒绝本候选，不接入导航。** 半秒测试 5.2208% 的退步略超门槛，不能四舍五入为通过；另外三项切换组失败也仍存在。旧加速度模型与速度残差模型的原拒绝结论继续保留。','',
'本轮没有新增采集、fresh 评估、控制 A/B 或 E0/E1/E2；没有修改 planner、Oracle、执行限速、动作空间或正式 35 秒规则。','',
'## 6. 对下一步的影响','',
'这次排除了“在现有训练残差上加一个小门控就能可靠兜底”的乐观假设，保留了完整失败证据。下一步不应继续调这个 0.5 阈值来追逐四条旧评估序列。若继续可信度路线，应先引入**没有参与基础模型拟合的独立校准序列**，与最终 fresh 测试严格分开，检验能否识别错误修正；不能把现有旧测试或预封存 fresh 验证转作训练后仍称其独立。小门控表达能力是否足够也需要后续受控比较，本次尚未证明。','',
'目标仍是可用于导航的运动预测，但目前只证明部分历史总体预测有收益，尚未证明候选动作排序或闭环成功。','',
'## 7. 测试、审查与复现','',
'- 新门控的七项测试先出现 **7 failed**（模块尚不存在），实现后 **7 passed、3 warnings**：手算缩放标签与能量、未来信息隔离、精确回退、yaw 包角插值、低系数回退、真实拟合区分状态、联合模型请求前缀因果性。',
'- 运行器验证移除/替换未来 target 后输出不变。独立审查进一步在全部 188 窗口以 NaN 替换 target，预测逐位一致；检查训练/验证/测试完整序列隔离、训练专用标准化，并独立复算 28 项结果一致。',
'- 全测试尝试在 90 秒终止（返回码 124），途中出现环境缓存错误；单独定位到 `test_bev_ground_truth.py::test_center_depth_marks_forward_ray_visible`，原因为受限环境无法写默认 Numba 缓存。将缓存位置改到可写的 /tmp 后重新运行常用回归集，排除历史上未能正常完成的 `test_dynamic_lattice_planning.py`。最终回归 **1210 passed、1 xfailed、3 warnings，17.28 秒**；不能称完整仓库套件全通过。',
'- 全部新脚本、训练前规则、权重、评分及首次失败日志保留；模型与原实验文件均核验指纹。研究代码先在当前可写工作区完成，验证后归档到项目工作树，运行协议保留实际原始输出位置。','',
'证据：'+L('protocol.json','训练前协议')+'；'+L('assessment.json','28 项验收')+'；'+L('gate_usage.json','逐序列使用比例')+'；'+L('calibration_diagnostics.json','事后校准诊断')+'；'+L('gate.pt','独立门控研究权重')+'；'+L('regression_tests.log','回归结果')+'；'+L('final_verification.json','最终核验')+'。','',
'复现入口 run_experiment.py 拒绝覆盖已有 status；应使用新目录、保持 confidence_gate.py/test_confidence_gate.py/协议所列源码指纹与输入一致。基础模型保持冻结，门控仅支持本次 CPU float32 路径。analyze_calibration.py 为不参与选模的事后分析。','']
(R/'world_model_aux30_confidence_gate_20260926.md').write_text('\n'.join(text));print('Report and plot generated.')
