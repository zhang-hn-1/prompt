from pathlib import Path
import json,hashlib
H=Path(__file__).resolve().parent;W=H.parent.parent;a=json.loads((H/'terminal_goal_navigation_assessment.json').read_text());r=json.loads((H/'terminal_goal_execution_cause.json').read_text())
p=W/'reports/benchmark_terminal_goal_consumer_20261007.md'
p.write_text(f'''# 唯一终点减速消费者：实跑与拒绝结论

唯一新动作环节：冻结nominalcap1父控制器执行一次；当前pointgoal_base_xy距离<=.60米时锁存，将五个主轴最终请求裁剪到±.3，辅助轴、源/地图/参考/规划/roll-PID/协议/加速度/速度supervisor/相机/光照均不改。.60是球半径.40加原到达阈值.20的预先定义几何启发式，不是已证明的动力学停车距离。未扫描阈值/换网/拟合模型。

真实knownpose＋CAD双目＋10秒history两米局部任务：**12.3秒到达，终距.196979947米，接触0，runtime0**；123动作/124捕获/616物理/25推理。原task35秒/.2米/无接触PASS；运动起点起横向RMS {a['movement_cross_RMS_m']:.12f}/peak {a['movement_cross_peak_m']:.12f}米，原.12/.30门槛PASS。但严格候选终距不退化门槛(.19302877235323554+1e−6)失败，多{a['terminal_distance_regression_m']:.12f}米，**候选REJECT**。旧fast12.1/.195381984拒绝保留，旧slow22.6/.193028772保留。没有触发完整6.6米、正式E0/E1/E2或学习模型增量。

真实事件在frame110/11秒，当前距目标.575037533米（10Hz第一个满足<=.60的采样）。与实际旧fast551×17物理字段/111执行状态前缀精确，之后有效改变7个请求块；实际速度变化来自真实执行，未二次调用有状态policy。123次终点距离/锁存/父请求/最终请求按已存当前输入重建一致。源/父状态/参考/map与独立同流slow影子每次相等，影子只读不执行。两次RGB重置差异仍在，不能把全部实际性能差唯一归因于控制消费者。

执行因果核验：11.02秒开始raw/applied阶段，请求.3但smoothed/applied当时.94；直到12.30秒goal终止才到.3。实际主轴角速度.898747802→.454842508 rad/s、世界XY速度.361067981→.185759738 m/s；到达时实际速度未稳定在较慢目标。可用减速时长1.30秒，而冻结.5 rad/s²从上一smoothed目标到.3本身需要1.30秒，还存在物理响应延迟。目标裁剪确实生效，但“请求变慢”不能直接等同“实际状态已稳定”。这是一次实测因果诊断，不修改门槛、追加停止后的成绩或把推算冒充未来实测。

测试：3项新literal import RED→生产实现，首次GREEN临时fixture试图修改冻结PolicyObservation字段失败（日志保留）；更正fixture用新观测实例后3新+4旧共7PASS1.92秒；完整408PASS21.86秒。机械runner逆派生精确，旧nominal17owned均原样。封存1280源、924关闭产物共2204身份，源包/mask、history map、同流slow影子字段、未来payload trap均审计；两个完整124帧10Hz视频解码。GT深度/接触/实际进度仅离线评分；在线knownpose/地面与初始安全区先验明确，不是纯RGB或安全认证。

来源重建跨不同batch只接受32机器epsilon尺度差异，包对自身保存仍精确；grid-only验证器仅posthoc省deepcopy，不改变线上mapper。科学门槛与原封存gate完全保持。预完成评分身份文件 terminal_goal_scoring_pre_finish_identity.json 保存评分程序版本。

代码/原始证据：H=experiments/stereo_memory_navigation_20261004_v1，terminal_goal_consumer.py、run_terminal_goal_navigation.py、protocol/status、terminal_goal_history_run、assessment.json、execution_cause.json及execution_response.png。当前原始state是末次goalentry，不是停稳保持结果，不能声称稳定停在目标。后续不继续扫减速阈值，转向完整记忆失败时参考侧别与实际可执行路径的定点因果核验，避免局部终点精度成为研究主线。独立ONE终审稍后追加；整体任务未完成，自动任务保持ACTIVE。
''')
(H/'terminal_goal_ledger.md').write_text('ONE local actualFINISHED/taskPASS/candidateREJECT终距+3.951mm；fullgoalNOTtriggered。3RED→GREEN/408suite/2204identity，实际请求已生效但实际速度未稳。No threshold scan. Next bounded memory reference-side audit using closedhistory; no policyact/repeatGPU/newmodel.\n')
files=[p,H/'terminal_goal_consumer.py',H/'test_terminal_goal_consumer.py',H/'run_terminal_goal_navigation.py',H/'terminal_goal_navigation_protocol.json',H/'terminal_goal_navigation_status.json',H/'terminal_goal_navigation_assessment.json',H/'terminal_goal_execution_cause.json',H/'terminal_goal_execution_response.png',H/'diagnose_terminal_goal_execution.py',H/'terminal_goal_scoring_pre_finish_identity.json']
(H/'terminal_goal_author_final_identity.json').write_text(json.dumps(dict(owned_sha256={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in files},review_pending=True,whole_research_complete=False),indent=2)+'\n')
r=json.loads((H/'terminal_goal_continuation.json').read_text());r.update(status='finished_candidate_rejected',elapsed_s=12.3,final_distance_m=.1969799472001205,fullgoal_triggered=False,next_stage='closed memory route-side snapshot causal audit; no terminal threshold sweep');(H/'terminal_goal_completion_v2.json').write_text(json.dumps(r,indent=2)+'\n');print(p)
