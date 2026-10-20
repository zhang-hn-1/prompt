"""Describe actual closed evidence; no gate, controller or label changes."""
from pathlib import Path
import json,datetime,hashlib
H=Path(__file__).resolve().parent;W=H.parent.parent;D=H/'camera_heading_observation_run_v1'
a=json.loads((H/'camera_heading_observation_assessment_v1.json').read_text());v=json.loads((H/'camera_heading_actual_viewpoint_measurement_v1.json').read_text());p=json.loads((H/'camera_heading_observation_prefix_comparison_v1.json').read_text());m={k:x for k,x in a['actual_task_metrics_separate_not_formal_navigation'].items() if not isinstance(x,(list,dict))};status=json.loads((H/'camera_heading_observation_status_v1.json').read_text());ver={k:json.loads((H/f'camera_heading_observation_{k}_verification_v1.json').read_text()) for k in ['interface','source','execution','collection']}
rows=a['all_window_source_updates'];table='\n'.join(f"|{r['frame']}|{r['time_s']:.1f}|{r['n']}|{r['correct']}|{r['insufficient']}|{r['finite_wrong']}|{r['precision'] if r['precision'] is not None else 'N/A'}|" for r in rows)
goal=v['first_goal_row'];total={k:sum(r[k] for r in rows) for k in ['n','correct','insufficient','finite_wrong']}
text=f'''# 当前相机朝向反馈观察：实际运行与拒绝/准入分层

本次判断 **{a['decision']}**。这是一个真实闭环开发诊断，整个研究未完成，没有触发正式E0/E1/E2，也没有新增导航成功率。

## 问题和唯一改动

上一版速度方向反馈在事件后0.2秒已经改变10.2048°，相机光轴水平朝向只改变0.7213°，导致转向提前结束。它符合旧course规格，但代理量没有完成观察目标。本次只将观察反馈改为当前左相机光轴在世界XY中的朝向，光轴=-本次USD相机world transform的Z列。该transform来自既有capture事务，未增加物理步/相机读取/policy调用。

这是**真位姿特权诊断**，不是纯RGB估计位姿。图像模型仍仅读取当前双目RGB。相机位置、照明、Fast权重、PID、事件两个不同源不足100、原始停稳五次/反弹规则、+.5主轴/−30°副轴请求、目标+10°、4秒转向硬截止、6秒观察硬截止、完整35秒预算全部冻结。水平投影退化明确NOT_EXERCISED，不能发明方向。窗口结束后不能锁存成功，不等待、不重置、不触发第二次观察。

预先冻结计划：`docs/superpowers/plans/2026-10-07-camera-heading-observation-correction.md`。只一次实际完整序列，无科学重试/幅度时长扫描/模型或门槛救分。

## 实际执行和终态

实际驱动开始{status['start_utc']}，结束{status['end_utc']}，壁钟{status['elapsed_wall_s']:.3f}s；终态原始指标如下。任务是相同2.6m局部开发参考、yaw−20初态，不是原完整正式场景，继承Oracle元数据不用于SPL。

```json
{json.dumps(m,indent=2,ensure_ascii=False)}
```

事件发生{a['event_time_s']}s；实际相机目标达到={a['camera_heading_goal_reached_within_observation']}，相机水平朝向最大变化{v['camera_heading_peak_deg']}°，3D光轴最大变化{v['camera_axis_3D_change_peak_deg']}°，真实根部最大位移{v['max_root_displacement_m']}m。首次目标记录：{json.dumps(goal,ensure_ascii=False)}。不能用后来恢复B0的运动冒充观察命令的标签。无区间接触={a['no_contact_during_probe']}；没有接触不等于导航完成，观察占用预算可能延后失败。

## 全部观察窗口源测量

每个非空包都要求左右两个实际native像素端点的世界误差<=.10m且精度>=.95，数据不足继续留在分母；至少两个不同更新100点、新区域20点/2格，无区间接触。离线renderer标签只在源/请求封存后评分，不进入决策。全部窗口合计{total}。

|帧|秒|源点|两端正确|不足|有限错误|全体正确率|
|---|---|---|---|---|---|---|
{table}

每非空包95%={a['every_nonempty_geometry_ge95']}；两个100点={a['two_distinct_100point_updates']}；新20点/2格={a['newregion_20points_2bins']}。单个好包不能代替全部窗口验收，观察执行通过也不意味着深度、地图、控制或导航通过。

## 接口、执行与实际证据

作者实际运行六项意义回归和完整522项测试（24.24s）。最初六个RED为未实现；首次GREEN尝试有1项测试将旧source205/20.5误写为新207/20.5，时钟保护拒绝，保留失败日志后修正测试身份。无放松生产保护。首次独立审查C0/I0/M1，审查者个人六测试与1295源SHA、实际diff、窗口外不锁存；完整suite/物理运行由作者个人完成。唯一Minor旧schema名称由单独effective scope覆盖，旧协议字节与旧scope均保留，源码行为不变。

collection首次因未配置USD库缺pxr失败，保留日志，仅补环境后同一只读核验通过，没有重跑物理或改变科学输出。

实际接口验证{ver['interface']['requests']}请求，所有当前轴/名义请求/最终请求/事件字段逐条重建；前缀截断输出相同，未来源/轴时钟在读取payload前拒绝。原始handoff、smoother/PID/supervisor在**实测反馈**下重建一致，不是无未来真值递归物理预测。源包/双目CAD/当前地图与只读shadow逐字段重建，源与输出SHA/曝光时钟核验，视频逐帧实际解码。验证脚本和JSON在独立camera_heading命名空间。

与上一版分叉前的实际前缀：

```json
{json.dumps(p,indent=2,ensure_ascii=False)}
```

如果strict_prefix_certified为false，只能作同任务历史比较，不能声称严格唯一因果A/B；当前实验内部单一改动的代码差异和封存身份仍明确。

## 分层结论与后续边界

反馈语义修正、观察是否实际转向、源几何、可执行导航是不同层。当前判断与上述所有冻结门槛分别保留；旧course、neutral、Hero、preLR及M2拒绝均不追改。不能通过延后碰撞、停住、路径索引跳转、未知填自由或单点稳态通过宣称视觉导航成功。

若本候选拒绝，不继续扫观察角度、时长、增益或替换网络。下一项工作必须依据本次实际可见空间/源输出/执行证据选择，并继续保留未知、当前观测、历史观测和预测的来源差异；估计位姿、学习记忆闭环以及原35秒E0/E1/E2尚未准入。本报告不是整个任务完成通知。

实际图像与路径：

![实际路径、源测量及bank]({H}/camera_heading_observation_causal_audit_v1/actual_visibility_and_bank_v1.png)

![实际native双目]({H}/camera_heading_observation_causal_audit_v1/actual_native_stereo_snapshots_v1.png)

[双目视频]({D}/stereo_view.mp4) · [第三人称视频]({D}/third_person.mp4)
'''
f=W/'reports/benchmark_camera_heading_observation_correction_20261007.md';assert not f.exists();f.write_text(text)
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();identity=H/'camera_heading_observation_results_closed_identity_v1.json';assert not identity.exists();identity.write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),whole_research_complete=False,formal_navigation_triggered=False,posthoc_only=True,artifact_sha256={str(x):sha(x) for x in [f,*[H/f'camera_heading_observation_{k}_verification_v1.json' for k in ver],H/'camera_heading_observation_assessment_v1.json',H/'camera_heading_actual_viewpoint_measurement_v1.json',H/'camera_heading_observation_prefix_comparison_v1.json']}),indent=2)+'\n');print(f)
