"""Append evidence-based stage closure; leave whole research and automation active."""
from pathlib import Path
import json, hashlib, datetime
H=Path(__file__).resolve().parent; W=H.parent.parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
 v=json.loads((H/'active_observation_verification.json').read_text())
 r=json.loads((H/'active_observation_review.json').read_text())
 assert r['critical_count']==0 and r['important_count']==0
 assert v['actual_sequences']==1 and v['actual_motion_gate'] and not v['source_gate'] and not v['whole_research_complete']
 report=W/'reports/benchmark_original_scene_active_observation_20261005.md'
 assert sha(report)==v['source_and_artifact_sha256'][str(report)]
 for f,s in v['source_and_artifact_sha256'].items(): assert sha(f)==s,f
 files=[W/'research_mission.md', W/'docs/superpowers/plans/2026-10-04-stereo-memory-navigation.md',H/'progress.md',W/'docs/superpowers/plans/2026-10-05-original-scene-active-observation.md']
 heading='## 00:36UTC original-scene active observation — final stage closure'
 assert all(heading not in p.read_text() for p in files), 'already synchronized; do not duplicate'
 text=f'''

{heading}

完整报告 `reports/benchmark_original_scene_active_observation_20261005.md`。一次原 golden E1 的18秒完整采集，00:45:55.968560—00:47:12.551316UTC、退出0；180请求、181双目对、901物理采样、1081可用接触记录均无实际接触，两视频各181帧/10fps。相机、照明、材质、roll PID与执行链冻结。已知完整地图只作受控采集的二维扫掠前提，未成为视觉观测/自由区；这不是视觉决定动作或E1导航成功。

实际整段转弯和自身停车响应通过原运动门槛：XY peak .000072081m、六时域yaw最大.000694rad/bank .000632°；五个实际10Hz XY速度低于.05m/s的资格14.2秒，与原实测响应相同。同六秒前缀17非原始时钟字段301物理采样、61执行状态及历史逐值一致，raw原点相同。readonly报表初稿通配排除raw_漏检raw_command_target已保留；独立精确版本补齐17字段，所有点/标签/误差/运动/gate及其他字段完全不变，没有重采或重推理。

冻结ORB的362 current/history预测在00:48:04.371701UTC全部保存后才读离线标签；模型/gate采集前冻结，推理runner/367输入身份推理前封存。完整5531查询（5087有限renderer标签、444无标签）current 226保留/206正确、3.724%完整准确/p90 .044577m；history 568/512、9.257%/p90 .051520m。历史正确查询净增306，不是地图面积或导航增益。两臂原90%完整门槛均拒绝，不用91.150%/90.141%的条件比例宣布可用。终态249记忆记录、203有标签/200正确、46无标签/3错误、247旧查询；从未认证free。旧ORB、hard错误、V1/Fast/M2及所有历史结论保持。

128CPU测试通过，四个坏时间/整段停车支持/初态测试RED→GREEN。作者最终验证 {v['verified_utc']}：{v['old_sha_checks']}去重旧SHA、{len(v['source_and_artifact_sha256'])}新产物身份，181双眼特征重提取、362预测/有限wrapper等价/未来payload替换、181记忆快照、180历史移位与17字段完整前缀核验。一次独立只读whole-stage审查及其实际范围/限制见 `active_observation_review.json`；初次作者核验把当时仍写出的stdout log按空文件散列，独立审查确认为Minor；初次JSON保留并明确此局限，最终改用已预留排除的final日志输出，重新散列已完成的初次log。候选/数据/验证器未修改，最终完整身份逐项匹配；最终log在文档同步记录单独封存。不将作者检查冒充审查者执行。四张实际图作者已检查，六事件大图有缩放，另检查原尺寸两事件图。最终报告SHA `{sha(report)}`。

这是预封存后的完整新采集，但环境/初态来自已使用E1，不是新的独立环境泛化。known-pose诊断不是估计位姿；nearest-pixel renderer深度/亚像素射线、边界混合、body-ID不足与ORB完整滤波支撑限制保留。未新增全robot资产或DOF/gain读回审计，不将此单一初态响应迁移作通用安全证书。没有新导航率、学习/世界模型或原35秒E0/E1/E2结果。阶段实际进程均结束；全研究未完成、automation ACTIVE，常规不发完成通知。

下一项只做同181帧的只读“稀疏来源地图→完整前进与停车扫掠支持”诊断shadow。固定同一实测参考及完整181帧；current/history同口径，unknown/当前/历史/predicted与来源/位姿层次明确，不重采动作、不换网络/描述子/阈值、不用真值过滤或seed脚下free。这些181帧的标签已被本阶段评价使用，消费者设计继承此知识，只能称历史开发诊断，不能称fresh held-out。先冻结消费者语义/预算及仅观测输入，消费者输出保存后才重新读取标签作分层核验；明确全查询比例不能替代任务空间面积。若本段支持仍不足，给出具体冷启动/观测任务条件阻塞并关闭这一分支，避免无限局部审计。拒绝候选shadow不获得控制准入；只有来源与消费者各自验证通过才改变动作，再验证估计位姿、唯一学习增量和原35秒评测。
'''
 for p in files:
  with p.open('a') as f:f.write(text)
 result=dict(synchronized_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),stage_complete=True,whole_research_complete=False,automation_status='ACTIVE',actual_sequence_count=1,old_results_unchanged=True,document_sha256={str(p):sha(p) for p in files+[report]},verification_sha256=sha(H/'active_observation_verification.json'),review_sha256=sha(H/'active_observation_review.json'),final_verification_log_sha256=sha(H/'active_observation_verify_final.log'),initial_verification_self_log_hash_valid=False,final_artifact_hashes_valid=True,sync_script_sha256=sha(__file__))
 (H/'active_observation_document_sync.json').write_text(json.dumps(result,indent=2)+'\n')
 print(json.dumps(result,indent=2))
if __name__=='__main__':main()
