"""Append-only post-review stage closure; no extra scientific execution."""
from pathlib import Path
import json,datetime,hashlib
H=Path(__file__).resolve().parent;W=H.parent.parent
REPORT=W/'reports/benchmark_direction_root_cost_audit_20261005.md';PLAN=W/'docs/superpowers/plans/2026-10-05-direction-root-cost.md';SPEC=W/'docs/superpowers/specs/2026-10-05-direction-root-cost-design.md'
sha=lambda f:hashlib.sha256(Path(f).read_bytes()).hexdigest()
def read(n):return json.loads((H/n).read_text())
def save(n,d):(H/n).write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n')
def main():
 p=read('direction_root_cost_protocol.json');v=read('direction_root_cost_verification.json');a=read('direction_root_cost_assessment.json');st=read('direction_root_cost_status.json');review=read('direction_root_cost_review.json');pkg=read('direction_root_cost_review_package.json')
 assert review['review_count']==1 and review['critical_count']==review['important_count']==0
 prefix=REPORT.read_bytes().split('\n## 独立审查与作者收尾'.encode())[0];assert hashlib.sha256(prefix).hexdigest()==review['reviewed_report_sha256']==pkg['reviewed_report_sha256']
 assert v['interface_gate_pass'] and v['nominal_behavior_eligibility_pass'] and not v['new_physical_candidate_tested'] and not v['braking_switch_elimination_demonstrated']
 assert st['status']=='finished' and st['exit_code']==0 and st['actual_grid_searches']==1 and st['actual_hybrid_searches']==2 and st['new_simulations']==0
 assert sha(H/'direction_root_cost_assessment.json')==v['assessment_sha256']
 for d in [p['source_sha256'],p['input_sha256']]:
  for f,s in d.items():assert sha(f)==s,f
 old=read('guarded_reference_document_sync.json')
 for f,s in old['document_sha256'].items():assert sha(H/'direction_root_cost_pre_stage_ledger.md' if Path(f)==H/'progress.md' else f)==s,f
 heading='## 06:44UTC current-direction root cost — final stage closure';docs=[W/'research_mission.md',W/'docs/superpowers/plans/2026-10-04-stereo-memory-navigation.md',H/'progress.md']
 assert all(heading not in f.read_text() for f in docs)
 for f in docs:(H/('direction_root_cost_pre_close_'+f.name)).write_bytes(f.read_bytes())
 excluded={'direction_root_cost_final_identity.json','direction_root_cost_document_sync.json','direction_root_cost_final_check.json'}
 files=set(f for f in H.glob('*direction_root*') if f.is_file() and f.name not in excluded);files.update([REPORT,PLAN,SPEC])
 identity=dict(closed_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),stage_complete=True,whole_research_complete=False,automation='ACTIVE',interface_gate_pass=True,nominal_behavior_eligibility_pass=True,braking_switch_elimination_demonstrated=False,physical_control_candidate_tested=False,navigation_gate_unchanged_rejected=True,actual_grid_searches=1,actual_hybrid_searches=2,new_simulations=0,source_checks=len(p['source_sha256']),input_checks=len(p['input_sha256']),reviewed_report_body_sha256=review['reviewed_report_sha256'],final_report_sha256=sha(REPORT),author_verification_sha256=sha(H/'direction_root_cost_verification.json'),review_sha256=sha(H/'direction_root_cost_review.json'),final_artifact_sha256={str(f):sha(f) for f in sorted(files)})
 save('direction_root_cost_final_identity.json',identity)
 text=f'''\n\n{heading}\n\n先完整读 `reports/benchmark_direction_root_cost_audit_20261005.md` 与独立spec/plan `docs/superpowers/{{specs,plans}}/2026-10-05-direction-root-cost*.md`。一次归档frame175/17.5s名义几何对照完成06:55:35.435283—06:55:41.269461UTC退出0，preseal06:55:21.321515UTC94源/5630输入。仅1原grid+2Hybrid，同当前地图/pointgoal/XY/yaw、原node3000/.3primitive/.05sampling/.2goal、原reverse1.25/switch.5，不改实际动作/物理/rig/光照/mapper/PID/速度/guard/five-low/rebound。没有新仿真、推理、训练或视频，旧实际视频仅context，不能作候选真实标签。全研究NOTcomplete/automationACTIVE。\n\n原planner完整1821节点/route/target/pose/direction/command EXACT归档。当前acceptedforward/pendingNone，与当时appliedmain+.7、mainjoint实测+.876695/XY纵向+.339162一致；当前afterplanningdesiredreverse和零请求direction_change_braking单独report-only，预测snapshot禁止它们/未来输入。独立copiedplanner仅机械import、initialenum参数/验证、rootg+priority改变，两根仍可选。候选2434节点success首R→F，path3.3→3.0，统一current-directionobjective4.375→4.075，终点误差.174093→.018737。原内部换向1→2，但包括初始交接两者均2；拒绝消除换向/已降低制动耗时解释，不把名义曲线称真实轨迹或导航改善。\n\n作者{v['verified_utc']} 无search验证94源5630输入/字段defaults/当前来源/严格未来payload，21primitive147采样端点<1e-15/原inflatedraster/终点.2均过。Primitive栅格min .632456→.6，当前整polyline连续min .642272→.614018，原guard栅格min .583095FAIL→.6PASS，层次不同/未知风险和动态3D未认证。接口PASS与预declarednominaleligibilityPASS仅给未来独立单次控制对照的必要证据；旧原35s超时距goal4.156980/0contact保持，全部RGB/sourceconsumer/M2拒绝不改，estimatedpose/学习增量/正式E0E1E2未准入。\n\n16真实fixturecallableRED1.18→GREEN1.36s，178全Htests7.01s。作者两次cwd+relativepath写入全失败且无文件创建、缺文件logs保留，随后真正一次科学对照，不隐藏failure/不重搜；机械import用于独立副本/旧defaults原样。ONE只读独立review/每条declined范围裁决见direction_root_cost_review.json；审查bodySHA {review['reviewed_report_sha256']}，最终作者报告SHA {sha(REPORT)}；作者append/封存另核验，不冒称再审查。全部代码、两完整参考、当前快照、协议、数据/测试/图/日志/来源与旧身份保存，不做无关Gitcommit/delete。\n\n接续一项尚未执行的传统控制候选：仅在原guarded-reference policy每次正常rebuild向新planner传当前accepted-direction并应用原.5rootcost，retain不搜索/reset，不按episode/time触发、允许必要reverse。先CPU核验真实class/request类型转换/状态/五次与反弹及保持原guard来源；新协议事前封存最多一条同起点/同完整任务/原35s goldenE1对归档guarded B0，真实目标/独立进度/横向/空间检查点/接触/动作链验收，失败直接拒绝，无penalty/nodebudget/goal距离/速度扫描。该候选仍有2次换向，真实效果不能由当前几何gate代替。本heartbeat不新增控制run/更多search；可靠RGB、估计位姿和唯一学习记忆/世界模型仍待分层研究，全研究继续ACTIVE。\n'''
 for f in docs:
  with f.open('a') as out:out.write(text)
 sync=dict(synchronized_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),stage_complete=True,whole_research_complete=False,automation='ACTIVE',document_sha256={str(f):sha(f) for f in docs+[REPORT,PLAN]},final_identity_sha256=sha(H/'direction_root_cost_final_identity.json'),author_verification_sha256=sha(H/'direction_root_cost_verification.json'),review_sha256=sha(H/'direction_root_cost_review.json'),closure_script_sha256=sha(__file__))
 save('direction_root_cost_document_sync.json',sync);print(json.dumps({k:v for k,v in identity.items() if k!='final_artifact_sha256'},indent=2))
if __name__=='__main__':main()
