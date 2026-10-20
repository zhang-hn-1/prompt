"""Author append-only evidence closure; no policy/search/sim."""
from pathlib import Path
import hashlib,json,datetime
H=Path(__file__).resolve().parent;W=H.parent.parent
REPORT=W/'reports/benchmark_outgoing_lookahead_and_heading_chain_20261005.md'
HEADING='## 10:45UTC outgoing-edge lookahead CPU audit — final stage closure'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
read=lambda n:json.loads((H/n).read_text())
def save(n,d):(H/n).write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n')
def main():
 p=read('outgoing_lookahead_protocol_v2.json');a=read('outgoing_lookahead_assessment.json');v=read('outgoing_lookahead_verification.json');r=read('outgoing_lookahead_review.json');pkg=read('outgoing_lookahead_review_package.json')
 assert r['review_count']==1 and r['critical_count']==r['important_count']==0
 assert a['interface_regression_pass'] and a['valid_tracking_rows']==188 and a['actual_feedback_rows_exact']==1750
 assert a['archived_policy_act_calls']==a['new_physical_runs']==a['new_planner_searches']==0
 assert not a['actual_navigation_candidate_tested'] and not a['candidate_physical_prediction']
 assert v['completed_archived_passes']==1 and v['maximum_attempts']==2
 assert sha(H/'outgoing_lookahead_assessment.json')==v['assessment_sha256'] and sha(H/'outgoing_lookahead_protocol_v2.json')==v['protocol_v2_sha256']
 body=REPORT.read_bytes().split('\n## 独立审查与作者收尾'.encode())[0]
 assert hashlib.sha256(body).hexdigest()==r['reviewed_report_sha256']==pkg['reviewed_report_sha256']
 for mapping in [p['source_sha256'],p['input_sha256']]:
  for f,d in mapping.items():assert sha(f)==d,f
 old=read('cusp_navigation_document_sync.json')
 for f,d in old['document_sha256'].items():
  q=H/'outgoing_lookahead_pre_stage_ledger.md' if Path(f)==H/'progress.md' else Path(f);assert sha(q)==d,f
 docs=[W/'research_mission.md',W/'docs/superpowers/plans/2026-10-04-stereo-memory-navigation.md',H/'progress.md']
 for f in docs:
  assert HEADING not in f.read_text();(H/('outgoing_lookahead_pre_close_'+f.name)).write_bytes(f.read_bytes())
 excl={'outgoing_lookahead_final_identity.json','outgoing_lookahead_document_sync.json','outgoing_lookahead_final_check.json'}
 files={f for f in H.glob('*outgoing_lookahead*') if f.is_file() and f.name not in excl};files.update([REPORT,W/'docs/superpowers/specs/2026-10-05-outgoing-lookahead-design.md',W/'docs/superpowers/plans/2026-10-05-outgoing-lookahead.md'])
 f=dict(closed_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),stage_complete=True,whole_research_complete=False,automation='ACTIVE',total_archive_attempts=2,completed_archive_passes=1,archived_policy_act_calls=0,new_physical_runs=0,new_planner_searches=0,cpu_outgoing_query_regression_pass=True,actual_navigation_candidate_tested=False,old_navigation_gate_unchanged_rejected=True,source_checks=len(p['source_sha256']),input_checks=len(p['input_sha256']),reviewed_report_body_sha256=r['reviewed_report_sha256'],final_report_sha256=sha(REPORT),verification_sha256=sha(H/'outgoing_lookahead_verification.json'),review_sha256=sha(H/'outgoing_lookahead_review.json'),final_artifact_sha256={str(x):sha(x) for x in sorted(files)})
 save('outgoing_lookahead_final_identity.json',f)
 text=f'''\n\n{HEADING}

先完整读 `reports/benchmark_outgoing_lookahead_and_heading_chain_20261005.md` 与独立spec/plan `docs/superpowers/{{specs,plans}}/2026-10-05-outgoing-lookahead*.md`。只有CPU最小query修复+一完整归档审计，无新sim/search/fit/inference/视频/正式successrate。CPU query regression PASS，实际新候选导航未测；旧35s cusp timeout4.756419/进度3.707562/RMS.135045/peak.260893/0contact与两对照限定改善REJECT不改。全研究NOTcomplete/automationACTIVE，常规安静。

确定outgoing convention bug：planner _reconstruct在i点赋actions[i]，direction是i→i+1出段；Oracle lookahead先看nextdirection故丢掉当前最后edgeendpoint。version3/6单R段旧targetidx1本身/累积0，实际197PP+17.088480而原名义Rbodyk-.956781/yaw+.287034；还有175首F0→1R也旧targetcurrent导致PP0。独立with_outgoing_endpoint仅先消费CURRENT edge再到nextendpoint；不会进对向下一edge/不改pathidx或latch。make_endpoint_oracle包冻结actualclass，EndpointLookaheadPolicy继承CuspProjectionPolicy只episode reset装配，act/rootplan/原projection/five-low/rebound/PID/BankMap/速度/协议/smoother/planner/mapper/rig/light原样，未bundle入口None Minor。

15字面F/R直线圆弧±k/小误差/边界与终点/完整同向act/五低速反弹真实fixtures，callable RED13fail2pass1.39→GREEN15pass1.36；fullH210pass9.18。初写cwd失败missingfileexit4不当RED，初fullsuite test_board_material缺libtf.so collectFAIL后加已有SDKLD库全pass，fail logs保留无testdisabled。v1审计10:56:13.960111封存125/11310，frame74用了float64diagnostic速度代替原actfloat32proprio norm导致5e-9yaw差/1e-10assertFAIL，无输出data。旧源/protocol/fail日志不改；独立v2只恢复原CURRENT dtype层语义、不放宽gate/不改control，10:57:51.417566封存126/11312。共2尝试(1不完整fail+1complete)明确披露；完整归档审计10:57:52.429785→10:57:56.377453exit0，不冒称singlelaunch。

350records188有效tracking/162无效或早return，188原target/PP/V1inverse/BankMap/raw/protocol全一致，1750旧actualbank/PIDtarget/smoothed/applied与physics逐tick一致；非递归物理预测/不重放policy.act。61zero query改正52raw/32protocol/5main当前代数改变，既有speed策略因新k触发不是新speedpolicy。首次代数分叉175，其他188行各自条件旧实测状态what-if，不是candidate真实trajectory/future标签。R3 |PP|median17.4087→2.3645仍19/19clamp；R6 26.4472→.5216clamp15→6；后F13/17queries仍same/clamp。不能把raw不同且protocol相同当执行改善，不能以counterfactual夹边少称导航成功。

真实旧R3Euler yaw-.053950/world-z integral-.057851与名义+.287034不同，但rootEuler/worldz非同量；bank是asin2(wx+yz) bodyY worldZ，非完整Eulerroll，原PID实际量归属明确。旧R3bankerrRMS.452392/PIDclip100%/supervisor3.16%，F3 .353504/78.46%/4.62%；低supervisor比例不能证限制无影响或删除，PIDclip不能独断gain不足。已知上游target错误可以改变bank请求，欠对准/routeobservedblocked仍有其他因素。已检查图只CURRENTknown refs/实际positions/查询目标，无candidate未来运动。

作者{v['verified_utc']}无science重算hash/唯一行/计数/时序检查126源11312inputs/188rows1750feedback；ONE fresh readonlyreview独立范围/发现/每条declined裁决见outgoing_lookahead_review.json。审查bodySHA {r['reviewed_report_sha256']}，作者最终reportSHA {sha(REPORT)}；作者append/finalcheck不冒称二审，全部源码/REDGREEN/suite/failure/v1v2data/图/旧视频/身份保留，无无关Gitcommit/delete。自动续做仍ACTIVE。

下一阶段唯一未执行物理工作：预封存EndpointLookaheadPolicy同起点/同6s前缀/原35s goldenE1，相对归档cusp solequery因果对照，同时报告较强guarded与已拒绝root。相机/地图/rootgeometry/gains/PID/speed/handler五低速反弹/原acceptance均保持。共同完整physical/joint/exec/smoother/history/source到真实firstdivergence(不能假定197，应可能175)，之后只测candidate自己的applied/XY/bank/yaw/出段/独立进度/RMS/peak/检查点/goal/contact，原改善5项/35sgoal<=.2 0contact不放宽，失败直接reject不扫PID/阈值/网络。CPU修复不等于navigation，RGB+estimatedpose/唯一学习记忆世界模型/正式E0E1E2未准入，全任务继续ACTIVE。本CPUheartbeat不再phys/search/fit。
'''
 for q in docs:
  with q.open('a') as out:out.write(text)
 sync=dict(synchronized_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),stage_complete=True,whole_research_complete=False,automation='ACTIVE',document_sha256={str(q):sha(q) for q in docs+[REPORT,W/'docs/superpowers/plans/2026-10-05-outgoing-lookahead.md']},final_identity_sha256=sha(H/'outgoing_lookahead_final_identity.json'),closure_script_sha256=sha(__file__))
 save('outgoing_lookahead_document_sync.json',sync);print(json.dumps({k:v for k,v in f.items() if k!='final_artifact_sha256'},indent=2))
if __name__=='__main__':main()
