"""Author closure of one actual35s stage. No policy/search/sim/new gate."""
from pathlib import Path
import datetime, hashlib, json
H=Path(__file__).resolve().parent
W=H.parent.parent
REPORT=W/'reports/benchmark_cusp_projection_navigation_20261005.md'
HEADING='## 09:45UTC cusp projection actual35s — final stage closure'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
read=lambda n:json.loads((H/n).read_text())
def save(n,d):(H/n).write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n')
def main():
 p=read('cusp_navigation_protocol.json');a=read('cusp_navigation_assessment.json');v=read('cusp_navigation_verification.json');r=read('cusp_navigation_review.json');pkg=read('cusp_navigation_review_package.json');st=read('cusp_navigation_status.json');m=read('cusp_navigation_motion_audit.json')
 assert r['review_count']==1 and r['critical_count']==r['important_count']==0
 assert st['status']=='finished' and st['exit_code']==0 and st['maximum_sequences']==1
 assert v['interface_gate_pass'] and not v['navigation_gate_pass']
 assert not a['limited_improvement_both_comparators_pass'] and not a['navigation_gate_pass']
 assert a['navigation']['elapsed_s']==35 and a['acquisition']['actual_contact_samples']==0
 assert m['all_acceptances_have_five_current_low'] and len(m['accepted_transitions'])==4
 body=REPORT.read_bytes().split('\n## 独立审查与作者收尾'.encode())[0]
 assert hashlib.sha256(body).hexdigest()==r['reviewed_report_sha256']==pkg['reviewed_report_sha256']
 for mapping in [p['source_sha256'],p['input_sha256']]:
  for f,d in mapping.items():assert sha(f)==d,f
 manifest=read('cusp_navigation_run/artifact_manifest_post_close.json')
 for f,d in manifest['artifact_sha256'].items():assert sha(f)==d,f
 old=read('cusp_projection_document_sync.json')
 for f,d in old['document_sha256'].items():
  target=H/'cusp_navigation_pre_stage_ledger.md' if Path(f)==H/'progress.md' else Path(f)
  assert sha(target)==d,f
 docs=[W/'research_mission.md',W/'docs/superpowers/plans/2026-10-04-stereo-memory-navigation.md',H/'progress.md']
 for f in docs:
  assert HEADING not in f.read_text()
  (H/('cusp_navigation_pre_close_'+f.name)).write_bytes(f.read_bytes())
 excluded={'cusp_navigation_final_identity.json','cusp_navigation_document_sync.json','cusp_navigation_final_check.json'}
 files={f for f in H.glob('*cusp_navigation*') if f.is_file() and f.name not in excluded}
 files.update([REPORT,W/'docs/superpowers/specs/2026-10-05-cusp-navigation-design.md',W/'docs/superpowers/plans/2026-10-05-cusp-navigation.md'])
 ident=dict(closed_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),stage_complete=True,whole_research_complete=False,automation='ACTIVE',actual_sequences=1,original_episode_budget_s=35,interface_gate_pass=True,limited_improvement_both_comparators_pass=False,navigation_gate_pass=False,source_checks=len(p['source_sha256']),input_checks=len(p['input_sha256']),actual_artifact_checks=len(manifest['artifact_sha256']),reviewed_report_body_sha256=r['reviewed_report_sha256'],final_report_sha256=sha(REPORT),verification_sha256=sha(H/'cusp_navigation_verification.json'),review_sha256=sha(H/'cusp_navigation_review.json'),final_artifact_sha256={str(f):sha(f) for f in sorted(files)})
 save('cusp_navigation_final_identity.json',ident)
 text=f'''\n\n{HEADING}

先完整读 `reports/benchmark_cusp_projection_navigation_20261005.md` 与独立冻结spec/plan `docs/superpowers/{{specs,plans}}/2026-10-05-cusp-navigation*.md`。ONE同起点goldenE1原35s于09:49:35.045858→09:52:28.764418UTC结束exit0，无遗留仿真/第二候选，旧root/guarded不重跑。接口PASS，限定改善对两臂均REJECT，导航REJECT(timeout35s/goalremain4.756419/0contact)。全研究NOTcomplete/automationACTIVE，常规安静，不替换较强归档B0。

只安装冻结CuspProjectionPolicy投影边界修复：未accepted当前方向期间保持索引，资格后原投影；legitimate replacement仍取消过时pending，不强制提前反转。外层继承RootDirectionPolicy act/plan_boundary，只episode reset换tracker；inner真实ConfidentSpeed/MappedBankOracle/CuspBoundary/Oracle原act/handler。runtime5处机械diff；原planner/rootcost.5/reverse1.25/node3000/geometry/guard/mapper/PID/speed/five-low/rebound/rig/light保持。仍是双rendererdepth+knownpose/未知风险，不是RGB估计pose/认证安全，outerprivilegedFalse不洗掉sensor privilege；旧RGB/sourceconsumer/M2/停止包络拒绝保留。

119源/8509输入09:49:05.318957presealed，实际350请求351双方capture1751physics2101contactsamples0actual，两视频351帧无尾步/hiddenmarkers/0age，2752关闭后manifest含feedback。首分叉182/18.2s相对root：911physics17nonraw/183execution+来源historyEXACT，raworigin-.02s/-1tick另报；141rawblocks840appliedticks改变。不能称与guarded到182全共同前缀，它与root此前175已经分叉。

四次19.7/23.7/29.7/33.3 accepted F/R交接，均真实五个当前10HzXY<.05，原反弹语义保留。180负rawticks278负applied428实测main负各自分母；197acceptedR时main还+.040957/previousapplied0，215main-.460650/previousapplied-.5/XY倒车。version3/6各约.299mR段，自己reference寿命内实际最接近终点.044042/.013268m、有向投影max.359736/.390492m，不能仅用index或accepted认证primitive。离线描述auditpostseal不改control/gate，lifespan负ticks不专属单primitive。

独立全参考终态progress3.707562、RMS.135045/peak.260893；相对rootprogress-1.682213/goalremain+1.085041，相对较强guardedprogress-1.148014/goalremain+.599439。虽然误差/0contact三项过，进度/goal两项失败，两臂gate均拒绝；4m检查点27.4s，6/8未实际到达。74direction-change零块=7.4s，合理停稳不当bug；25/25.5s observed-obstacle route失败、26s新F/R/F导致第二循环，不把首交接当全部失败。19.7→21.5实际yaw-1.178943→-1.224909；23.7前向接受时reference-.737970/actual-1.198351误差.460381，24.9仍.433406。是真实欠对准证据，未证PID/符号/映射单因果。

作者{v['verified_utc']}核验119源/2752actualSHA、350causalpolicy/map/reference/request精确、351双方packet/350单次history、1745实测feedback执行tick（首5不clone）、3finitepastfuturetrap/truncatePASS；非递归物理预测/非3freshphysics。195/8.72与11RED/GREEN是前阶段冻结证据未冒称新跑；wall10Hz不成立，planp95 3.444s/capturepartialp95.311s另披露。继承SDKhiddenmarkerprimvarwarnings留rawlog，actualexit0不称blocked。原机械plot继承错label文件保留，独立postruncomparison_v2三实际臂更正且视觉检查。ONEfreshreadonlyreview范围/发现/每条declined裁决见cusp_navigation_review.json；审查bodySHA {r['reviewed_report_sha256']}、作者最终reportSHA {sha(REPORT)}；作者append与最终hashcheck不冒称二次review。保存全部来源/数据/失败/测试/视频/版本，无无关Gitcommit/delete。

下一项尚未实施的有界CPU工作：针对version3/6实测R tracking和资格后F tracking，核验reference body/profile与PP/feedback实际曲率合成、V1请求方向、roll反馈/PID/applied、实际yaw/XYcourse，尤其23.7/33.3欠对准和短R名义yaw与实际变化；字面直线/圆弧/正反向/连续性fixture，不二次act/不拿braking陈旧值/不默认符号bug。明确错误才选择唯一传统tracking最小修复，无错则拒绝该假设并隔离动态欠对准/来源route阻塞证据，不扫PID阈值/换M2M3网络。实际候选需另事前封存，同初态35s与原进度/goal/横向/contact gate；本heartbeat不追加search/sim。RGB+estimatedpose/唯一学习记忆世界模型/正式E0E1E2仍待分层推进，全任务未完成，保持ACTIVE。
'''
 for f in docs:
  with f.open('a') as out:out.write(text)
 sync=dict(synchronized_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),stage_complete=True,whole_research_complete=False,automation='ACTIVE',document_sha256={str(f):sha(f) for f in docs+[REPORT,W/'docs/superpowers/plans/2026-10-05-cusp-navigation.md']},final_identity_sha256=sha(H/'cusp_navigation_final_identity.json'),closure_script_sha256=sha(__file__))
 save('cusp_navigation_document_sync.json',sync)
 print(json.dumps({k:v for k,v in ident.items() if k!='final_artifact_sha256'},indent=2))
if __name__=='__main__':main()
