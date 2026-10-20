"""Append-only author closure; no scientific reruns or changes to frozen inputs."""
from pathlib import Path
import datetime, hashlib, json
H=Path(__file__).resolve().parent
W=H.parent.parent
REPORT=W/'reports/benchmark_guarded_reference_retention_20261005.md'
PLAN=W/'docs/superpowers/plans/2026-10-05-guarded-reference-retention.md'
SPEC=W/'docs/superpowers/specs/2026-10-05-guarded-reference-retention-design.md'
MARK='\n## 独立审查与作者收尾'
def sha(f):return hashlib.sha256(Path(f).read_bytes()).hexdigest()
def read(n):return json.loads((H/n).read_text())
def save(n,d):(H/n).write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n')
def verify_hashes(d):
 for f,s in d.items():assert sha(f)==s,f

def main():
 p=read('guarded_reference_protocol.json');v=read('guarded_reference_verification.json');a=read('guarded_reference_assessment.json');r=read('guarded_reference_review.json');st=read('guarded_reference_status.json');pkg=read('guarded_reference_review_package.json')
 assert r['review_count']==1 and r['critical_count']==r['important_count']==0
 assert r['reviewed_report_sha256']==pkg['reviewed_report_sha256']
 assert hashlib.sha256(REPORT.read_bytes().split(MARK.encode())[0]).hexdigest()==r['reviewed_report_sha256']
 assert v['interface_gate_pass'] and a['comparison']['limited_improvement_pass'] and not v['navigation_gate_pass'] and not v['whole_research_complete']
 assert st['status']=='finished' and st['exit_code']==0 and st['maximum_sequences']==p['maximum_sequences']==1
 assert p['episode_budget_s']==a['navigation']['elapsed_s']==35.0
 assert p['sealed_utc']<st['start_utc']<st['end_utc']<v['verified_utc']
 verify_hashes(p['source_sha256']);verify_hashes(p['input_sha256'])
 post=read('guarded_reference_run/artifact_manifest_post_close.json');verify_hashes(post['artifact_sha256'])
 old=read('reference_continuity_document_sync.json')
 for f,s in old['document_sha256'].items():
  assert sha(H/'guarded_reference_pre_stage_ledger.md' if Path(f)==H/'progress.md' else f)==s,f
 heading='## 05:42UTC guarded reference retention — final stage closure'
 docs=[W/'research_mission.md',W/'docs/superpowers/plans/2026-10-04-stereo-memory-navigation.md',H/'progress.md']
 assert all(heading not in f.read_text() for f in docs)
 for f in docs:(H/('guarded_reference_pre_close_'+f.name)).write_bytes(f.read_bytes())
 excluded={'guarded_reference_final_identity.json','guarded_reference_document_sync.json','guarded_reference_final_check.json'}
 files=set(f for f in H.glob('*guarded_reference*') if f.is_file() and f.name not in excluded)
 files.update(f for f in (H/'guarded_reference_run').rglob('*') if f.is_file())
 files.update([REPORT,PLAN,SPEC])
 final=dict(closed_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),stage_complete=True,whole_research_complete=False,automation='ACTIVE',interface_gate_pass=True,limited_improvement_pass=True,navigation_gate_pass=False,new_physical_sequences=1,maximum_sequences=1,source_checks=len(p['source_sha256']),prior_artifact_checks=len(p['input_sha256']),post_close_artifact_checks=len(post['artifact_sha256']),author_verification_sha256=sha(H/'guarded_reference_verification.json'),assessment_sha256=sha(H/'guarded_reference_assessment.json'),reviewed_report_body_sha256=r['reviewed_report_sha256'],final_report_sha256=sha(REPORT),review_sha256=sha(H/'guarded_reference_review.json'),final_artifact_sha256={str(f):sha(f) for f in sorted(files)})
 save('guarded_reference_final_identity.json',final)
 text=f'''\n\n{heading}\n\n先完整读 `reports/benchmark_guarded_reference_retention_20261005.md` 和冻结spec/plan `docs/superpowers/{{specs,plans}}/2026-10-05-guarded-reference-retention*.md`。ONE真实原35s goldenE1已结束05:51:21.407818—05:54:11.093362UTC、退出0，无待续仿真或第二条运行。仅参考替换时机改变；保持原planner/goal/map/rollPID/speed/rig/光照/五低速确认/反弹清零。接口PASS、事前限定改善PASS、导航REJECT(timeout35s/目标剩4.156980m/0实际接触)。全研究NOTcomplete/automationACTIVE，常规安静。\n\n86源/2834旧输入05:51:05.605849封存；350请求/351双方capture/1751physics/2101接触样本，两视频351帧10fps/无尾步。58边界50保留8重建(6success/2failure)，旧suffix/current bridge .6连续和.6栅格、remaining>=.5/projection<=.5，只在原.5s边界检验，不认证动态/3D/unknown安全。首次动作分叉69/6.9s，此前69请求346physics17非raw字段/70完整execution+packet-history EXACT；raw起点-.02s/-1tick分开。249请求block/1393appliedticks实际改变，绝非raw规范化等价。独立冻结评价reference进度4.167298→4.855576m，目标距离4.486646→4.156980，RMS.497267→.174067/peak1.114722→.475266m；2/4m检查点24.9→13.9和34.2→25.3s，6/8未通过，不把索引/短local误差当导航成功。\n\n原真实方向状态17.5—23s经历3.8s制动及1.7s反向跟踪，随后26—35s沿当前local曲线增1.397859m/RMS.045075/peak.102368，但main+.5/83.333%请求裁剪，独立参考偏差仍上升。moving全224帧clip54.464%，V1outside45.089%比旧33.585%更高；不称全链恢复/不把剩余失败全归PID。nominal planner根同时forward/reverse成本0且输入未含实际direction，是初态与执行接口待隔离假设，不是已证明bug或修复。knownpose+真实rendererdepth仍特权诊断层；所有RGBsource/consumer/M2/旧拒绝保持，estimatedpose/学习增量/正式E0E1E2未准入。\n\n162全HCPUtests6.40s，7feature RED+1旧cadence回归；初GREEN fixture误把no-route rebuild当已安装时间，数据前仅修fixture，完整失败log保留。作者full {v['verified_utc']} 86source/2752post-close SHA，351bilateral packet、350causal地图/reference/request/diag/history重建，1745实测feedback执行链一致(首5ticks未克隆，非递归物理预测)，3finite-past checkpoints futurepayload-trap/truncate(非3完整freshphysicsruns)。独立review仅一次，范围/发现与逐条declined裁决见guarded_reference_review.json；原bodySHA {r['reviewed_report_sha256']}，最终作者报告SHA {sha(REPORT)}，追加作者段不冒称再审查。最终closure不重跑科学矩阵；旧源/数据/旧gate不改，归档B0对照不是独立fresh环境泛化证据。\n\n下一项尚未执行的有界任务：对本次17.5s冻结输入做一个根状态成本反事实，比较把原有.5m direction-switch成本应用于当时真实accepted executed direction的首次primitive是否改变选择与可行性。先独立版本TDD/同map/同goal/同原primitive代价和nodebudget，允许必要reverse，不强制forward/不扫描新惩罚/不改PID速度阈值/不再次重做矩阵；归档几何不冒用另一计划真实运动标签。有合理且真实改变的证据才另外封存唯一35s运行；本heartbeat不再新增search/物理序列。来源可靠性、估计位姿和唯一学习记忆增量仍待分层研究，未宣称整个任务完成。\n'''
 for f in docs:
  with f.open('a') as out:out.write(text)
 sync=dict(synchronized_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),stage_complete=True,whole_research_complete=False,automation='ACTIVE',document_sha256={str(f):sha(f) for f in docs+[REPORT,PLAN]},final_identity_sha256=sha(H/'guarded_reference_final_identity.json'),author_verification_sha256=sha(H/'guarded_reference_verification.json'),review_sha256=sha(H/'guarded_reference_review.json'),closure_script_sha256=sha(__file__))
 save('guarded_reference_document_sync.json',sync)
 print(json.dumps({k:x for k,x in final.items() if k!='final_artifact_sha256'},indent=2))
if __name__=='__main__':main()
