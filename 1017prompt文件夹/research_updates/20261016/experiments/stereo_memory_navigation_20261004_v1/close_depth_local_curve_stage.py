"""Post-review identity and append-only mission synchronization. No scientific rerun."""
from pathlib import Path
import json,hashlib,datetime
H=Path(__file__).resolve().parent;W=H.parent.parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
 review=json.loads((H/'depth_local_curve_review.json').read_text());assert review['critical_count']==0 and review['important_count']==0
 v=json.loads((H/'depth_local_curve_verification.json').read_text());assert v['interface_gate_pass'] and not v['navigation_gate_pass'] and not v['whole_research_complete']
 p=json.loads((H/'depth_local_curve_protocol.json').read_text());status=json.loads((H/'depth_local_curve_status.json').read_text());assert status['status']=='finished' and status['exit_code']==0
 assert p['sealed_utc']<status['start_utc']<v['verified_utc']
 for f,s in p['source_sha256'].items():assert sha(f)==s,f
 post=json.loads((H/'depth_local_curve_run/artifact_manifest_post_close.json').read_text())
 for f,s in post['artifact_sha256'].items():assert sha(f)==s,f
 prior=json.loads((H/'oracle_rgb_shadow_final_identity.json').read_text())
 for f,s in prior['final_artifact_sha256'].items():assert sha(f)==s,f
 report=W/'reports/benchmark_depth_local_curve_navigation_20261005.md';prefix=report.read_bytes().split('\n## 独立审查与作者收尾'.encode())[0];assert hashlib.sha256(prefix).hexdigest()==review['reviewed_report_sha256']
 heading='## 03:39UTC privileged depth local curve — final stage closure'
 docs=[W/'research_mission.md',W/'docs/superpowers/plans/2026-10-04-stereo-memory-navigation.md',H/'progress.md']
 assert all(heading not in f.read_text() for f in docs)
 oldsync=json.loads((H/'oracle_rgb_shadow_document_sync.json').read_text())
 for f,s in oldsync['document_sha256'].items():
  if Path(f)==H/'progress.md':assert sha(H/'depth_local_curve_pre_stage_ledger.md')==s
  else:assert sha(f)==s,f
 for f in docs:(H/('depth_local_curve_pre_close_'+f.name)).write_bytes(f.read_bytes())
 files=sorted(set([f for f in (H/'depth_local_curve_run').rglob('*') if f.is_file()]+[f for f in H.glob('*depth_local_curve*') if f.is_file() and f.name not in ['depth_local_curve_final_identity.json','depth_local_curve_document_sync.json','depth_local_curve_final_check.json']]+[report]))
 closure=dict(closed_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),stage_complete=True,whole_research_complete=False,automation='ACTIVE',navigation_gate_pass=False,interface_gate_pass=True,full_reconstruction_verified_utc=v['verified_utc'],full_verification_sha256=sha(H/'depth_local_curve_verification.json'),source_unchanged_since_preseal=True,post_close_outputs_unchanged=True,reviewed_body_sha256=review['reviewed_report_sha256'],author_final_report_sha256=sha(report),prior_artifact_checks=len(prior['final_artifact_sha256']),source_checks=len(p['source_sha256']),post_close_artifact_checks=len(post['artifact_sha256']),review_sha256=sha(H/'depth_local_curve_review.json'),final_artifact_sha256={str(f):sha(f) for f in files})
 (H/'depth_local_curve_final_identity.json').write_text(json.dumps(closure,indent=2)+'\n')
 text=f'''\n\n{heading}\n\n先完整读 `reports/benchmark_depth_local_curve_navigation_20261005.md` 和本轮冻结spec/plan `docs/superpowers/{{specs,plans}}/2026-10-05-depth-local-curve*.md`。本阶段ONE真实原35s goldenE1已结束03:52:31.736585—03:55:56.221204UTC、退出0；没有待续仿真或第二条运行。接口通过、导航REJECT(timeout35/4.486646m/路程5.890657m/实际2101可用接触样本0)。350动作/351双方capture/1751physics、两视频各351帧10fps/无5s视频尾步。全研究未完成，automationACTIVE，不能作为完整结果通知。\n\n独立privileged rendererdepth+knownpose来源地图→局部target2.4m/crop±4m/原HybridA*node3000→ConfidentSpeed/MappedBankOracle/OraclePathPolicy→原rollPID(.4,.1,.7)、lookahead.5、auxsupervisor.3。外层privilegedFalse只拒绝runner全局Oracle输入，传感仍privileged；全图/全局参考不得入外层，全球goal允许。Unknown labels独立，规划副本风险探索不认证安全，无footprintseed/raycarving/GTfilter/futureinputs。初次局部reference初始化，后续仅替换几何，五低速/反弹/方向/历史/smoother状态保持。冻结Oracle .5/1/1.5主轴域与旧InformationPolicy .3 unknowncap不同，且6s零请求前缀计35s，不能单因素A/B归因或冒称RGB/完整3D认证。72source于03:52:11.003572UTC preseal后保持；post-close2762outputs覆盖bankfeedback，初始manifest只pre-close。\n\n58plansALLsuccess；localtarget world-y符号7.5s(-1.4→+1.4)、16s(+2.2→-2.0)两次变动，与来源hazard变化关联，未隔离单个格因果或形式拓扑。仅一次请求forward→reverse16s，actual18.5s，原合理制动2.5s；不称方向反复/resetbug。265有效movingtracking，protocolclip186(70.19%)、bankcurvatureclamp183、V1outside89(33.58%)。独立旧评价参考finalprogress4.167298/RMS.497267/peak1.114722m，不作为当前planner输入或纯trackingerror。0.5s重新起始参考exitRMS.027668/peak.080303不能证长期成功；保存冻结5s描述窗口，无另一计划实际label冒用。当前/历史floor31/1715、hazard63/163、unknown158028；depth层有来源floor，不改变旧ORBfloor0/RGBsourceconsumer REJECT结论。planp50.797596/p951.243388s，policy p95.940341/capturepartialp95.297808；causalonline≠wall10Hz。\n\n148CPUtests4.39s；作者full{v['verified_utc']}72source+2762postcloseSHA、351双方depthpacket、350真实请求/地图/局部reference/historyshift精确重建，3finitepaststate clones futurepayload/truncation一致；不是3完整freshphysicsruns。全部1750raw physics请求对齐，实测feedback执行链1745ticks重建一致，首5ticks不克隆/不是递归物理预测。初次assessment pending保留，随后verification interfacePASS/navREJECT为当前状态。预数据fixtureV1aux0非理想直行、boundaryhazard首次未RED、seal terminalnewline/initialcapture-reset调用序问题均如实归档，全部封存前解决；没有数据后调参/重复候选。ONE独立只读review范围/发现/declined rulings见depth_local_curve_review.json；原审查正文SHA {review['reviewed_report_sha256']}，最终作者报告SHA {sha(report)}，后续作者append分开。最终轻量identity只封存/复查现有完整科学输出，旧2513SHA与新72源一致，不重做矩阵。\n\n接续尚未执行：从本次完整58map/reference数据先核验合法参考连续性候选，已有走廊仍受观测支持时稳定reference消费；真实新hazard失效允许明确重建，不锁死绕行/强制方向。仅一个参考消费修改，经CPU几何证据/预sealed独立进度、空间检查点、目标、接触gate后，最多一条同起点/同depth-risk-domain/同6s前缀/同原35s候选对本次B0。保持planner算法、PID、speed、mapping、rig/oldgates，候选不合理则拒绝；本heartbeat不再仿真。局部层未通过，RGB+估计位姿/唯一学习增量/正式35sE0E1E2未触发。常规安静，ACTIVE。\n'''
 for f in docs:
  with f.open('a') as out:out.write(text)
 sync=dict(synchronized_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),stage_complete=True,whole_research_complete=False,automation='ACTIVE',document_sha256={str(f):sha(f) for f in docs+[report,W/'docs/superpowers/plans/2026-10-05-depth-local-curve.md']},final_identity_sha256=sha(H/'depth_local_curve_final_identity.json'),full_verification_sha256=sha(H/'depth_local_curve_verification.json'),review_sha256=sha(H/'depth_local_curve_review.json'),closure_script_sha256=sha(__file__))
 (H/'depth_local_curve_document_sync.json').write_text(json.dumps(sync,indent=2)+'\n');print(json.dumps({k:z for k,z in closure.items() if k!='final_artifact_sha256'},indent=2))
if __name__=='__main__':main()
