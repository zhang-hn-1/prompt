"""Evidence-based closure of one failed endpoint-consumer branch, not whole research."""
from pathlib import Path
import json,hashlib,datetime
H=Path(__file__).resolve().parent;W=H.parent.parent;sha=lambda f:hashlib.sha256(Path(f).read_bytes()).hexdigest()
def main():
 v=json.loads((H/'point_support_verification.json').read_text());r=json.loads((H/'point_support_review.json').read_text());assert r['critical_count']==0 and r['important_count']==0
 for f,s in v['source_and_artifact_sha256'].items():assert sha(f)==s,f
 report=W/'reports/benchmark_point_support_shadow_20261005.md';assert not v['consumer_support_gate_passed'] and not v['source_gate_passed'] and not v['whole_research_complete']
 files=[W/'research_mission.md',W/'docs/superpowers/plans/2026-10-04-stereo-memory-navigation.md',H/'progress.md',W/'docs/superpowers/plans/2026-10-05-point-support-shadow.md']
 heading='## 01:38UTC endpoint support shadow — final stage closure'
 assert all(heading not in f.read_text() for f in files),'already synchronized'
 text=f'''

{heading}

先完整阅读 `reports/benchmark_point_support_shadow_20261005.md`。一次真实历史回放181帧/362地图；190点/代码/参考身份在01:47:24.005734UTC封存，全部输出01:47:26.673320UTC保存后才重读离线标签。没有重采、重推理、训练、改动作或新视频；旧实际18秒双目视频作为来源。两地图都接收完全相同的causal当前query估计，current无旧query记录、history10秒原query时间；current不等于单帧双目，既往深度增益不混作本轮地图记忆效果。

实际568保留点全部world-Z .936991—1.015382m、nominal floor0；height/uncertainty筛选前，也没有任何点的nominal XY进入完整扫掠333格。起点187/转弯300/自身匹配停车225/whole333格在每臂每181时刻全部未知，所有phase0次通过，floor格全0。6秒开始运动前仍无起点支持；18秒事后249记忆也未补足。strict unknown-blocked语义下消费者拒绝，不能用已经走过/无接触回填观测free或调半径/放大稀疏点。

history在6秒252记录/248正确/4错误；末态249记录/203有标签200正确/46无标签3错误247旧query。hazard代理峰值20末态17，current峰值4末态1；history与旧occupancy自由格重叠峰值11末态10只是2D/高度/跨格代理，不自动等于真实错误物体。false-free0仅因floor空集合，不是安全通过。no ray-carving/interpolation/bodyseed/model-imputed-area；sigma只是扰动敏感性/knownpose_sigma0层；floor代理不是全体积认证，source旧拒绝保持。

六个literal消费fixture RED6→GREEN6；文档完整身份fixture另RED1→GREEN，最终135CPU测试通过（实际日志保存）。作者最终 {v['verified_utc']}：{v['old_sha_checks']}去重旧/预封存身份，362地图重建、181packet/旧有效FeatureMemory点等价/未来payload与截断，4冻结raster几何+4独立continuousprojection格集合、1448直接map索引分类。原计划上轮append closing使全文SHA变，原采集前prefixSHA75204b3a…和recorded closure SHAda551acb…都逐字节验证并归档，不略过任意变更；所有其他旧源/数据/报告匹配。新增fixture曾并入封存测试文件，已精确恢复原6fixture、另文件保存新1fixture/合并中间版；首次split缺pytest的失败日志保存，最终135 green。作者一次重复1448geometry核验被主动中断后改为4geometry+1448map-index，仅核验进程、非实验中断，不少查数据。所有失败与版本保存，模型/mapper/runner/190协议/362地图/数据/gate未改，没有实验重跑。

一次fresh-context只读审查范围和发现见 `point_support_review.json`，作者复核不冒充其执行。两图实际检查；所有输出日志另封存，避免循环自散列。报告最终SHA `{sha(report)}`；最终{len(v['source_and_artifact_sha256'])}新产物身份和文档同步单独保存。任务阶段完成、全研究未完成，automation ACTIVE，常规不发完成通知。

关闭“冻结ORB+这条主动观察+保守endpoint支持解决严格冷启动”分支；没有近场floor/扫掠支持是具体阻塞，不能推出所有像素/所有方法/固定rig永久不可导航。停止追加动作、sigma/半径/TTL/网络扫描和同一局部诊断循环。

下一项回到完整控制/在线系统基线：先检查是否已有相同当前rig与控制身份的完整Oracle结果，存在则复用，不重做矩阵；核验 `H/reusable_oracle_audit.json` 及 `experiments/oracle_feasible_route_20260929_v9/hybrid/` 的实际class/继承/act/PID/协议/环境身份。旧wrapper9同SHA、旧core两份变化/旧environment缺rig，不能由名称继承旧3/3、5/7。必要时最多一条原35秒完整golden E1的已知地图Oracle诊断，同时真正在线运行仅当前/过去RGB+本层knownpose的只读来源shadow；参考、增益、速度配置保留旧成功控制器，不将其当原InformationPolicy的单层修复或同时调参。shadow不改B0动作，全图/全局参考不得写进RGB观测地图，旧source/consumer gate不解除。保存完整路径/真实到达/接触/时延/图像/执行/身份，不报告纯RGB或正式E0/E1/E2矩阵成功率。Oracle成功只说明控制上限，不能替代视觉基线；若要safe-start/已知地面先验必须单列prior任务条件，禁止冒充观测。估计位姿/唯一学习增量/原35秒正式分层结果仍按总计划，未完成不宣称完成。
'''
 for f in files:
  with f.open('a') as out:out.write(text)
 result=dict(synchronized_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),stage_complete=True,whole_research_complete=False,automation='ACTIVE',source_rejected=True,consumer_rejected=True,document_sha256={str(f):sha(f) for f in files+[report]},verification_sha256=sha(H/'point_support_verification.json'),review_sha256=sha(H/'point_support_review.json'),source_sync_script_sha256=sha(__file__),completed_verification_log_sha256={str(f):sha(f) for f in H.glob('point_support_verify*.log')})
 (H/'point_support_document_sync.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
if __name__=='__main__':main()
