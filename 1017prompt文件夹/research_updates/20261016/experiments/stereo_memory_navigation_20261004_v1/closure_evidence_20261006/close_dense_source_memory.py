from pathlib import Path
import json,hashlib,datetime,re
W=Path('/home/an/Documents/ChatGPT/球型机器人导航');H=W/'experiments/stereo_memory_navigation_20261004_v1';sha=lambda f:hashlib.sha256(f.read_bytes()).hexdigest();now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat()
report=W/'reports/benchmark_dense_source_memory_and_startup_support_20261006.md';review=json.loads((H/'dense_source_memory_review.json').read_text());assert review['counts']==dict(critical=0,important=0,minor=1);archive=H/'dense_source_memory_reviewed_body.md';assert not archive.exists();archive.write_bytes(report.read_bytes())
app='''\n\n## 一次独立终审与作者裁决（原正文按字节保留）

ONE fresh readonly reviewer C0/I0/M1，亲自核595 SHA、9 sources/196 inputs、旧mapper身份、498 sweep与321states、saved统计、11 fixtures/.21s、6个精确actual-time前缀与未来payload陷阱、3帧实际双目投影符号。354项suite、全部260map重建、完整GT标签、旧模型/采集与PNG检查为作者证据，未由审查者重跑；无GPU/fit/physics。

M1 deferred：dense_source_points.py 将 confidence 转 bool 前没有验证数字有限性；NaN小样例会生成两个点。审查者逐个确认此次130输入均有限bool，故本封存结果不受影响、两层REJECT保持。作者接受该限制，接口限定为本次有限bool封存输入；未来非封存部署须独立拒绝NaN/非bool，不改这次封存源码、不重跑科学或再审。正文“有限置信度”只描述这130实际输入，不是任意输入API保证。下一perfect-source分支不调用point_packet，不能把未修接口当成修好。

精确浮点时钟说明：author nominal t16 trap证明截断不变，但不等于frame160 snapshot的TTL边界；reviewer另使用实际15.999999642372131s等savedtime验证六前缀，保持旧结果。

作者对declined范围逐项裁决：无新GPU/Isaac、无infer/fit/采集、无完整科学回放重跑、无runtime/control/nav准入、无部署/commit/source edit、无二审/额外代理、无整任务完成。各项维持未验证/未执行；若混称通过会造成准入或身份误报，因此不外推。后续行政closure及新prior实验不在此review覆盖；证据归属按实际执行者标注。\n'''
report.write_bytes(archive.read_bytes()+app.encode())
masters=[W/'research_mission.md',W/'docs/superpowers/plans/2026-10-04-stereo-memory-navigation.md',H/'progress.md']
pre={}
for i,f in enumerate(masters):
 dest=H/('dense_source_memory_preclose_'+['mission','plan','progress'][i]+'.md');assert not dest.exists();dest.write_bytes(f.read_bytes());pre[str(f)]=dict(archive=str(dest),old_sha256=sha(dest),bytes=dest.stat().st_size)
 text='''\n\n## 2026-10-06 稠密空间消费拒绝并实际接续独立起步条件

最新 reports/benchmark_dense_source_memory_and_startup_support_20261006.md。ONE既有65帧260map只读knownpose回放20:48:45→49:06UTC，原模型点29409/floor70全错误；adapter点40363/floor208中172错高度、188三维>.1、20正确，history最多4/17floorcells。两臂65次当前disk/498实际未来sweep支持全0；t6运动前185/185disk格未知。旧source90/99/.1gate和nav/pose拒绝保持；未来trajectory仅postmanifestofflineassessor。595SHA/260rebuild/6futuretraps作者证据，354suite10.84s，ONEreviewC0I0M1，NaN→bool接口局限独立append/此次130bool输入不影响；原正文按字节保留。无新physics/infer/fit/nav/rates。

已实际接续 startup-prior-separation 分支：spec/plan与H/startup_prior_protocol.json、startup_prior_status.json、run_startup_prior_diagnostic.py。原方案允许另列safe-start；固定t6初始185footprint格只标task_assumption、不随robot移动、不进observedmap、hazard优先，perfect当前renderer双目可见端点明确privileged、非RGB。9source66rawinput封存后ONE65帧消费正在执行，先检查实际进程/日志，勿重复启动。4prior与3projection literalRED→GREEN；下一postmanifest评currentdisk与futureactual498sweep、未知outsideprior，禁止拿prior解除旧coldstart/3D/controlgate。不改TTL/radius/mask/rig/light/model，不无限换网/Oracle标定。整体未完成、automationACTIVE。\n'''
 with f.open('a') as stream:stream.write(text)
 with (H/'dense_source_memory_ledger.md').open('a') as stream:stream.write('\nFinal review C0I0M1; minor(deferred) confidence NaN bool accepted outside current finitebool domain; no currentresultimpact, no frozenedit/review. Declinedscopes ruled unvalidated in report. Task3 stageclosed scienceREJECT; actualstartupconditionalconsumer started separately, missionNOTcomplete.\n')
checks=0;p=json.loads((H/'dense_source_memory_protocol.json').read_text())
for kind in ('source_sha256','input_sha256'):
 for name,d in p[kind].items():assert sha(Path(name))==d,name;checks+=1
for name,old in pre.items():assert Path(name).read_bytes()[:old['bytes']]==Path(old['archive']).read_bytes();assert sha(Path(old['archive']))==old['old_sha256']
owned=[report,archive]+list(H.glob('dense_source_memory*'))+[H/'dense_source_points.py',H/'test_dense_source_points.py',H/'seal_dense_source_memory.py',H/'run_dense_source_memory.py',H/'assess_dense_source_memory.py',H/'verify_dense_source_memory.py',H/'verify_dense_source_memory_v2.py']+[Path(n) for n in p['source_sha256']]
files=set()
for f in owned:
 if f.is_dir():files.update(x for x in f.rglob('*') if x.is_file())
 elif f.is_file():files.add(f)
files={f for f in files if f.name not in ['dense_source_memory_final_identity.json','dense_source_memory_final_check.json']}
idf=H/'dense_source_memory_final_identity.json';idf.write_text(json.dumps(dict(utc=now(),owned_sha256={str(f):sha(f) for f in sorted(files)},master_prefix_archives=pre,whole_research_complete=False),indent=2)+'\n')
for f,d in json.loads(idf.read_text())['owned_sha256'].items():assert sha(Path(f))==d,f
check=dict(utc=now(),administrative_pass=True,source_models='REJECT',consumer_necessary_support='REJECT',presealed_checks=checks,owned_files=len(files),review=dict(critical=0,important=0,minor=1),whole_research_complete=False,next_actual_status=str(H/'startup_prior_status.json'))
(H/'dense_source_memory_final_check.json').write_text(json.dumps(check,indent=2)+'\n');print(json.dumps(check,indent=2))
