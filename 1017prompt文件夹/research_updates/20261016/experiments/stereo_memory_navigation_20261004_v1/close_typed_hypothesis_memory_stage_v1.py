"""Finite completion follower: never launches simulation or changes actor/model."""
from pathlib import Path
import os,sys,time,json,datetime,hashlib,subprocess
H=Path(__file__).resolve().parent;W=H.parent.parent;S=H/'typed_hypothesis_memory_stage_status_v1.json';C=H/'typed_hypothesis_memory_completion_follower_v1.json';assert not C.exists();now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat();sha=lambda b:hashlib.sha256(b).hexdigest();r=dict(utc=now(),pid=os.getpid(),stage='WAIT_EXISTING_PAIR_NO_DUPLICATE',maximum_wait_s=2400,simulation_launches=0,whole_research_complete=False)
def save():C.write_text(json.dumps(r,indent=2)+'\n')
save();begin=time.monotonic()
while time.monotonic()-begin<2400:
 try:s=json.loads(S.read_text())
 except json.JSONDecodeError:time.sleep(5);continue
 if s['stage']!='RUN_ACTUAL_CURRENT_VS_STATIC_SEED':break
 time.sleep(10)
else:r.update(stage='WAIT_LIMIT_PRESERVED_NEEDS_RESUME',end_utc=now());save();sys.exit(2)
r['actual_status']=s;r['stage']='VERIFY_EXISTING_CLOSED_PAIR';save()
if s['stage']!='BOTH_ACTUAL_ARMS_CLOSED_NEEDS_VERIFICATION':r.update(stage='ACTUAL_INCOMPLETE_PRESERVED_NO_RETRY',end_utc=now());save();sys.exit(3)
env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTHONPATH='/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm',MPLCONFIGDIR='/tmp/mpl_typed_hypothesis_closure',OPENBLAS_NUM_THREADS='1');r['checks']=[]
for script,log in [('verify_typed_hypothesis_memory_causal_v1.py','typed_hypothesis_memory_causal_verification_v1.log'),('assess_typed_hypothesis_memory_v2.py','typed_hypothesis_memory_assessment_v2.log')]:
 with (H/log).open('x') as f:code=subprocess.call([sys.executable,str(H/script)],env=env,stdout=f,stderr=subprocess.STDOUT)
 r['checks'].append(dict(script=script,log=log,returncode=code));save()
 if code:r.update(stage='VERIFICATION_ERROR_PRESERVED_NEEDS_CAUSAL_REPAIR',end_utc=now());save();sys.exit(code)
a=json.loads((H/'typed_hypothesis_memory_assessment_v2.json').read_text());r.update(stage='PAIR_CLOSED_WITH_ACTUAL_ASSESSMENT',decision=a['decision'],end_utc=now(),not_formal35=True,whole_research_complete=False);save()
p=W/'reports/benchmark_typed_hypothesis_memory_closed_loop_20261007.md';p.write_text('# Typed RGB hypothesis memory: actual closed-loop pair\n\n'+a['decision']+'\n\nSame fixed single-cylinder task,4.4m goal,60s local budget, known-pose privileged diagnostic. B0initial RGB possible-set retention10s, C1retention60s. Only retention differs; moving predictions never enter actor; no observed-free/occupied claims; original metric-depth/full-class/global-source rejects retained. Not original E1/formal35/estimated-pose/world-model success.\n\n```json\n'+json.dumps(dict(arms=a['arms'],checks=a['checks'],prefix_frames=a['prefix_frames'],strict_causal_AB_claim=a['strict_causal_AB_claim'],maximum_physical_prefix_error=a['maximum_physical_prefix_error']),indent=2)+'\n```\n\nEvidence: typed_hypothesis_memory_assessment_v2.json; typed_hypothesis_memory_causal_verification_v1.json; both actual arm RGB/poses/proprio/typed maps/physics/clocks/actions/source and videos with post-close manifests. CPU automation executes frozen author checks; reviewer had only read-only source/focused CPU review. Full research incomplete; next source/control-layer decision must follow these actual outcomes, without resuming old rejected matrices.\n')
text='\n\n## '+now()+' — typed memory pair closed by finite verification follower\n'+a['decision']+'; '+json.dumps(dict(arms=a['arms'],checks=a['checks'],strict_causal_AB_claim=a['strict_causal_AB_claim']),ensure_ascii=False)+'\nActual raw evidence retained, no new simulation by follower. Next: personally inspect frozen check outputs, trajectory and videos; one bounded causal audit if candidate rejected, or next estimated-pose layer if local closure accepted. Do not rerun closed pair/source matrix. Whole research incomplete; no formal35/world-model/estimated-pose completion.\n';trace=[]
for f in [W/'research_mission.md',H/'progress.md',W/'docs/superpowers/plans/2026-10-04-stereo-memory-navigation.md']:
 b=f.read_bytes();n=b+text.encode();f.write_bytes(n);trace.append(dict(file=str(f),old_bytes=len(b),old_sha256=sha(b),new_sha256=sha(n),prefix_preserved=True))
r['append_trace']=trace;r['next_entry']='Inspect typed_hypothesis_memory_assessment_v2.json and actual figure/videos; causal layer audit or estimated-pose layer, no duplicate launches';save();print(a['decision'],flush=True)
