import os,sys,subprocess,json,time,hashlib,tarfile
from pathlib import Path
h=Path(__file__).resolve().parent;root=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark');wm=root/'research/ball_wm'
env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OMP_NUM_THREADS='2',MKL_NUM_THREADS='2',MPLCONFIGDIR='/tmp/ball_wm_mpl',PYTHONPATH=f'/home/an/.cache/ball_wm_visibility_numba_0612:{wm}:{wm}/scripts')
paths=[wm/'benchmark/isaac_runtime.py',wm/'benchmark/policies.py',wm/'benchmark/runner.py',wm/'scripts/run_pointgoal_benchmark.py',wm/'scripts/action_smoothing.py']+list((h/'pid').glob('*'))+list((h/'course').glob('*'))
identity={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths};(h/'source_identity.json').write_text(json.dumps(identity,indent=2))
with tarfile.open(h/'source_snapshot.tar.gz','w:gz') as t:
 for p in paths:t.add(p,arcname=str(p).lstrip('/'))
status=[]
cases=[('pid_0002_contact','pid','hard_0_dev_manifest.json','dev','hard_0_e1_0002','physical_contact_v2'),('course_0004_contact','course','hard_0_dev_manifest.json','dev','hard_0_e1_0004','physical_contact_v2'),('golden_e0_contact','pid','golden_v1_manifest.json','golden','golden_e0_0001','physical_contact_v2'),('golden_e1_contact','pid','golden_v1_manifest.json','golden','golden_e1_0001','physical_contact_v2'),('pid_0002_legacy','pid','hard_0_dev_manifest.json','dev','hard_0_e1_0002','map_clearance_v1')]
# Use the manifest's frozen split, not an assumed suite label.
for name,worker,manifest,split,episode,mode in cases:
 data=json.loads((wm/'configs/benchmark_v1'/manifest).read_text())
 entries=data['episodes'] if isinstance(data,dict) else data
 selected=next(e for e in entries if e['episode_id']==episode);split=selected['split']
 cmd=[sys.executable,str(h/worker/'run_navigation_worker.py'),'--manifest',str(wm/'configs/benchmark_v1'/manifest),'--split',split,'--episode-id',episode,'--policy','oracle','--scene-root','/home/an/project/NavDP/assets/scenes','--data-root',str(wm/'data'),'--run-root',str(h/name),'--headless','--record-video','--seed','3072','--oracle-lookahead-m','.5','--auxiliary-velocity-limit-rad-s','.30','--drive-profile',str(wm/'configs/benchmark_v1/drive_dynamics_v1.json'),'--collision-mode',mode]
 (h/(name+'_command.json')).write_text(json.dumps(cmd,indent=2));print('START',name,flush=True);start=time.time()
 with (h/(name+'.log')).open('w') as f:r=subprocess.run(cmd,env=env,stdout=f,stderr=subprocess.STDOUT,cwd=root)
 status.append(dict(case=name,returncode=r.returncode,wall_s=time.time()-start));(h/'status.json').write_text(json.dumps(status,indent=2));print('DONE',status[-1],flush=True)
 if r.returncode:break
(h/'identity_after.json').write_text(json.dumps({p:hashlib.sha256(Path(p).read_bytes()).hexdigest()==v for p,v in identity.items()},indent=2))
