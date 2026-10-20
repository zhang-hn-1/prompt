from pathlib import Path
import os,sys,subprocess,json,hashlib,tarfile,time
h=Path(__file__).resolve().parent;root=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark');wm=root/'research/ball_wm';env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OMP_NUM_THREADS='2',MKL_NUM_THREADS='2',MPLCONFIGDIR='/tmp/ball_wm_mpl',PYTHONPATH=f'/home/an/.cache/ball_wm_visibility_numba_0612:{wm}:{wm}/scripts');paths=sorted(p for folder in ['benchmark','scripts','training','configs'] for p in (wm/folder).rglob('*') if p.is_file() and p.suffix in ['.py','.json','.yaml'])
identity={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths};(h/'navigation_source_identity.json').write_text(json.dumps(identity,indent=2))
with tarfile.open(h/'navigation_source_snapshot.tar.gz','w:gz') as tar:
 for p in paths:tar.add(p,arcname=p.relative_to(root))
status=[]
for name,manifest,split in [('golden','golden_v1_manifest.json','golden'),('hard_0','hard_0_dev_manifest.json','dev')]:
 cmd=[sys.executable,str(wm/'scripts/run_pointgoal_benchmark.py'),'--manifest',str(wm/'configs/benchmark_v1'/manifest),'--split',split,'--policy','oracle','--scene-root','/home/an/project/NavDP/assets/scenes','--data-root',str(wm/'data'),'--run-root',str(h/('navigation_'+name)),'--headless','--record-video','--seed','3072','--oracle-lookahead-m','.5','--auxiliary-velocity-limit-rad-s','.30','--drive-profile',str(wm/'configs/benchmark_v1/drive_dynamics_v1.json')]
 (h/(name+'_command.json')).write_text(json.dumps(cmd,indent=2));print('START_NAVIGATION',name,flush=True);start=time.time()
 with (h/(name+'_navigation.log')).open('w') as f:result=subprocess.run(cmd,env=env,stdout=f,stderr=subprocess.STDOUT,cwd=root)
 status.append({'suite':name,'returncode':result.returncode,'wall_s':time.time()-start});(h/'navigation_status.json').write_text(json.dumps(status,indent=2));print('DONE_NAVIGATION',name,result.returncode,flush=True)
 # Preserve failures and still evaluate remaining suite. No parameter retries.
assert all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==v for p,v in identity.items()),'Source changed during navigation'
(h/'navigation_identity_verified.json').write_text(json.dumps({'unchanged':True,'file_count':len(identity)},indent=2))
