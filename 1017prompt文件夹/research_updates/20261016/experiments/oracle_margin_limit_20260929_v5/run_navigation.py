import os,sys,subprocess,json,hashlib,time,tarfile
from pathlib import Path
h=Path(__file__).resolve().parent;root=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark');wm=root/'research/ball_wm'
env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OMP_NUM_THREADS='2',MKL_NUM_THREADS='2',MPLCONFIGDIR='/tmp/ball_wm_mpl',PYTHONPATH=f'/home/an/.cache/ball_wm_visibility_numba_0612:{wm}:{wm}/scripts')
old=json.loads((h.parent/'oracle_long_hold_navigation_20260928_v1/navigation_source_identity.json').read_text());assert all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==v for p,v in old.items())
paths=[Path(p) for p in old]+sorted(h.glob('*.py'))+[h/'selected_candidate.json',h/'bank_mapping.json',h/'PROTOCOL.md']
identity={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths};(h/'navigation_source_identity.json').write_text(json.dumps(identity,indent=2))
with tarfile.open(h/'navigation_source_snapshot.tar.gz','w:gz') as tar:
 for p in paths:tar.add(p,arcname=str(p).lstrip('/'))
status=[]
suites=[('margin_0002','hard_0_dev_manifest.json','dev'),('margin_0004','hard_0_dev_manifest.json','dev')]
for name,manifest,split in suites:
 cmd=[sys.executable,str(h/'run_navigation_worker.py'),'--manifest',str(wm/'configs/benchmark_v1'/manifest),'--split',split,'--policy','oracle','--scene-root','/home/an/project/NavDP/assets/scenes','--data-root',str(wm/'data'),'--run-root',str(h/('navigation_'+name)),'--headless','--record-video','--seed','3072','--oracle-lookahead-m','.5','--auxiliary-velocity-limit-rad-s','.30','--drive-profile',str(wm/'configs/benchmark_v1/drive_dynamics_v1.json')]
 cmd.extend(['--episode-id','hard_0_e1_'+name[-4:]])
 (h/(name+'_command.json')).write_text(json.dumps(cmd,indent=2));print('START_NAVIGATION',name,flush=True);t=time.time()
 with (h/(name+'_navigation.log')).open('w') as f:r=subprocess.run(cmd,env=env,stdout=f,stderr=subprocess.STDOUT,cwd=root)
 status.append(dict(suite=name,returncode=r.returncode,wall_s=time.time()-t));(h/'navigation_status.json').write_text(json.dumps(status,indent=2));print('DONE_NAVIGATION',name,r.returncode,flush=True)
assert all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==v for p,v in identity.items())
(h/'identity_verified.json').write_text(json.dumps({'unchanged':True,'count':len(identity)}))
