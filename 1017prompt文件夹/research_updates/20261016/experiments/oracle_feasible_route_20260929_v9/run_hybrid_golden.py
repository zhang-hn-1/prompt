import os,sys,json,hashlib,subprocess,time,tarfile
from pathlib import Path
h=Path(__file__).resolve().parent;worker=h/'hybrid';wm=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm');root=wm.parent.parent
files=[wm/'benchmark/isaac_runtime.py',wm/'benchmark/policies.py',wm/'benchmark/kinodynamic_planning.py',wm/'benchmark/runner.py',wm/'scripts/run_pointgoal_benchmark.py']+sorted(worker.glob('*.py'))+[worker/'bank_mapping.json',worker/'selected_candidate.json']
identity={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files};(h/'hybrid_golden_source_identity.json').write_text(json.dumps(identity,indent=2))
with tarfile.open(h/'hybrid_golden_source_snapshot.tar.gz','w:gz') as t:
 for p in files:t.add(p,arcname=str(p).lstrip('/'))
env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OMP_NUM_THREADS='2',MKL_NUM_THREADS='2',MPLCONFIGDIR='/tmp/ball_wm_mpl',PYTHONPATH=f'/home/an/.cache/ball_wm_visibility_numba_0612:{wm}:{wm}/scripts')
status=[]
for case in ('golden_e0_0001','golden_e1_0001','golden_e2_0001'):
 cmd=[sys.executable,str(worker/'run_navigation_worker.py'),'--manifest',str(wm/'configs/benchmark_v1/golden_v1_manifest.json'),'--split','golden','--episode-id',case,'--policy','oracle','--scene-root','/home/an/project/NavDP/assets/scenes','--data-root',str(wm/'data'),'--run-root',str(h/f'navigation_hybrid_golden_{case}'),'--headless','--record-video','--seed','3072','--oracle-lookahead-m','.5','--auxiliary-velocity-limit-rad-s','.30','--drive-profile',str(wm/'configs/benchmark_v1/drive_dynamics_v1.json'),'--collision-mode','physical_contact_v2']
 (h/f'hybrid_golden_{case}_command.json').write_text(json.dumps(cmd,indent=2));print('START',case,flush=True);t=time.time()
 with (h/f'hybrid_golden_{case}.log').open('w') as f:r=subprocess.run(cmd,env=env,stdout=f,stderr=subprocess.STDOUT,cwd=root)
 status.append(dict(case=case,returncode=r.returncode,wall_s=time.time()-t));(h/'hybrid_golden_status.json').write_text(json.dumps(status,indent=2));print('DONE',case,r.returncode,flush=True)
 if r.returncode:break
(h/'hybrid_golden_identity_after.json').write_text(json.dumps({p:hashlib.sha256(Path(p).read_bytes()).hexdigest()==v for p,v in identity.items()},indent=2))
