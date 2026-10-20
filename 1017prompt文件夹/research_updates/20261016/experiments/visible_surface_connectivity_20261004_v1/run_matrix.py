"""Frozen eight-run matrix; fresh physics for both arms, no replay."""
import datetime,json,os,subprocess,sys
from pathlib import Path
H=Path(__file__).resolve().parent
P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')
CASES=(('golden_e0_0001','e0','golden_v1_manifest.json','golden'),('golden_e2_0001','e2','golden_v1_manifest.json','golden'),('hard_0_e1_0003','h3','hard_0_dev_manifest.json','dev'),('hard_0_e1_0004','h4','hard_0_dev_manifest.json','dev'))
status=[]
for representation in ('points','surface'):
 for episode,case,manifest,split in CASES:
  root=H/'runs'/representation/case
  assert not root.exists(), 'refuse overwrite/reuse actual runs'
  command=[sys.executable,str(H/'run_information.py'),'--manifest',str(P/'configs/benchmark_v1'/manifest),'--split',split,'--episode-id',episode,'--policy','goal_greedy','--scene-root','/home/an/project/NavDP/assets/scenes','--data-root',str(P/'data'),'--run-root',str(root),'--lighting-profile','fixed_camera_fill_v1','--collision-mode','physical_contact_v2','--headless','--record-video']
  print('START',representation,case,flush=True);start=datetime.datetime.now(datetime.timezone.utc).isoformat()
  with (H/f'{representation}_{case}.log').open('w') as log:
   result=subprocess.run(command,env=dict(os.environ,PYTHONPATH=str(P),OPENBLAS_NUM_THREADS='1',MPLCONFIGDIR='/tmp/mpl_surface_stage',INFORMATION_MODE='visible',MAP_REPRESENTATION=representation),stdout=log,stderr=subprocess.STDOUT)
  episodes=list(root.glob('episode_*/metrics.json'))
  good=result.returncode==0 and len(episodes)==1 and (root/'physics.npz').exists() and (root/'vision_records.json').exists() and (root/'belief_final.npz').exists() and (root/'camera_clocks.json').exists()
  if good:
   metrics=json.loads(episodes[0].read_text());good=float(metrics['elapsed_time_s'])>=34.99 if 'elapsed_time_s' in metrics and metrics['termination_reason']=='timeout' else good
  row=dict(representation=representation,case=case,episode=episode,start_utc=start,end_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),exit_code=result.returncode,artifacts_complete=bool(good),command=command)
  status.append(row);(H/'run_status.json').write_text(json.dumps(status,indent=2)+'\n')
  print('FINISH',representation,case,result.returncode,good,flush=True)
  if not good:raise RuntimeError('run failed or artifacts incomplete')
print('EIGHT_ACTUAL_RUNS_COMPLETE',flush=True)
