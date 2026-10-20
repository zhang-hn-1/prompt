import json,os,subprocess,sys
from pathlib import Path
H=Path(__file__).resolve().parent;P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')
status=[]
for mode in ('map','visible'):
 for episode,label,manifest,split in (('golden_e0_0001','e0','golden_v1_manifest.json','golden'),('golden_e2_0001','e2','golden_v1_manifest.json','golden'),('hard_0_e1_0003','h3','hard_0_dev_manifest.json','dev'),('hard_0_e1_0004','h4','hard_0_dev_manifest.json','dev')):
  root=H/'runs'/mode/label
  if root.exists():continue
  cmd=[sys.executable,str(H/'run_information.py'),'--manifest',str(P/'configs/benchmark_v1'/manifest),'--split',split,'--episode-id',episode,'--policy','goal_greedy','--scene-root','/home/an/project/NavDP/assets/scenes','--data-root',str(P/'data'),'--run-root',str(root),'--lighting-profile','fixed_camera_fill_v1','--collision-mode','physical_contact_v2','--headless','--record-video']
  print('Starting',mode,label,flush=True)
  with (H/f'{mode}_{label}.log').open('w') as f:r=subprocess.run(cmd,env=dict(os.environ,PYTHONPATH=str(P),OPENBLAS_NUM_THREADS='1',MPLCONFIGDIR='/tmp/mpl_information',INFORMATION_MODE=mode),stdout=f,stderr=subprocess.STDOUT)
  status.append(dict(mode=mode,episode=episode,label=label,exit_code=r.returncode,command=cmd));(H/'run_status.json').write_text(json.dumps(status,indent=2)+'\n')
  print('Finished',mode,label,r.returncode,flush=True)
  if r.returncode:raise SystemExit(r.returncode)
