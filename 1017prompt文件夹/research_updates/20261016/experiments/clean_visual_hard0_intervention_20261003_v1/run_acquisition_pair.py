import json,os,subprocess,sys
from pathlib import Path
H=Path(__file__).parent;P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')
status=[]
for mode in ('original','every_render'):
 episode='hard_0_e1_0003';root=H/'acquisition_pair'/mode;assert not root.exists()
 cmd=[sys.executable,str(H/'run_acquisition.py'),'--manifest',str(P/'configs/benchmark_v1/hard_0_dev_manifest.json'),'--split','dev','--episode-id',episode,'--policy','goal_greedy','--scene-root','/home/an/project/NavDP/assets/scenes','--data-root',str(P/'data'),'--run-root',str(root),'--lighting-profile','fixed_camera_fill_v1','--headless','--record-video']
 print('Starting',mode,flush=True)
 with (H/f'acquisition_{mode}.log').open('w') as f:r=subprocess.run(cmd,env=dict(os.environ,PYTHONPATH=str(P),OPENBLAS_NUM_THREADS='1',MPLCONFIGDIR='/tmp/mpl_visual_hard0',VISION_ACQUISITION_MODE=mode),stdout=f,stderr=subprocess.STDOUT)
 status.append({'mode':mode,'exit_code':r.returncode,'command':cmd});(H/'acquisition_status.json').write_text(json.dumps(status,indent=2))
 print('Finished',mode,r.returncode,flush=True)
 if r.returncode:break
