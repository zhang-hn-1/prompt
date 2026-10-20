"""Exactly one preregistered actual candidate, existing history B0 remains frozen."""
import datetime,json,os,subprocess,sys
from pathlib import Path
H=Path(__file__).resolve().parent
P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')
protocol=json.loads((H/'local_stopping_protocol.json').read_text())
root=H/'local_stopping_run';assert not root.exists()
command=[sys.executable,str(H/'run_stopping_navigation.py'),'--manifest',str(P/'configs/benchmark_v1/golden_v1_manifest.json'),'--split','golden','--episode-id',protocol['episode_id'],'--policy','goal_greedy','--scene-root','/home/an/project/NavDP/assets/scenes','--data-root',str(P/'data'),'--run-root',str(root),'--lighting-profile','fixed_camera_fill_v1','--collision-mode','physical_contact_v2','--headless','--record-video']
status=dict(command=command,start_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),status='running')
(H/'local_stopping_status.json').write_text(json.dumps(status,indent=2)+'\n')
with (H/'local_stopping_run.log').open('w') as stream:
    result=subprocess.run(command,stdout=stream,stderr=subprocess.STDOUT,env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTHONPATH=str(P),OPENBLAS_NUM_THREADS='1',MPLCONFIGDIR='/tmp/mpl_stereo_memory',INFORMATION_MODE='visible',MAP_REPRESENTATION='points',MEMORY_MODE='history'))
status.update(exit_code=result.returncode,end_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),status='finished')
(H/'local_stopping_status.json').write_text(json.dumps(status,indent=2)+'\n')
print(status,flush=True)
raise SystemExit(result.returncode)
