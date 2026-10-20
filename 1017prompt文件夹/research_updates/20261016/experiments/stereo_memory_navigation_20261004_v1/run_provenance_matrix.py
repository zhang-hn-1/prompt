import json,os,subprocess,sys,datetime
from pathlib import Path
H=Path(__file__).resolve().parent
P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')
protocol=json.loads((H/'online_provenance_protocol.json').read_text());status=[]
for mode in protocol['arms']:
    root=H/'online_provenance_runs'/mode;assert not root.exists()
    command=[sys.executable,str(H/'run_provenance_navigation.py'),'--manifest',str(P/'configs/benchmark_v1/golden_v1_manifest.json'),'--split','golden','--episode-id',protocol['episode_id'],'--policy','goal_greedy','--scene-root','/home/an/project/NavDP/assets/scenes','--data-root',str(P/'data'),'--run-root',str(root),'--lighting-profile','fixed_camera_fill_v1','--collision-mode','physical_contact_v2','--headless','--record-video']
    print('START',mode,flush=True);start=datetime.datetime.now(datetime.timezone.utc).isoformat()
    with (H/f'online_provenance_{mode}.log').open('w') as stream:
        process=subprocess.run(command,stdout=stream,stderr=subprocess.STDOUT,env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTHONPATH=str(P),OPENBLAS_NUM_THREADS='1',MPLCONFIGDIR='/tmp/mpl_stereo_memory',INFORMATION_MODE='visible',MAP_REPRESENTATION='points',MEMORY_MODE=mode))
    status.append(dict(mode=mode,exit_code=process.returncode,command=command,start_utc=start,end_utc=datetime.datetime.now(datetime.timezone.utc).isoformat()))
    (H/'online_provenance_status.json').write_text(json.dumps(status,indent=2)+'\n')
    print('FINISH',mode,process.returncode,flush=True)
    if process.returncode:raise RuntimeError(mode)
