"""Two bounded actual navigation diagnostics. Not accepted RGB/formal baselines."""
import subprocess,sys,os,json,datetime
from pathlib import Path
H=Path(__file__).resolve().parent
P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')
status=[]
for case in ('e0','e2'):
    root=H/'risk_runs_capture_transaction'/case
    assert not root.exists()
    command=[sys.executable,str(H/'run_transaction_navigation.py'),'--manifest',str(P/'configs/benchmark_v1/golden_v1_manifest.json'),'--split','golden','--episode-id',f'golden_{case}_0001','--policy','goal_greedy','--scene-root','/home/an/project/NavDP/assets/scenes','--data-root',str(P/'data'),'--run-root',str(root),'--lighting-profile','fixed_camera_fill_v1','--collision-mode','physical_contact_v2','--headless','--record-video']
    print('START',case,flush=True);start=datetime.datetime.now(datetime.timezone.utc).isoformat()
    with (H/f'transaction_risk_{case}.log').open('w') as f:
        r=subprocess.run(command,stdout=f,stderr=subprocess.STDOUT,env=dict(os.environ,PYTHONPATH=str(P),OPENBLAS_NUM_THREADS='1',MPLCONFIGDIR='/tmp/mpl_stereo_memory',INFORMATION_MODE='visible',MAP_REPRESENTATION='surface'))
    row=dict(case=case,exit_code=r.returncode,start_utc=start,end_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),command=command)
    status.append(row);(H/'transaction_risk_status.json').write_text(json.dumps(status,indent=2)+'\n')
    print('FINISH',case,r.returncode,flush=True)
    if r.returncode:raise RuntimeError(case)
print('TWO_ACTUAL_RISK_RUNS_COMPLETE',flush=True)
