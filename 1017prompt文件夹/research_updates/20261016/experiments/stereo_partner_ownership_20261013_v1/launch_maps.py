from pathlib import Path
import os,sys,json
D=Path(__file__).resolve().parent;sys.path.insert(0,str(D.parents[1]/'experiments/stereo_memory_navigation_20261004_v1'));from durable_stage_supervision_v1 import run_durable
usd=Path('/home/an/miniconda3/envs/ball_wm/lib/python3.10/site-packages/isaacsim/extscache/omni.usd.libs-1.0.1+d02c707b.lx64.r.cp310');env={**os.environ,'PYTHONPATH':str(usd),'LD_LIBRARY_PATH':str(usd/'bin'),'OPENBLAS_NUM_THREADS':'1','PYTHONDONTWRITEBYTECODE':'1'}
r=run_durable([sys.executable,str(D/'maps_and_visuals.py')],D/'map_status.json',D/'maps.log',timeout=1200,env=env);print(json.dumps(r))
