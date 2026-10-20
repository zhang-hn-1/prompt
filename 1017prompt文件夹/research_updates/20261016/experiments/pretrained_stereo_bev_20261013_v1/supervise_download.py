from pathlib import Path
import sys,json,os
D=Path(__file__).resolve().parent;H=D.parent/'stereo_memory_navigation_20261004_v1';sys.path.insert(0,str(H));from durable_stage_supervision_v1 import run_durable
assert not (D/'weight_download_resume_status.json').exists(),'no duplicate dispatch'
(D/'download_attempts.json').write_text(json.dumps({'cap':2,'used':2,'first_exec':78210,'first_host':[2482594,2482596],'first_returncode':143,'first_state':'interrupted_partial_preserved','science_fit':0,'science_physical':0,'science_recovery':0,'resume_once':True},indent=2)+'\n')
r=run_durable([sys.executable,str(D/'fetch_weights.py')],D/'weight_download_resume_status.json',D/'weight_download_resume.log',timeout=3600,grace=20,env={**os.environ,'PYTHONUTF8':'1','PYTHONDONTWRITEBYTECODE':'1'});print(json.dumps(r),flush=True)
