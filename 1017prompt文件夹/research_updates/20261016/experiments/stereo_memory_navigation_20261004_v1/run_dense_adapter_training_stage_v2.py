import os,sys,json,datetime,subprocess
from pathlib import Path
H=Path(__file__).resolve().parent;now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat();status=H/'dense_adapter_v2_pipeline_status.json';assert not status.exists()
row=dict(start_utc=now(),status='running',pid=os.getpid(),stages=[]);status.write_text(json.dumps(row,indent=2)+'\n')
for stage in ['fit_dense_adapter_v2','predict_dense_adapter_v2','assess_dense_adapter_v2']:
 row['current_stage']=stage;row['stage_start_utc']=now();status.write_text(json.dumps(row,indent=2)+'\n')
 with (H/(stage+'.log')).open('w') as f:r=subprocess.run([sys.executable,str(H/(stage+'.py'))],stdout=f,stderr=subprocess.STDOUT,env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',TORCH_COMPILE_DISABLE='1'))
 row['stages'].append(dict(name=stage,start_utc=row['stage_start_utc'],end_utc=now(),exit_code=r.returncode));status.write_text(json.dumps(row,indent=2)+'\n')
 if r.returncode:row.update(end_utc=now(),status='failed',exit_code=r.returncode);status.write_text(json.dumps(row,indent=2)+'\n');raise SystemExit(r.returncode)
row.update(end_utc=now(),status='finished',exit_code=0);status.write_text(json.dumps(row,indent=2)+'\n');print(json.dumps(row,indent=2))
