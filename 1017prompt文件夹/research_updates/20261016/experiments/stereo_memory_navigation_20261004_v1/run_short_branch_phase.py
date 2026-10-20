import argparse,datetime,json,os,subprocess,sys
from pathlib import Path
H=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('--phase',choices=['development','fresh'],required=True);args=p.parse_args()
command=[sys.executable,str(H/'run_short_branch.py'),'--phase',args.phase]
statuspath=H/('short_branch_'+args.phase+'_status.json')
row=dict(phase=args.phase,command=command,start_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),status='running')
statuspath.write_text(json.dumps(row,indent=2)+'\n')
with (H/('short_branch_'+args.phase+'.log')).open('w') as stream:
    result=subprocess.run(command,stdout=stream,stderr=subprocess.STDOUT,env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTHONPATH='/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm',OPENBLAS_NUM_THREADS='1',MPLCONFIGDIR='/tmp/mpl_stereo_memory'))
row.update(status='finished',exit_code=result.returncode,end_utc=datetime.datetime.now(datetime.timezone.utc).isoformat());statuspath.write_text(json.dumps(row,indent=2)+'\n')
print(json.dumps(row,indent=2))
if result.returncode:raise RuntimeError('bounded short sequence runtime failure; retain logs, no automatic rerun')
