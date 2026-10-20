"""Replay this frozen navigation evaluation in the same installed environment."""
from pathlib import Path
import argparse,json,hashlib,os,subprocess,sys
p=argparse.ArgumentParser();p.add_argument('--run-root',type=Path);p.add_argument('--check-only',action='store_true');p.add_argument('--suite',choices=['all','golden','hard_0'],default='all');a=p.parse_args();h=Path(__file__).resolve().parent
identity=json.loads((h/'navigation_source_identity.json').read_text());bad=[name for name,value in identity.items() if not Path(name).is_file() or hashlib.sha256(Path(name).read_bytes()).hexdigest()!=value]
for row in json.loads((h/'asset_verification.json').read_text()):
 if hashlib.sha256(Path(row['path']).read_bytes()).hexdigest()!=row['current_sha256']:bad.append(row['path'])
if bad:raise SystemExit('Frozen source/assets differ; restore snapshot separately.\n'+'\n'.join(bad))
print('Frozen source and assets verified',flush=True)
if a.check_only:raise SystemExit(0)
if a.run_root is None:p.error('--run-root required unless --check-only')
a.run_root=a.run_root.resolve();a.run_root.mkdir(parents=True,exist_ok=False)
wm=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm');env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OMP_NUM_THREADS='2',MKL_NUM_THREADS='2',MPLCONFIGDIR='/tmp/ball_wm_mpl',PYTHONPATH=f'/home/an/.cache/ball_wm_visibility_numba_0612:{wm}:{wm}/scripts')
for suite in (['golden','hard_0'] if a.suite=='all' else [a.suite]):
 cmd=json.loads((h/(suite+'_command.json')).read_text());cmd[0]=sys.executable;cmd[cmd.index('--run-root')+1]=str(a.run_root/('navigation_'+suite));(a.run_root/(suite+'_command.json')).write_text(json.dumps(cmd,indent=2));print('Running',suite,flush=True)
 with (a.run_root/(suite+'.log')).open('w') as log:r=subprocess.run(cmd,env=env,stdout=log,stderr=subprocess.STDOUT,cwd=wm.parents[1])
 if r.returncode:raise SystemExit(r.returncode)
 print('Completed',suite,flush=True)
