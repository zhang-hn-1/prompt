"""Verify recorded source/asset identity; rerun one complete suite into a NEW directory."""
import argparse,hashlib,json,os,subprocess,sys
from pathlib import Path
h=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('--check-only',action='store_true');p.add_argument('--suite',choices=['hard_0'],default='hard_0');p.add_argument('--run-root',type=Path);a=p.parse_args()
x=json.loads((h/'navigation_source_identity.json').read_text())
for path,digest in x.items():
 assert hashlib.sha256(Path(path).read_bytes()).hexdigest()==digest,path
for row in json.loads((h/'asset_verification.json').read_text()):
 assert hashlib.sha256(Path(row['path']).read_bytes()).hexdigest()==row['sha256'],row['path']
print('Frozen sources and assets match',flush=True)
if a.check_only:raise SystemExit(0)
if a.run_root is None or a.run_root.exists():raise SystemExit('--run-root must name a new output directory')
cmd=json.loads((h/(a.suite+'_command.json')).read_text());cmd[0]=sys.executable;cmd[1]=str(h/'run_navigation_worker.py');cmd[cmd.index('--run-root')+1]=str(a.run_root.resolve())
wm=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm');env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OMP_NUM_THREADS='2',MKL_NUM_THREADS='2',MPLCONFIGDIR='/tmp/ball_wm_mpl',PYTHONPATH=f'/home/an/.cache/ball_wm_visibility_numba_0612:{wm}:{wm}/scripts')
raise SystemExit(subprocess.call(cmd,env=env))
