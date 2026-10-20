"""Read-only preparation diagnostics. Never grants a launch or starts Isaac."""
from pathlib import Path
import json,hashlib,importlib.util,importlib.metadata,subprocess,shutil,sys,argparse
HERE=Path(__file__).resolve().parent

def file_issues(table):
 issues=[]
 for name,digest in table.items():
  p=Path(name)
  if not p.is_file():issues.append(dict(path=name,reason='missing_file'));continue
  if hashlib.sha256(p.read_bytes()).hexdigest()!=digest:issues.append(dict(path=name,reason='sha_mismatch'))
 return issues

def decision(**flags):
 required=['identity_match','assets_present','modules_discoverable','gpu_visible']
 if set(flags)!=set(required):raise ValueError('explicit preflight flags required')
 return dict(**flags,runtime_preflight_ready=all(flags.values()),physics_launch_admitted=False,
  scene_runtime_validated=False,transitive_USD_assets_validated=False,portable_installation_complete=False,
  reliable_navigation_complete=False,whole_mission_complete=False)

def build_parser():
 parser=argparse.ArgumentParser();parser.add_argument('--requirements',type=Path,default=HERE/'requirements_v3.json');return parser

def main():
 a=build_parser().parse_args();req=json.loads(a.requirements.read_text());issues=file_issues(req['frozen_identity_sha256']);assets=[]
 for path in req['primary_assets']:
  p=Path(path);assets.append(dict(path=path,present=p.is_file(),sha256=hashlib.sha256(p.read_bytes()).hexdigest() if p.is_file() else None))
 modules=[]
 for name in req['modules']:
  found=importlib.util.find_spec(name) is not None
  versions={}
  if found:
   for package in importlib.metadata.packages_distributions().get(name,[]):
    versions[package]=importlib.metadata.version(package)
  modules.append(dict(module=name,discoverable=found,distributions=versions))
 program=shutil.which('nvidia-smi');gpu=dict(program=program,visible=False,returncode=None)
 if program:
  try:
   r=subprocess.run([program,'--query-gpu=name,driver_version','--format=csv,noheader'],capture_output=True,text=True,timeout=15)
   gpu.update(returncode=r.returncode,visible=r.returncode==0 and bool(r.stdout.strip()),stdout=r.stdout.strip(),stderr=r.stderr.strip())
  except (OSError,subprocess.TimeoutExpired) as e:gpu['error']=str(e)
 flags=decision(identity_match=not issues,assets_present=all(x['present'] for x in assets),modules_discoverable=all(x['discoverable'] for x in modules),gpu_visible=gpu['visible'])
 result=dict(**flags,python=dict(executable=sys.executable,version=sys.version),identity_issues=issues,verified_identities=len(req['frozen_identity_sha256']),primary_assets=assets,modules=modules,gpu=gpu,scope='Discovery only, no runtime module import, model load, USD reference resolution or physics.')
 print(json.dumps(result,indent=2));return 0 if flags['runtime_preflight_ready'] else 3
if __name__=='__main__':sys.exit(main())
