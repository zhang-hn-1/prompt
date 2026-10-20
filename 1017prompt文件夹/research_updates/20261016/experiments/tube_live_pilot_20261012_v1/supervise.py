"""One no-retry durable launch, preserving partial evidence after child exit."""
from pathlib import Path
import argparse,sys,json,hashlib,datetime
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'experiments/stereo_memory_navigation_20261004_v1'))
from durable_stage_supervision_v1 import run_durable

def main():
 p=argparse.ArgumentParser();p.add_argument('--admission',required=True);args=p.parse_args();directory=Path(__file__).resolve().parent;out=directory/'run_v1'
 status=directory/'run_status_v1.json';log=directory/'run_durable_v1.log'
 result=run_durable([sys.executable,str(directory/'driver.py'),'--admission',args.admission,'--out',str(out)],status,log,timeout=1200,grace=45)
 files=[f for f in out.rglob('*') if f.is_file()] if out.exists() else []
 files += [status,log]
 manifest=directory/'supervisor_seal_v1.json'
 with manifest.open('x') as f:json.dump(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),durable_status=result,artifact_sha256={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in files},interruption_not_navigation_failure=True,whole_mission_complete=False),f,indent=2)
 return 0 if result['status']=='finished' else 1
if __name__=='__main__':raise SystemExit(main())
