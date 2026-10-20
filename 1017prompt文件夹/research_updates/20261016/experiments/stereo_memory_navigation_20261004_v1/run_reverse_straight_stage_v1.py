from pathlib import Path
import json,datetime,time,subprocess,sys,hashlib,argparse
H=Path(__file__).resolve().parent;parser=argparse.ArgumentParser();parser.add_argument('--arm',required=True,choices=['development','validation']);arm=parser.parse_args().arm
p=H/('reverse_straight_'+arm+'_status_v1.json');assert not p.exists();stamp=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat();s=dict(stage='RUNNING_ONE32S_SEQUENCE',utc=stamp(),maximum_sequences=2,arm=arm,whole_research_complete=False,not_navigation=True);p.write_text(json.dumps(s,indent=2)+'\n');t=time.monotonic();child=None;code=1
try:
 with (H/('reverse_straight_'+arm+'_run_v1.log')).open('x') as log:
  child=subprocess.Popen([sys.executable,str(H/'run_reverse_straight_v1.py'),'--arm',arm],stdout=log,stderr=subprocess.STDOUT,cwd=H);s['child_pid']=child.pid;p.write_text(json.dumps(s,indent=2)+'\n')
  try:code=child.wait(timeout=900)
  except subprocess.TimeoutExpired:
   child.terminate()
   try:child.wait(timeout=30)
   except subprocess.TimeoutExpired:child.kill();child.wait(timeout=30)
   code=124
finally:
 s.update(stage='ACTUAL_CLOSED_NEEDS_VERIFICATION' if code==0 else 'FAILED_NO_RETRY',complete=code==0,returncode=code,end_utc=stamp(),wall_s=time.monotonic()-t);D=H/('reverse_straight_'+arm+'_run_v1');files={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in D.rglob('*') if f.is_file()};man=D/'post_close_manifest.json'
 if D.exists():man.write_text(json.dumps(dict(artifact_sha256=files),indent=2)+'\n');s['manifest_sha256']=hashlib.sha256(man.read_bytes()).hexdigest()
 p.write_text(json.dumps(s,indent=2)+'\n')
sys.exit(code)
