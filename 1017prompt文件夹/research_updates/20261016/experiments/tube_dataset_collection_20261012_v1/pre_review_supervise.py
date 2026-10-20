"""Bounded sequential schedule, stops after first failed data gate; no retry."""
from pathlib import Path
import sys,json,hashlib,argparse,os,datetime
ROOT=Path(__file__).resolve().parents[2];D=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'experiments/stereo_memory_navigation_20261004_v1'))
from durable_stage_supervision_v1 import run_durable
from driver import validate_admission
from import_group import read_group

def main():
 parser=argparse.ArgumentParser();parser.add_argument('--admission',required=True);args=parser.parse_args();validate_admission(args.admission)
 schedule=json.loads((D/'bound_schedule_v1.json').read_text());ledger_path=D/'collection_status_v1.json'
 ledger=dict(status='starting',parent_pid=os.getpid(),used=0,cap=30,retries=0,groups=[],fit_admitted=False)
 with ledger_path.open('x') as f:json.dump(ledger,f)
 def save():
  temp=ledger_path.with_suffix('.tmp');temp.write_text(json.dumps(ledger,indent=2)+'\n');os.replace(temp,ledger_path)
 try:
  for group in schedule['groups']:
   name=group['group'];run=D/'runs'/name;status=D/(name+'_status.json');log=D/(name+'_log.txt');ledger.update(status='running',current_group=name);save()
   result=run_durable([sys.executable,str(D/'driver.py'),'--admission',args.admission,'--group',name,'--out',str(run)],status,log,timeout=1200,grace=45);ledger['used']+=1
   files=list(run.rglob('*')) if run.exists() else [];files=[f for f in files if f.is_file()]+[status,log];seal=D/(name+'_seal.json')
   seal.write_text(json.dumps(dict(durable_status=result,artifact_sha256={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in files}),indent=2)+'\n')
   if result['status']!='finished':ledger['groups'].append(dict(group=name,durable_status=result));ledger['status']='execution_stopped';break
   record=read_group(run,status,seal,group);ledger['groups'].append(record)
   if record['scorable']/70<.95 or len(record['support_cells'])<8:ledger['status']='coverage_rejected';break
  else:ledger['status']='collection_completed_pending_full_review'
 finally:
  ledger['unattempted']=30-ledger['used'];ledger['end_utc']=datetime.datetime.now(datetime.timezone.utc).isoformat();save()
if __name__=='__main__':main()
