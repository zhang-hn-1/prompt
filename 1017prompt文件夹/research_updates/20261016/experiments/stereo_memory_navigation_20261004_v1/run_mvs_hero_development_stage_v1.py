"""Bounded stage continuation: wait existing acquisition/review, run exactly once."""
from pathlib import Path
import json,datetime,time,os,subprocess,hashlib,urllib.request
H=Path(__file__).resolve().parent;A=H/'mvsanywhere_source_preflight_v1';STATUS=H/'mvs_hero_development_stage_status_v1.json';assert not STATUS.exists();start=time.monotonic();env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',MPLCONFIGDIR='/tmp/mpl-ball-navigation',PYTHONUNBUFFERED='1');python='/home/an/miniconda3/envs/ball_wm/bin/python'
def status(stage,**kw):STATUS.write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),pid=os.getpid(),stage=stage,elapsed_s=time.monotonic()-start,**kw),indent=2)+'\n')
def resume_one_partial():
 # One administrative HTTP range continuation of same official bytes, never another model.
 old=json.loads((A/'artifact_acquisition_status_v1.json').read_text());partial=A/'weights/mvsanywhere_hero.ckpt.partial';offset=partial.stat().st_size;url='https://storage.googleapis.com/niantic-lon-static/research/mvsanywhere/mvsanywhere_hero.ckpt';status('one_same_artifact_range_resume',offset=offset,old_acquisition_failure=old);t=time.monotonic();request=urllib.request.Request(url,headers={'User-Agent':'ball-navigation-research','Range':f'bytes={offset}-'})
 with urllib.request.urlopen(request,timeout=40) as r,partial.open('ab') as f:
  assert r.status==206 and r.headers['Content-Range'].startswith(f'bytes {offset}-');total=int(r.headers['Content-Range'].split('/')[-1]);size=offset
  while True:
   if time.monotonic()-t>900:raise TimeoutError('single fixed900s resume budget exceeded')
   block=r.read(1024**2)
   if not block:break
   f.write(block);size+=len(block)
   if size%(8*1024**2)==0:status('one_same_artifact_range_resume',bytes=size,total=total,offset=offset)
 assert size==total
 p=A/'weights/mvsanywhere_hero.ckpt';assert not p.exists();partial.rename(p);identity=json.loads((A/'DINOv2_source_identity_v1.json').read_text());digest=hashlib.sha256()
 with p.open('rb') as f:
  for block in iter(lambda:f.read(8*1024**2),b''):digest.update(block)
 identity.update(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),Hero_url=url,Hero_bytes=size,Hero_sha256=digest.hexdigest(),one_administrative_range_resume=True,initial_download_failure_preserved=True,no_source_or_weights_executed=True);(A/'artifact_identity_v1.json').write_text(json.dumps(identity,indent=2)+'\n')
try:
 status('waiting_existing_acquisition_and_first_review');resumed=False
 while not (A/'artifact_identity_v1.json').exists() or not (H/'mvs_adapter_review_author_closed_v1.json').exists():
  if time.monotonic()-start>2100:raise TimeoutError('fixed2100s stage preparation budget; continuation required')
  if (A/'artifact_acquisition_status_v1.json').exists():
   a=json.loads((A/'artifact_acquisition_status_v1.json').read_text())
   if a['stage']=='acquisition_failed_preserved' and not (A/'artifact_identity_v1.json').exists():
    if resumed:raise RuntimeError('single administrative resume already used')
    resumed=True;resume_one_partial()
  time.sleep(10)
 gate=json.loads((H/'mvs_adapter_review_author_closed_v1.json').read_text());assert gate['no_unresolved_critical_or_important']
 for script,stage in [('seal_mvs_hero_inputs_v1.py','sealing_inputs_before_first_prediction'),('predict_mvs_hero_temporal_v1.py','predicting_six_once'),('assess_mvs_hero_temporal_v1.py','assessing_after_all_sealed')]:
  status(stage,script=script)
  with (H/('mvs_hero_'+stage+'_v1.log')).open('wb') as f:r=subprocess.run([python,str(H/script)],cwd=H.parent.parent,env=env,stdout=f,stderr=subprocess.STDOUT,timeout=600)
  if r.returncode:raise RuntimeError(script+' exit '+str(r.returncode)+'; no silent scientific retry')
 result=json.loads((H/'mvs_hero_temporal_development_v1/assessment_v1.json').read_text());status('finished_development_only',decision=result['decision'],whole_complete=False,formal_navigation_not_run=True,independent_postresult_evidence_review_pending=True)
except Exception as e:status('failed_preserved',exception=type(e).__name__,reason=str(e),whole_complete=False);raise
