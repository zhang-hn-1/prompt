from pathlib import Path
import json,datetime,time,subprocess,os
H=Path(__file__).resolve().parent;STATUS=H/'mvs_hero_native_api_recovery_status_v2.json';assert not STATUS.exists();old=json.loads((H/'mvs_hero_development_stage_status_v1.json').read_text());assert old['stage']=='failed_preserved' and old['reason'].startswith('predict_mvs_hero_temporal_v1.py exit 1');assert not (H/'mvs_hero_temporal_development_v1/records.json').exists();assert (H/'mvsanywhere_source_preflight_v1/artifact_identity_v1.json').exists();start=time.monotonic();env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',MPLCONFIGDIR='/tmp/mpl-ball-navigation',PYTHONUNBUFFERED='1');python='/home/an/miniconda3/envs/ball_wm/bin/python'
def status(stage,**kw):STATUS.write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),pid=os.getpid(),stage=stage,elapsed_s=time.monotonic()-start,one_administrative_official_API_recovery=True,old_failure_preserved=True,**kw),indent=2)+'\n')
try:
 for script,stage in [('seal_mvs_hero_inputs_v2.py','sealing_recovery_inputs_before_first_prediction'),('predict_mvs_hero_temporal_v2.py','predicting_six_once_native_API'),('assess_mvs_hero_temporal_v2.py','assessing_after_all_sealed')]:
  status(stage,script=script)
  with (H/('mvs_hero_'+stage+'_v2.log')).open('wb') as f:r=subprocess.run([python,str(H/script)],cwd=H.parent.parent,env=env,stdout=f,stderr=subprocess.STDOUT,timeout=600)
  if r.returncode:raise RuntimeError(script+' exit '+str(r.returncode)+'; no further retry')
 result=json.loads((H/'mvs_hero_temporal_development_v2_api_scale_recovery/assessment_v1.json').read_text());status('finished_development_only',decision=result['decision'],whole_complete=False,formal_navigation_not_run=True,postresult_validation_pending=True)
except Exception as e:status('failed_preserved_no_further_retry',exception=type(e).__name__,reason=str(e),whole_complete=False);raise
