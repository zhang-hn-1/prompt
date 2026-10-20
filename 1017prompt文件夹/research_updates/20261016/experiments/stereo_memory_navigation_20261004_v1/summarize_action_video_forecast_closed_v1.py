"""Report one closed forecast experiment, no refit or change of frozen gates."""
from pathlib import Path
import json,hashlib,numpy as np,cv2
from baseline35_metrics_v1 import assess
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
H=Path(__file__).resolve().parent;O=H/'action_video_forecast_fit_validation_v1';c=json.loads((H/'action_conditioned_dual_video_forecast_protocol_v1.json').read_text())
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
 s=json.loads((H/'action_video_forecast_stage_status_v1.json').read_text());assert s['status']=='finished' and s['exit_code']==0;r=json.loads((O/'assessment.json').read_text());m=json.loads((O/'artifact_manifest_post_close.json').read_text());assert all(sha(p)==v for p,v in m['artifact_sha256'].items());acquisition=[];fig,axes=plt.subplots(2,2,figsize=(12,7))
 for k,n in enumerate(c['fresh_whole_sequences']):
  D=H/n;seal=json.loads((D/'artifact_manifest_post_close.json').read_text());assert all(sha(p)==v for p,v in seal['artifact_sha256'].items());req=np.load(D/'requests.npz')['requests'];states=json.loads((D/'states.json').read_text());contacts=json.loads((D/'physics_contacts.json').read_text());ep=json.loads((D/'task_episode.json').read_text())['actual_episode'];metrics=assess(states,[dict(time_s=a['simulation_time_s'],collision=a['contact'],available=a['available']) for a in contacts],ep['start_position_world_m'][:2],ep['goal_position_world_m'][:2]);video={}
  for tag in ('stereo_view','third_person'):
   cap=cv2.VideoCapture(str(D/(tag+'.mp4')));fps=float(cap.get(cv2.CAP_PROP_FPS));count=0
   while True:
    ok,_=cap.read()
    if not ok:break
    count+=1
   cap.release();video[tag]=dict(decoded_frames=count,fps=fps);assert count==len(req)+1
  acquisition.append(dict(sequence=n,requests=len(req),metrics=metrics,videos=video,acquisition_only_not_new_navigation_admission=True))
  f=np.load(O/(n+'_causal_inputs.npz'));p=np.load(O/(n+'_predictions_before_future_teacher.npz'))['prediction'];y=np.load(O/(n+'_future_teacher_after_predictions.npz'));flow=y['flow'];v=y['valid']
  for en,eye in enumerate(('left','right')):
   a=axes[k,en];err=np.linalg.norm(p[:,en]-flow[:,en],axis=-1);base=np.linalg.norm(f['baseline'][:,en]-flow[:,en],axis=-1);mask=v[:,en]&f['moving'][:,None];mean=np.divide(np.where(mask,err,0).sum(1),mask.sum(1),out=np.full(len(err),np.nan),where=mask.sum(1)>0);bmean=np.divide(np.where(mask,base,0).sum(1),mask.sum(1),out=np.full(len(err),np.nan),where=mask.sum(1)>0);a.plot(f['times_s'],mean,label='one learned forecast');a.plot(f['times_s'],bmean,label='past-flow continuation');a.set(title=f'{n} / {eye}',xlabel='current simulation time (s)',ylabel='moving-query mean EPE (px)');a.legend();a.grid(alpha=.3)
 fig.tight_layout();fig.savefig(O/'fresh_forecast_errors_v1.png',dpi=150);plt.close(fig)
 out=dict(original_assessment_sha256=sha(O/'assessment.json'),original_outputs_verified=len(m['artifact_sha256']),fresh_acquisition=acquisition,one_frozen_fit=True,accepted_forecast_layer_only=r['accepted_forecast_layer_only'],fresh_rows=r['fresh'],not_online_shadow=True,not_navigation_worldmodel=True,no_source_safety_or_navigation_admission=True,whole_mission_complete=False)
 (O/'closed_forecast_summary_v1.json').write_text(json.dumps(out,indent=2)+'\n');(O/'artifact_manifest_post_summary_v1.json').write_text(json.dumps(dict(artifact_sha256={str(p):sha(p) for p in O.rglob('*') if p.is_file()}),indent=2)+'\n');print(json.dumps({k:v for k,v in out.items() if k!='fresh_acquisition'},indent=2))
if __name__=='__main__':main()
