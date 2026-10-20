"""Independent zero-shot floor labels after all8predictions serialized."""
from pathlib import Path
import datetime,json,hashlib
import numpy as np
from multiview_rgb import optical_rotation
H=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
 p=json.loads((H/'semantic_floor_protocol.json').read_text());m=json.loads((H/'semantic_floor_predictions/manifest.json').read_text());started=datetime.datetime.now(datetime.timezone.utc).isoformat();assert m['all_eight_saved_before_truth'] and all(r['ended_utc']<started for r in m['rows'])
 for field in ['source_sha256','prediction_input_sha256','evaluation_input_sha256','checkpoint_sha256']:
  for f,v in p[field].items():assert sha(f)==v,f
 masks=np.load(p['mask_path']);yy,xx=np.mgrid[:480,:640];par=p['projection'];rays=np.stack([(xx-par['cx'])/par['focal'],(yy-par['cy'])/par['focal'],np.ones((480,640))],axis=-1);norm=np.linalg.norm(rays,axis=-1);rows=[];arrays={};labels=json.loads((H/'semantic_floor_model/config.json').read_text())['id2label']
 for r in m['rows']:
  i=r['frame'];eye=r['eye'];truth=np.load(H/'rolling_online_run/offline_truth'/f'frame_{i:03d}.npz');pred=np.load(r['prediction']);assert sha(r['prediction'])==r['sha256'];z=truth[eye+'_depth_m'];pose=truth[eye+'_camera_usd_world']
  with np.errstate(invalid='ignore'):points=pose[:3,3]+(rays*z[...,None])@optical_rotation(pose).T
  finite=np.isfinite(z)&(z>0)&~masks[eye];floor=finite&(abs(points[...,2])<=.05);query=floor&(z>=.5)&(z<=3);out=pred['confidence'];err=abs(pred['depth_m']-z)*norm;correct=out&floor&(err<=.1);n=int(query.sum());nout=int(out.sum());raw=pred['semantic_floor_native']&~masks[eye];rawnum=int(raw.sum());e=err[out&np.isfinite(err)];precision=float(correct.sum()/nout) if nout else None;recall=float((correct&query).sum()/n) if n else None;p90=float(np.percentile(e,90)) if len(e) else None
  qlabels=pred['semantic_label_native'][query];counts={labels[str(k)]:int((qlabels==k).sum()) for k in np.unique(qlabels)};probs=pred['floor_probability_native'][query]
  row=dict(frame=i,eye=eye,time_s=r['time_s'],floor_queries=n,projected_outputs=nout,correct_outputs=int(correct.sum()),floor_correct_fraction_all=recall,all_output_floor_precision=precision,ray_error_p90_m=p90,raw_unmasked_semantic_floor_outputs=rawnum,raw_semantic_floor_precision=float((raw&floor).sum()/rawnum) if rawnum else None,true_floor_argmax_labels=counts,true_floor_probability_median=float(np.median(probs)) if len(probs) else None,true_floor_probability_max=float(probs.max()) if len(probs) else None,elapsed_cpu_wall_s=r['elapsed_cpu_wall_s'])
  row['gates']=dict(count=n>=100,all_floor=recall is not None and recall>=.9,precision=precision is not None and precision>=.99,error=p90 is not None and p90<=.1);row['accepted']=all(row['gates'].values());rows.append(row)
  for key,value in dict(floor_query=query,all_visible_floor=floor,output=out,correct=correct).items():arrays[f'frame_{i:03d}_{eye}__'+key]=value
  print('SEMANTIC_FLOOR_LABELS',i,eye,n,nout,row['true_floor_argmax_labels'],row['true_floor_probability_max'],flush=True)
 result=dict(started_utc=started,ended_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),rows=rows,candidate_accepted=all(r['accepted'] for r in rows),new_physics=0,new_fits=0,new_models=1,online_map_writes=0,role=p['role'],old_gates_changed=False,assessment_sha256=sha(H/'audit_semantic_floor.py'),whole_research_complete=False)
 np.savez_compressed(H/'semantic_floor_measurements.npz',**arrays);(H/'semantic_floor_audit.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
if __name__=='__main__':main()
