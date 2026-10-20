"""Offline-only plane-hypothesis labels; all predictions precede GT assessment."""
from pathlib import Path
import json,hashlib,datetime
import numpy as np
from multiview_rgb import optical_rotation
from past_multiview import measure_view
H=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
 p=json.loads((H/'floor_hypothesis_protocol.json').read_text());m=json.loads((H/'floor_hypothesis_predictions/manifest.json').read_text());started=datetime.datetime.now(datetime.timezone.utc).isoformat();assert m['all_four_saved_before_labels'] and all(r['ended_utc']<started for r in m['rows'])
 for field in ['source_sha256','prediction_input_sha256','producer_sha256','evaluation_input_sha256']:
  for path,value in p[field].items():assert sha(path)==value,path
 masks=np.load(p['mask_path']);yy,xx=np.mgrid[:480,:640];par=p['parameters'];rays=np.stack([(xx-par['cx'])/par['focal'],(yy-par['cy'])/par['focal'],np.ones((480,640))],axis=-1);norm=np.linalg.norm(rays,axis=-1);rows=[];arrays={}
 for r in m['rows']:
  i=r['frame'];truth=np.load(H/'rolling_online_run/offline_truth'/f'frame_{i:03d}.npz');pred=np.load(r['prediction']);assert sha(r['prediction'])==r['sha256'];pose=truth['left_camera_usd_world'];z=truth['left_depth_m'];points=pose[:3,3]+(rays*z[...,None])@optical_rotation(pose).T
  finite=np.isfinite(z)&(z>0)&~masks['left'];floor=finite&(abs(points[...,2])<=.05);queries=floor&(z>=.5)&(z<=3);outputs=pred['confidence'];error=abs(pred['depth_m']-z)*norm;correct=outputs&floor&(error<=.1);n=int(queries.sum());outn=int(outputs.sum());err=error[outputs&np.isfinite(error)];precision=float(correct.sum()/outn) if outn else None;fraction=float((correct&queries).sum()/n) if n else None;p90=float(np.percentile(err,90)) if len(err) else None
  sourcecounts=np.zeros((480,640),np.uint8);qy,qx=np.where(queries);pts=points[qy,qx]
  for s in r['sources']:
   st=np.load(H/'rolling_online_run/offline_truth'/f"frame_{s['frame']:03d}.npz");eye=s['eye'];v=measure_view(pts,st[eye+'_camera_usd_world'],st[eye+'_depth_m'],masks[eye],par['focal'],par['cx'],par['cy'],.01);sourcecounts[qy,qx]+=v['visible'].astype(np.uint8)
  row=dict(frame=i,time_s=r['time_s'],potential_floor_queries=n,finite_unmasked_pixels=int(finite.sum()),all_visible_floor_pixels=int(floor.sum()),floor_outside_half_to_three_m=int((floor&~queries).sum()),outputs=outn,correct_outputs=int(correct.sum()),floor_all_queries_correct_fraction=fraction,all_outputs_ground_precision=precision,output_ray_error_p90_m=p90,plane_geom_pixels=int(np.isfinite(pred['plane_axis_depth_m']).sum()),photometric_support_histogram={str(k):int((pred['support_count']==k).sum()) for k in range(4)},potential_floor_with_two_true_visible_sources=int((queries&(sourcecounts>=2)).sum()),elapsed_cpu_wall_s=r['elapsed_cpu_wall_s'])
  row['gates']=dict(query_count=n>=100,floor_all_correct=fraction is not None and fraction>=.9,precision=precision is not None and precision>=.99,ray_error=p90 is not None and p90<=.1);row['accepted']=all(row['gates'].values());rows.append(row)
  for key,value in dict(floor_query=queries,all_visible_floor=floor,output=outputs,correct=correct,source_visible_count=sourcecounts).items():arrays[f'frame_{i:03d}__'+key]=value
  print('FLOOR_ASSESSMENT',i,n,outn,row['potential_floor_with_two_true_visible_sources'],row['accepted'],flush=True)
 result=dict(started_utc=started,ended_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),rows=rows,candidate_accepted=all(r['accepted'] for r in rows),new_physics=0,new_fits=0,online_map_writes=0,prior_as_observation=False,role=p['role'],old_gates_changed=False,source_sha256={str(H/'audit_floor_hypothesis.py'):sha(H/'audit_floor_hypothesis.py')},whole_research_complete=False)
 np.savez_compressed(H/'floor_hypothesis_measurements.npz',**arrays);(H/'floor_hypothesis_audit.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
if __name__=='__main__':main()
