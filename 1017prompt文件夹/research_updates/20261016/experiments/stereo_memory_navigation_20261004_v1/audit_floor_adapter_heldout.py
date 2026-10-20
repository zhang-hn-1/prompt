"""Full independent fresh-sequence labels only after all130predictions saved."""
from pathlib import Path
import json,datetime,hashlib
import numpy as np
from multiview_rgb import optical_rotation
H=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
 p=json.loads((H/'floor_adapter_fit_protocol.json').read_text());fit=json.loads((H/'floor_adapter_fit.json').read_text());seal=json.loads((H/'semantic_adapter_heldout_seal.json').read_text());m=json.loads((H/'floor_adapter_heldout_predictions/manifest.json').read_text());started=datetime.datetime.now(datetime.timezone.utc).isoformat()
 expected={(i,e) for i in range(0,321,5) for e in ['left','right']};actual=[(r['frame'],r['eye']) for r in m['rows']];assert len(actual)==130 and len(set(actual))==130 and set(actual)==expected,'missing/duplicate/empty predictions cannot pass'
 assert m['all_130_saved_before_labels'] and all(r['ended_utc']<started for r in m['rows']);assert fit['optimizer_converged'];assert sha(H/'semantic_adapter_heldout_seal.json')==p['heldout_seal_sha256']
 for file,value in seal['original_artifact_sha256'].items():assert sha(file)==value,file
 for file,value in p['source_sha256'].items():assert sha(file)==value,file
 masks=np.load(p['mask_path']);yy,xx=np.mgrid[:480,:640];par=p['projection'];rays=np.stack([(xx-par['cx'])/par['focal'],(yy-par['cy'])/par['focal'],np.ones((480,640))],axis=-1);norm=np.linalg.norm(rays,axis=-1);rows=[];errors=[];npixels=correctcount=totalqueries=correctqueries=0
 for r in m['rows']:
  i=r['frame'];eye=r['eye'];truth=np.load(H/'semantic_adapter_heldout_run/frames'/f'frame_{i:03d}.npz');pred=np.load(r['prediction']);assert sha(r['prediction'])==r['sha256'];depth=truth[eye+'_depth_m'];pose=truth[eye+'_camera_usd_world']
  with np.errstate(invalid='ignore'):pts=pose[:3,3]+(rays*depth[...,None])@optical_rotation(pose).T
  floor=np.isfinite(depth)&(depth>0)&~masks[eye]&(abs(pts[...,2])<=.05);query=floor&(depth>=.5)&(depth<=3);outputs=pred['confidence'];error=abs(pred['depth_m']-depth)*norm;correct=outputs&floor&(error<=.1);n=int(query.sum());outn=int(outputs.sum());cn=int(correct.sum());cq=int((correct&query).sum());e=error[outputs&np.isfinite(error)];precision=float(cn/outn) if outn else None;recall=float(cq/n) if n else None;p90=float(np.percentile(e,90)) if len(e) else None;raw=pred['semantic_floor_native']&~masks[eye];raw_n=int(raw.sum())
  gates=dict(count=n>=100,all_floor=recall is not None and recall>=.9,precision=precision is not None and precision>=.99,error=p90 is not None and p90<=.1)
  row=dict(frame=i,eye=eye,time_s=r['time_s'],queries=n,outputs=outn,correct_outputs=cn,correct_queries=cq,all_output_floor_precision=precision,all_query_correct_fraction=recall,ray_error_p90_m=p90,raw_unmasked_floor_pixels=raw_n,raw_floor_height_band_precision=float((raw&floor).sum()/raw_n) if raw_n else None,data_insufficient=n<100,gates=gates,accepted=all(gates.values()),elapsed_cpu_wall_s=r['elapsed_cpu_wall_s']);rows.append(row);npixels+=outn;correctcount+=cn;totalqueries+=n;correctqueries+=cq;errors.append(e)
  if i%30==0:print('FRESH_HELDOUT_LABELS',i,eye,n,outn,precision,recall,row['accepted'],flush=True)
 e=np.concatenate(errors);summary=dict(total_queries=totalqueries,total_outputs=npixels,correct_outputs=correctcount,correct_queries=correctqueries,output_precision=correctcount/npixels if npixels else None,all_query_correct_fraction=correctqueries/totalqueries if totalqueries else None,ray_error_p90_m=float(np.percentile(e,90)) if len(e) else None,accepted_image_samples=sum(r['accepted'] for r in rows),image_samples=len(rows),data_insufficient_samples=sum(r['data_insufficient'] for r in rows))
 result=dict(started_utc=started,ended_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),rows=rows,summary=summary,candidate_accepted=all(r['accepted'] for r in rows),fresh_whole_sequences=1,adjacent_random_train_test_split=False,fit_count=1,backbone_updated=False,estimated_pose_layer_pass=False,online_map_or_navigation_admitted=False,formal_e0e1e2_run=False,whole_research_complete=False)
 (H/'floor_adapter_heldout_audit.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n');print('FRESH_WHOLE_SEQUENCE_GATE',result['candidate_accepted'],summary,flush=True)
if __name__=='__main__':main()
