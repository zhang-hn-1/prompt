from pathlib import Path
import json,hashlib,datetime
import numpy as np,cv2
from scipy.spatial import cKDTree
from partial_cue_source_v2 import PartialCueSetSource
H=Path(__file__).resolve().parent;D=H/'offset_replication_D4_run_v1';O=H/'offset_missing_support_audit_v1';O.mkdir(exist_ok=False);sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
man=json.loads((D/'artifact_manifest_post_close.json').read_text())['artifact_sha256'];assert all(sha(f)==s for f,s in man.items())
source=PartialCueSetSource(.2);records=json.loads((D/'source_opportunity_records.json').read_text());fields=0
for r in records:
 i=r['frame']
 if i>20:break
 t=r['time_s'];est=dict(np.load(D/'estimated_poses'/f'frame_{i:03d}.npz'));out=source.update(dict(np.load(D/'sensor_packets'/f'source_{i:03d}.npz')),dict(np.load(D/'online_inputs'/f'frame_{i:03d}.npz')),est,dict(np.load(D/'masks'/f'frame_{i:03d}.npz')),r['qualified_seed'],t,i,est['root_position_world_m'][:2],np.array([2.2,.15]),np.array([-2.2,.15]));saved=np.load(D/'possible_sets'/f'frame_{i:03d}.npz')
 for k in out:np.testing.assert_equal(out[k],saved[k]);fields+=1
# Region selection is an offline truth-derived diagnostic, never an actor feature.
diag=json.loads((H/'offset_source_coverage_diagnosis_v1/assessment.json').read_text());missing=np.asarray(diag['metrics']['factual']['missing_xy']);allw=source.vox;sel=(cKDTree(missing).query(allw[:,:2])[0]<=np.sqrt(2)*.05+1e-9)&(allw[:,2]<=1.0);w=allw[sel];assert len(w)>0
positive=np.zeros((len(w),2),int);exposures=np.zeros((len(w),len(source.views)),bool);evidence=[]
for j,v in enumerate(source.views):
 for e,eye in enumerate(['left','right']):
  cl=source.class_masks[(v['frame'],eye)];T=v['poses'][eye];local=(w-T[:3,3])@(T[:3,:3]@np.diag([1,-1,-1]))
  with np.errstate(divide='ignore',invalid='ignore'):uv=local[:,:2]/local[:,2,None]*584.8888888888889+[319.5,239.5]
  valid=np.isfinite(uv).all(1)&(local[:,2]>=.5)&(local[:,2]<=3)&(uv[:,0]>=3)&(uv[:,0]<636)&(uv[:,1]>=3)&(uv[:,1]<476)&(np.linalg.norm(w[:,:2]-est['root_position_world_m'][:2],axis=1)>.55)
  blocked=cv2.dilate(v['masks'][eye].astype(np.uint8),np.ones((7,7),np.uint8)).astype(bool);ids=np.flatnonzero(valid);x,y=np.rint(uv[ids]).astype(int).T;bad=blocked[y,x];usable=ids[~bad];x,y=np.rint(uv[usable]).astype(int).T;chosen=usable[cl[y,x]];positive[chosen,e]+=1;exposures[chosen,j]=True
  evidence.append(dict(frame=v['frame'],eye=eye,total=len(w),invalid_projection=int((~valid).sum()),CAD_dilated_blocked=int(bad.sum()),usable_RGB_class_negative=int(len(usable)-len(chosen)),positive=len(chosen)))
keep=(positive>0).all(1)&(exposures.sum(1)>=2)&(np.linalg.norm(w[:,:2]-source.start,axis=1)>.7);raw=out['raw_world_voxel_centres'];assert not keep.any() or all(cKDTree(raw).query(w[keep])[0]<1e-9)
np.savez_compressed(O/'local_voxel_support.npz',xyz=w,positive_counts_per_eye=positive,distinct_exposure_support=exposures,original_keep=keep,offline_missing_xy=missing)
r=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),factual_fields_exact=fields,diagnostic_voxels=len(w),diagnostic_columns=len(np.unique(w[:,:2],axis=0)),both_eye_support=int((positive>0).all(1).sum()),no_eye_support=int((positive==0).all(1).sum()),one_eye_only=int(((positive>0).sum(1)==1).sum()),fewer_than_two_distinct_exposures=int((exposures.sum(1)<2).sum()),original_kept=int(keep.sum()),view_breakdown=evidence,no_actor_geometry_modified=True,historical_not_fresh=True,missing_GT_selection_offline_only=True,gate_and_radius_unchanged=True,source_sha256={str(Path(__file__)):sha(__file__),str(H/'partial_cue_source_v2.py'):sha(H/'partial_cue_source_v2.py')},output_sha256=sha(O/'local_voxel_support.npz'),whole_research_complete=False)
(O/'assessment.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
