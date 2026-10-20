"""Birth-only hypotheses from causal sealed RGB/estimated poses; no truth inputs."""
from pathlib import Path
import json,hashlib,datetime
import numpy as np,cv2
from occluded_eye_partial_source import OccludedEyePartialSource
from occluded_eye_support import combine_support
from hypothesis_features_v1 import build_features
H=Path(__file__).resolve().parent;O=H/'hypothesis_learning_features_v1';sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def build_case(case,y):
 D=H/case;seal=json.loads((D/'artifact_manifest_post_close.json').read_text())['artifact_sha256'];assert all(sha(p)==s for p,s in seal.items());source=OccludedEyePartialSource(.2);inputs={}
 for row in json.loads((D/'source_opportunity_records.json').read_text()):
  i=row['frame'];files=[D/'sensor_packets'/f'source_{i:03d}.npz',D/'online_inputs'/f'frame_{i:03d}.npz',D/'estimated_poses'/f'frame_{i:03d}.npz',D/'masks'/f'frame_{i:03d}.npz'];inputs.update({str(f):sha(f) for f in files});packet,rgb,poses,masks=[dict(np.load(f)) for f in files];root=poses['root_position_world_m'][:2]
  result=source.update(packet,rgb,poses,masks,row['qualified_seed'],row['time_s'],i,root,np.array([2.2,y]),np.array([-2.2,y]))
  if source.model is not None:break
 else:raise ValueError('no qualified seed/birth')
 w=source.vox;n=len(w);nv=len(source.views);p=np.zeros((n,nv,2),bool);u=np.ones_like(p);depth=np.zeros(p.shape);centres=np.empty((nv,2,3))
 for j,v in enumerate(source.views):
  assert v['time_s']<=row['time_s']
  for e,eye in enumerate(['left','right']):
   T=v['poses'][eye];centres[j,e]=T[:3,3];local=(w-T[:3,3])@(T[:3,:3]@np.diag([1,-1,-1]));depth[:,j,e]=local[:,2]
   with np.errstate(divide='ignore',invalid='ignore'):uv=local[:,:2]/local[:,2,None]*584.8888888888889+[319.5,239.5]
   valid=np.isfinite(uv).all(1)&(local[:,2]>=.5)&(local[:,2]<=3)&(uv[:,0]>=3)&(uv[:,0]<636)&(uv[:,1]>=3)&(uv[:,1]<476)&(np.linalg.norm(w[:,:2]-root,axis=1)>.55)
   ids=np.flatnonzero(valid);x,y=np.rint(uv[ids]).astype(int).T;blocked=cv2.dilate(v['masks'][eye].astype(np.uint8),np.ones((7,7),np.uint8)).astype(bool);ids=ids[~blocked[y,x]];x,y=np.rint(uv[ids]).astype(int).T;u[ids,j,e]=False;p[ids,j,e]=source.class_masks[(v['frame'],eye)][y,x]
 keep=combine_support(p,u)&(np.linalg.norm(w[:,:2]-source.start,axis=1)>.7);w=w[keep];np.testing.assert_allclose(w,result['raw_world_voxel_centres']);haz=(packet['world'][:,2]>.05)&(packet['world'][:,2]<=1.5)
 x=build_features(w,p[keep],u[keep],depth[keep],centres,packet['world'][haz],packet['sigma_m'][haz],root,row['time_s'],np.array([v['time_s'] for v in source.views]))
 np.savez_compressed(O/(case+'.npz'),features=x,voxels=w,baseline_possible_xy=result['possible_xy'],time_s=row['time_s'],frame=i,root_xy=root,model_hypothesis=True,measured_geometry=False,certified_free=False)
 return dict(case=case,birth_frame=i,rows=len(w),input_sha256=inputs,source_manifest_sha256=sha(D/'artifact_manifest_post_close.json'))
def main():
 status=json.loads((H/'hypothesis_learning_acquisition_status_v1.json').read_text());assert status['stage']=='TWO_WHOLE_SEQUENCES_SEALED_BEFORE_FIT' and not status['fit_started'];O.mkdir(exist_ok=False)
 rows=[build_case(c,y) for c,y in [('flat_ground_pose_D3_run_v1',0.),('offset_replication_D4_run_v1',.15),('hypothesis_learning_H1_run_v1',0.),('hypothesis_learning_H2_run_v1',.15)]]
 (O/'features_pretruth_seal.json').write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),rows=rows,output_sha256={str(f):sha(f) for f in O.glob('*.npz')},code_sha256={str(H/f):sha(H/f) for f in ['build_hypothesis_learning_features_v1.py','hypothesis_features_v1.py','occluded_eye_partial_source.py','occluded_eye_support.py','partial_cue_source_v2.py']},no_GT_labels_or_future=True,held_whole_sequences_before_fit=True),indent=2)+'\n');print([(r['case'],r['rows']) for r in rows])
if __name__=='__main__':main()
