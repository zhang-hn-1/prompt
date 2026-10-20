"""Bounded current-pair support measurements only; no source filtering or matcher search."""
from pathlib import Path
import json,hashlib,numpy as np,cv2
from stereo_geometry import restore_native

def measure(left,right,q,d):
 dx,dy=np.meshgrid(np.arange(-3,4),np.arange(-3,4));p=np.column_stack([dx.ravel(),dy.ravel()]);c=np.asarray(q,float)[:,None,:]+p[None];r=c.copy();r[:,:,1]-=np.asarray(d)[:,None]
 def sample(rgb,coords):return cv2.remap(cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY).astype(np.float32),coords[:,:,0].astype(np.float32),coords[:,:,1].astype(np.float32),cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT,borderValue=0)
 if not len(q):return {k:np.zeros(0) for k in ['left_std','right_std','ncc','patch_inside']}
 a,b=sample(left,c),sample(right,r);aa=a-a.mean(1,keepdims=True);bb=b-b.mean(1,keepdims=True);ncc=(aa*bb).sum(1)/np.maximum(np.linalg.norm(aa,axis=1)*np.linalg.norm(bb,axis=1),1e-12);inside=((c[:,:,0]>=0)&(c[:,:,0]<=639)&(c[:,:,1]>=0)&(c[:,:,1]<=479)&(r[:,:,0]>=0)&(r[:,:,0]<=639)&(r[:,:,1]>=0)&(r[:,:,1]<=479)).all(1)
 return dict(left_std=a.std(1),right_std=b.std(1),ncc=ncc,patch_inside=inside)

def main():
 H=Path(__file__).resolve().parent;D=H/'live_generic_height_navigation_run_v1';O=H/'generic_height_binocular_support_v1';O.mkdir(exist_ok=False);(O/'features').mkdir();sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();inputs={};rows=json.loads((D/'source_rows.json').read_text())
 for row in rows:
  i=row['frame'];t=row['time_s'];f=f'frame_{i:03d}.npz';files=[D/'partial_observations'/f,D/'source_inputs'/f,D/'source_predictions'/f]
  for p in files:inputs[str(p)]=sha(p)
  a=np.load(files[0]);rgb=np.load(files[1]);pred=np.load(files[2]);assert float(a['acquisition_time_s'])==float(rgb['time_s'])==t;ids=a['query_pixel_id'];x,y=ids%640,ids//640;d=restore_native(pred['disparity_px'])[y,x];v=measure(rgb['left_rgb'],rgb['right_rgb'],np.column_stack([x,y]),d);np.savez_compressed(O/'features'/f,**v,query_pixel_id=ids,time_s=t)
 (O/'features_seal_before_group_labels.json').write_text(json.dumps(dict(inputs=inputs,outputs={str(p):sha(p) for p in (O/'features').glob('*.npz')},source_sha256=sha(__file__),GT_only_group_labels_not_yet_read_by_this_processor=True,old_Glabels_already_used_not_fresh=True,no_filter_or_network_or_control=True,window7_std2_NCC0p9_are_frozen_old_diagnostic_cuts_not_new_source_gate=True),indent=2)+'\n')
 groups={k:[] for k in ['valid_GT','invalid_GT']}
 for row in rows:
  f=f'frame_{row["frame"]:03d}.npz';a=np.load(O/'features'/f);g=np.load(D/'frames'/f);z=g['left_depth_m'].ravel()[a['query_pixel_id']];valid=np.isfinite(z)&(z>0)
  for name,mask in [('valid_GT',valid),('invalid_GT',~valid)]:
   groups[name].append(np.column_stack([a['left_std'][mask],a['right_std'][mask],a['ncc'][mask],a['patch_inside'][mask]]))
 result={}
 for name,items in groups.items():
  a=np.concatenate(items);tex=(a[:,0]>=2)&(a[:,1]>=2)&(a[:,3]>0);result[name]=dict(n=len(a),both_std2_and_inside=int(tex.sum()),texture_support_fraction=float(tex.mean()),ncc_p10_p50_p90=np.percentile(a[:,2],[10,50,90]).tolist(),both_texture_ncc0p9=int((tex&(a[:,2]>=.9)).sum()))
 out=dict(groups=result,only_current_RGB_and_given_disparity_for_features=True,truth_for_offline_groups_only=True,not_new_matcher_or_disparity_refinement=True,no_point_filter_or_actor_change=True,no_gate_rescue=True,not_causal_navigation_benefit=True);(O/'assessment.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out,indent=2))
if __name__=='__main__':main()
