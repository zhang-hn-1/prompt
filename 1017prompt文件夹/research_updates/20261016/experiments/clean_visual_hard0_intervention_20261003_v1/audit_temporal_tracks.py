"""Whole-sequence past-only replay and independent depth truth error audit."""
import json, argparse
from pathlib import Path
import numpy as np,cv2
from temporal_tracks import TemporalTracks,FOCAL,CX,CY
H=Path(__file__).parent;E=H.parent
runs={**{s:E/'fixed_rig_self_mask_and_curve_qualification_20261003_v1/legacy_clean'/s for s in ('e0','e1','e2')},**{s:H/'baseline'/s for s in ('hard_0_e1_0003','hard_0_e1_0004')}}
masks=np.load(E/'fixed_rig_self_mask_and_curve_qualification_20261003_v1/base_link_masks.npz')
parser=argparse.ArgumentParser();parser.add_argument('--synchronized',action='store_true');parser.add_argument('--only-h3',action='store_true');args=parser.parse_args()
if args.synchronized:
 runs={**{s:H/'synchronized_v2'/s for s in ('e0','e1','e2','hard_0_e1_0004')},'hard_0_e1_0003':H/'synchronized_v2/hard_0_e1_0003'}
if args.only_h3:runs={k:v for k,v in runs.items() if k=='hard_0_e1_0003'}
VERSION='synchronized' if args.synchronized else 'original'
summary={};cv2.setNumThreads(2)
for name,root in runs.items():
 model=TemporalTracks(masks);records=[];errors=[];obs_errors=[];root_errors=[];consecutive=0;trigger=None;output=H/('tracks_replay_'+VERSION)/name;output.mkdir(parents=True,exist_ok=True)
 for i,path in enumerate(sorted((root/'online_inputs').glob('frame_*.npz'))):
  with np.load(path) as f,np.load(root/'offline_truth'/path.name) as truth:
   xy=f['position_world_xy'];yaw=float(f['yaw_world_rad']);q=f['quaternion_world_wxyz_history'][-1];current=model.update(f['left_rgb'],f['right_rgb'],xy,q,i*.1)
   front=[];per_eye={};save={};root_errors.append(float(truth['root_position_world_m'][2]-.4))
   for eye,data in current.items():
    uv=data['uv'];points=data['points_world'];pose=truth[eye+'_camera_usd_world'];rotation=np.column_stack((pose[:3,0],-pose[:3,1],-pose[:3,2]))
    if len(uv):
     depth=cv2.remap(truth[eye+'_depth_m'].astype(np.float32),uv[:,0].astype(np.float32)[None],uv[:,1].astype(np.float32)[None],cv2.INTER_NEAREST)[0]
     rays=np.column_stack(((uv[:,0]-CX)/FOCAL,(uv[:,1]-CY)/FOCAL,np.ones(len(uv))))
     valid_depth=np.isfinite(depth)&(depth>0)
     actual=pose[:3,3]+(rays*np.where(valid_depth,depth,0)[:,None])@rotation.T;error=np.linalg.norm(points-actual,axis=1);error[~valid_depth]=np.inf
    else:error=np.empty(0)
    delta=points[:,:2]-xy;c,s=np.cos(yaw),np.sin(yaw);body=delta@np.array([[c,-s],[s,c]])
    obstacle=(points[:,2]>=.25)&(points[:,2]<=1.5)&(np.linalg.norm(points-np.r_[xy,.4],axis=1)>=.55)
    forward=obstacle&(body[:,0]>=.55)&(body[:,0]<=3.5)&(np.abs(body[:,1])<=.6)
    front.append(body[forward]);errors.extend(error.tolist());obs_errors.extend(error[obstacle].tolist());save.update({eye+'_'+key:val for key,val in data.items()});save[eye+'_truth_error_m']=error
    per_eye[eye]={'accepted':len(points),'obstacle':int(obstacle.sum()),'front':int(forward.sum()),'corners':data['corners'],'past_views':data['past_views']}
   cells=np.unique(np.floor(np.concatenate(front)/.1).astype(int),axis=0) if any(len(x) for x in front) else np.empty((0,2),int)
   consecutive=consecutive+1 if len(cells)>=6 else 0
   if consecutive>=2 and trigger is None:trigger=i*.1
   records.append({'time_s':i*.1,'eyes':per_eye,'front_cells':len(cells),'hypothetical_trigger':consecutive>=2})
   np.savez_compressed(output/path.name,**save)
 def arrays(v):
  a=np.asarray(v);finite=a[np.isfinite(a)]
  return {'n':len(v),'invalid_truth_depth':int((~np.isfinite(a)).sum()),'finite_p50_m':float(np.median(finite)) if len(finite) else None,'finite_p90_m':float(np.percentile(finite,90)) if len(finite) else None,'within_0p3m':float(np.mean(a<=.3)) if len(a) else None}
 summary[name]={'frames':len(records),'all_geometry':arrays(errors),'bodyband_geometry':arrays(obs_errors),'first_front_event_s':trigger,'max_front_cells':max(r['front_cells'] for r in records),'root_height_error_range_m':[min(root_errors),max(root_errors)],'records':records}
 (H/('temporal_tracks_audit_'+VERSION+('_h3' if args.only_h3 else '')+'.json')).write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
 print(name,{k:v for k,v in summary[name].items() if k!='records'},flush=True)
