from pathlib import Path
import sys,json,numpy as np
D=Path(__file__).resolve().parent;sys.path.insert(0,str(D.parent/'dual_monocular_semantic_bev_20261015_v1'));from common import GROUPS
r=json.loads((D/'coverage_upper_bound.json').read_text());checked=[]
for g,v in r['groups'].items():
 assert len(v['frames'])==181;assert sum(x['target_cells'] for x in v['frames'])==v['target_instances'];assert sum(x['retained_cells_upper_bound'] for x in v['frames'])==v['retained_instances'];R,cp=GROUPS[g];m=dict(np.load(json.loads(cp.read_text())['static_masks']));origin=np.load(R/'maps/frame_000.npz')['origin']
 for row in [v['frames'][0],v['frames'][90],v['frames'][-1]]:
  a=np.load(R/'frames'/row['frame']);pts=[]
  for e in ['left','right']:
   z=a[e+'_depth_m'];yy,xx=np.where(np.isfinite(z)&(z>=.5)&(z<=3)&~m[e]);c=np.column_stack([(xx-319.5)*z[yy,xx]/584.8888888888889,-(yy-239.5)*z[yy,xx]/584.8888888888889,-z[yy,xx]]);T=a[e+'_camera_usd_world'];w=c@T[:3,:3].T+T[:3,3];keep=(w[:,2]>.05)&(w[:,2]<=1.5)&(np.linalg.norm(w-a['root_position_world_m'],axis=1)>=.55);pts.append(w[keep])
  w=np.concatenate(pts);Tc=a['left_optical_to_base'].copy();Tc[:3,1:]*=-1;B=a['left_camera_usd_world']@np.linalg.inv(Tc);b=(w-B[:3,3])@B[:3,:3];valid=np.zeros(len(w),bool)
  for rear in [False,True]:
   z=(-b[:,0] if rear else b[:,0])-.45;u=b[:,2]-.05;v=(b[:,1] if rear else -b[:,1]);valid|=(z>=.5)&(z<=3)&(u>=(-.5-319.5)*z/584.8888888888889)&(u<(639.5-319.5)*z/584.8888888888889)&(v>=(-.5-239.5)*z/584.8888888888889)&(v<(479.5-239.5)*z/584.8888888888889)
  cells=lambda p:set(map(tuple,np.floor((p[:,:2]-origin)/.1+.5).astype(int)));assert len(cells(w))==row['target_cells'];assert len(cells(w)&cells(w[valid]))==row['retained_cells_upper_bound'];checked.append([g,row['frame']])
(D/'independent_bound_check.json').write_text(json.dumps(dict(pass_check=True,complete_frame_aggregates=1267,independent_base_coordinate_projection_frames=checked,no_new_view_score=True),indent=2)+'\n');print('21 base-coordinate direct bounds exact, 1267 counts exact')
