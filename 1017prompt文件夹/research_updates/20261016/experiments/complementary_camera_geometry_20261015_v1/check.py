from pathlib import Path
import sys,json,numpy as np
from scipy.spatial.transform import Rotation
D=Path(__file__).resolve().parent;sys.path.insert(0,str(D.parent/'dual_monocular_semantic_bev_20261015_v1'))
from common import GROUPS
r=json.loads((D/'results.json').read_text());checked=[]
for name,g in r['groups'].items():
 assert sum(x['target_cells'] for x in g['frames'])==g['target_cell_instances'];assert sum(x['retained_cells_upper_bound'] for x in g['frames'])==g['retained_cell_instances_upper_bound'];assert len(g['frames'])==181
 root,cp=GROUPS[name];m=dict(np.load(json.loads(cp.read_text())['static_masks']));origin=np.load(root/'maps/frame_000.npz')['origin']
 for row in [g['frames'][0],g['frames'][90],g['frames'][-1]]:
  a=np.load(root/'frames'/row['frame']);q=a['root_quaternion_world_wxyz'];base=Rotation.from_quat([*q[1:],q[0]]).as_matrix();allpts=[]
  for e in ['left','right']:
   z=a[e+'_depth_m'];yy,xx=np.where(np.isfinite(z)&(z>=.5)&(z<=3)&~m[e]);c=np.column_stack([(xx-319.5)*z[yy,xx]/584.8888888888889,-(yy-239.5)*z[yy,xx]/584.8888888888889,-z[yy,xx]]);T=a[e+'_camera_usd_world'];w=c@T[:3,:3].T+T[:3,3];keep=(w[:,2]>.05)&(w[:,2]<=1.5)&(np.linalg.norm(w-a['root_position_world_m'],axis=1)>=.55);allpts.append(w[keep])
  w=np.concatenate(allpts);vis=np.zeros(len(w),bool)
  for e,degrees in [('left',20),('right',-20)]:
   T=a[e+'_camera_usd_world'];rot=base@Rotation.from_euler('z',degrees,degrees=True).as_matrix()@base.T@T[:3,:3];c=(w-T[:3,3])@rot;c[:,1:]*=-1;z=c[:,2];vis|=(z>=.5)&(z<=3)&(c[:,0]>=(-.5-319.5)*z/584.8888888888889)&(c[:,0]<(639.5-319.5)*z/584.8888888888889)&(c[:,1]>=(-.5-239.5)*z/584.8888888888889)&(c[:,1]<(479.5-239.5)*z/584.8888888888889)
  cell=lambda p:set(map(tuple,np.floor((p[:,:2]-origin)/.1+.5).astype(int)));assert len(cell(w))==row['target_cells'];assert len(cell(w)&cell(w[vis]))==row['retained_cells_upper_bound'];checked.append([name,row['frame']])
(D/'independent_check.json').write_text(json.dumps(dict(complete_frame_aggregates=1267,independent_projection_frames=checked,pass_check=True,not_independent_new_scene_render=True),indent=2)+'\n');print('21 independent frame projections exact; 1267 aggregate counts exact')
