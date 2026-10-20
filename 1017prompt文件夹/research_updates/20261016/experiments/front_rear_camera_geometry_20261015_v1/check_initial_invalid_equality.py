from pathlib import Path
import sys,json,numpy as np
D=Path(__file__).resolve().parent;sys.path.insert(0,str(D.parent/'stereo_memory_navigation_20261004_v1'));from dynamic_self_mask import RobotSelfMask
r=json.loads((D/'cad_qualification.json').read_text());m=RobotSelfMask();checked=[]
for g in ['H','E0','E1','B54','C54','B65','C65']:
 rows=[a for a in r['rows'] if a['group']==g];assert len(rows)==181
 for a in [rows[0],rows[90],rows[-1]]:
  T=m.body_transforms(a['joints']);x=[]
  for k,v in m.triangles.items():
   p=np.einsum('...j,ij->...i',v,T[k][:3,:3])+T[k][:3,3];x.extend([float(p[:,:,0].min()),float(p[:,:,0].max())])
  front=max(x)-.45;rear=-min(x)-.45;assert abs(front-a['max_front_optical_z'])<1e-12 and abs(rear-a['max_rear_optical_z'])<1e-12;checked.append([g,a['frame']])
(D/'independent_CAD_check.json').write_text(json.dumps(dict(pass_check=True,complete_registered_frames=1267,independent_full_xyz_vertex_transform_frames=checked,max_positive_vertex_distance_from_camera_m=max(a['max_front_optical_z'] for a in r['rows']),renderer_near_clip_m=.01,no_model_or_navigation=True),indent=2)+'\n');print('21 full XYZ CAD transforms exact')
