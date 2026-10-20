"""Finite prefix geometry explanation, not an additional planner or future label."""
from pathlib import Path
import json,hashlib
import numpy as np
from scipy.ndimage import distance_transform_edt
from scipy.spatial.transform import Rotation
from supported_four_point_planning import sample_motion_primitive,primitive_is_collision_free
from execution_domain_four_point_source_v2 import supported_requests,supported_curvature
H=Path(__file__).resolve().parent;D=H/'reverse_main_navigation_e1_b0_run_v1'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
 m=np.load(D/'maps/frame_120.npz');f=np.load(D/'frames/frame_120.npz');R=Rotation.from_quat(f['root_quaternion_world_wxyz'][[1,2,3,0]]).as_matrix();root=np.r_[f['root_position_world_m'][:2],np.arctan2(R[1,0],R[0,0])];resolution=float(m['resolution']);margin=.6+.2+resolution*np.sqrt(2)/2;field=distance_transform_edt(np.pad(m['labels']!=1,1))[1:-1,1:-1]*resolution;occupied=(field<margin-1e-8).astype(np.uint8);hazard=np.argwhere(m['labels']==1);world=m['origin']+hazard*resolution;commands=[c for c in supported_requests() if c[0]>.0];checks=[];firsts=[]
 def check(pose,command,prefix):
  s=sample_motion_primitive(start_pose=pose,signed_arc_length_m=.3,curvature_1_m=supported_curvature(*command),sample_spacing_m=.05);cells=np.floor((s[:,:2]-m['origin'])/resolution+.5).astype(int);free=primitive_is_collision_free(s,occupancy_grid=occupied,origin_xy=m['origin'],resolution_m=resolution);bad=np.flatnonzero(occupied[cells[:,0],cells[:,1]]==1);nearest=[]
  for idx in bad:
   rasterworld=m['origin']+cells[idx]*resolution;j=int(np.linalg.norm(world-rasterworld,axis=1).argmin());h=hazard[j];nearest.append(dict(sample_index=int(idx),sample_world_xy=s[idx,:2].tolist(),raster_center_xy=rasterworld.tolist(),edt_clearance_m=float(field[tuple(cells[idx])]),hazard_cell=h.tolist(),hazard_world_xy=world[j].tolist(),hazard_acquisition_time_s=float(m['hazard_acquisition_time'][tuple(h)]),raster_to_hazard_center_m=float(np.linalg.norm(world[j]-rasterworld))))
  checks.append(dict(prefix_aux_rad=prefix+[command[1]],primitive_free=bool(free),prefix_valid_before_current=bool(not prefix or firsts[len(prefix)-1][0]) if False else None,samples=s.tolist(),blocked=nearest));return free,s[-1]
 for c in commands:firsts.append(check(root,c,[]))
 for i,(free,pose) in enumerate(firsts):
  for c in commands:
   check(pose,c,[commands[i][1]]);checks[-1]['prefix_first_level_free']=bool(free);checks[-1]['whole_two_level_free']=bool(free and checks[-1]['primitive_free'])
 assert len(checks)==12
 out=dict(root=root.tolist(),current_time_s=float(f['time_s']),planning_margin_m=margin,checks=checks,first_level_passes=sum(x['primitive_free'] for x in checks[:3]),two_level_passes=sum(x.get('whole_two_level_free',False) for x in checks),original_unreachable7_unchanged=True,source_and_input_sha256={str(p):sha(p) for p in [Path(__file__),H/'current_forward_root_geometry_scope_v1.json',D/'maps/frame_120.npz',D/'frames/frame_120.npz',H/'supported_four_point_planning.py',H/'execution_domain_four_point_source_v2.py',H/'current_boundary_forward_only_assessment_v1.json']},interpretation='Raster sampled nominal prefix only; EDT nearest hazard and acquisition time do not certify source or full 3D physical safety. No solver/actor/model call, no deletion or margin change.',whole_mission_complete=False)
 p=H/'current_forward_root_geometry_assessment_v1.json';assert not p.exists();p.write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(dict(first_level_passes=out['first_level_passes'],two_level_passes=out['two_level_passes'],checks=12,hazard_cells=sorted(set(tuple(b['hazard_cell']) for c in checks for b in c['blocked'])))))
if __name__=='__main__':main()
