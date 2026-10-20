"""One semantic post-reversal rebuild. Offline teacher never consumed by actor."""
from pathlib import Path
import json,hashlib,numpy as np
H=Path(__file__).resolve().parent;D=H/'reverse_main_navigation_e1_candidate_run_v1'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def main():
 p=H/'reverse_main_post_reversal_boundary_protocol_v1.json';cfg=json.loads(p.read_text())
 for n,h in cfg['input_sha256'].items():assert sha(Path(n))==h
 rows=json.loads((D/'policy_records.json').read_text());states=json.loads((D/'states.json').read_text());armed=next(i for i,r in enumerate(rows) if r.get('tube_viability_event_armed',False));F=next(i for i,r in enumerate(rows) if i>armed and r['local_plan_created']);row=rows[F];g=row['reference_decision']['guard'];screen=row['reference_decision']['tube_viability_screen'];assert screen and not screen['pass'] and g['retention_necessary_screen_pass']
 m=np.load(D/'maps'/f'frame_{F:03d}.npz');origin=m['origin'];res=float(m['resolution']);haz=np.argwhere(m['labels']==1);world=origin+haz*res;best=None
 for key in ('bridge_xy','suffix_xy'):
  q=np.asarray(g[key],float);arc=0.
  for j,(a,b) in enumerate(zip(q[:-1],q[1:])):
   delta=b-a;l=float(np.linalg.norm(delta));u=np.zeros(len(world)) if l==0 else np.clip((world-a)@delta/(l*l),0,1);projection=a+u[:,None]*delta;dist=np.linalg.norm(world-projection,axis=1);k=int(dist.argmin());v=dict(distance_m=float(dist[k]),hazard_cell=haz[k].tolist(),cell_world=world[k].tolist(),line=key,segment=j,line_arc_m=arc+float(u[k])*l,projection_world=projection[k].tolist());arc+=l
   if best is None or v['distance_m']<best['distance_m']:best=v
 assert abs(best['distance_m']-screen['minimum_continuous_cell_clearance_m'])<1e-7
 snap=json.loads((D/'raw_point_memory_snapshots'/f'frame_{F:03d}.json').read_text());summaries=[];inputs={str(D/'maps'/f'frame_{F:03d}.npz'):sha(D/'maps'/f'frame_{F:03d}.npz'),str(D/'raw_point_memory_snapshots'/f'frame_{F:03d}.json'):sha(D/'raw_point_memory_snapshots'/f'frame_{F:03d}.json')}
 for rec in snap['records']:
  points=np.asarray(rec['world']).reshape(-1,3);cells=np.rint((points[:,:2]-origin)/res).astype(int);ids=np.flatnonzero(np.all(cells==best['hazard_cell'],axis=1))
  if not len(ids):continue
  f=rec['frame'];assert f<=F and rec['time_s']<=row['time_s']+1e-7;fp=D/'frames'/f'frame_{f:03d}.npz';inputs[str(fp)]=sha(fp);data=np.load(fp);pixels=np.asarray(rec['query_pixel_id'],int)[ids];depth=data['left_depth_m'].ravel()[pixels];x=pixels%640;y=pixels//640;ray=np.c_[(x-319.5)/584.8888888888889,(y-239.5)/584.8888888888889,np.ones(len(ids))];T=data['left_camera_usd_world'];teacher=T[:3,3]+(ray*depth[:,None])@(T[:3,:3]@np.diag([1,-1,-1])).T;valid=np.isfinite(depth)&(depth>0);floor=valid&(teacher[:,2]<=.05);hazard=valid&(teacher[:,2]>.05)&(teacher[:,2]<=1.5)
  summaries.append(dict(source_frame=f,queries=len(ids),prediction_z_min=float(points[ids,2].min()),prediction_z_max=float(points[ids,2].max()),teacher_floor=int(floor.sum()),teacher_hazard=int(hazard.sum()),teacher_unlabelled=int((~(floor|hazard)).sum()),teacher_z_min=float(teacher[valid,2].min()) if valid.any() else None,teacher_z_max=float(teacher[valid,2].max()) if valid.any() else None))
 out=dict(post_measured_reversal_event_frame=armed,first_subsequent_rebuild_frame=F,actual_time_s=row['time_s'],current_world_xy=states[F]['position'][:2],reference_remaining_arc_m=g['remaining_arc_m'],closest_risk=best,closest_projection_distance_from_robot_m=float(np.linalg.norm(np.asarray(best['projection_world'])-states[F]['position'][:2])),required_clearance_m=screen['existing_planning_clearance_m'],current_sources=summaries,untraceable_persistent_cell=not bool(summaries),same_exposure_teacher_only=True,no_new_physics_or_planner_query=True,no_grid_deletion_or_candidate=True,no_counterfactual_trajectory=True,not_a_source_or_3D_safety_admission=True,geometry_log_reconstruction_pass=True,input_sha256=inputs,protocol_sha256=sha(p))
 target=H/'reverse_main_post_reversal_boundary_audit_v1.json';assert not target.exists();target.write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out))
if __name__=='__main__':main()
