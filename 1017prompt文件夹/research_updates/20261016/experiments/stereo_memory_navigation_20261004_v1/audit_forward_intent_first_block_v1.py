"""Single preselected first route loss; GT exposure labels offline only."""
from pathlib import Path
import json,hashlib,numpy as np
from scipy.ndimage import distance_transform_edt
H=Path(__file__).resolve().parent;D=H/'forward_intent_navigation_e1_candidate_run_v1';F=155
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def main():
 cfg=H/'forward_intent_first_block_protocol_v1.json';r=json.loads(cfg.read_text())
 for p,s in r['input_sha256'].items():assert sha(Path(p))==s
 rows=json.loads((D/'policy_records.json').read_text());states=json.loads((D/'states.json').read_text());row=rows[F];assert row['plan']['status']=='observed_obstacle_blocks_route' and all(x.get('plan',{}).get('status')=='success' for x in rows[60:F])
 m=np.load(D/'maps'/f'frame_{F:03d}.npz');origin=m['origin'];res=float(m['resolution']);labels=m['labels'];start=np.asarray(states[F]['position'][:2]);goal=np.asarray(json.loads((D/'task_episode.json').read_text())['actual_episode']['goal_position_world_m'][:2]);cells=np.rint((np.array([start,goal])-origin)/res).astype(int)
 clearance=distance_transform_edt(np.pad(labels!=1,1))[1:-1,1:-1]*res;haz=np.argwhere(labels==1);dist=np.linalg.norm(haz-cells[0],axis=1)*res;nearest=haz[np.argmin(dist)];summaries=[]
 snap=json.loads((D/'raw_point_memory_snapshots'/f'frame_{F:03d}.json').read_text())
 for rec in snap['records']:
  world=np.asarray(rec['world']).reshape(-1,3);c=np.rint((world[:,:2]-origin)/res).astype(int);ids=np.flatnonzero(np.all(c==nearest,axis=1))
  if not len(ids):continue
  frame=rec['frame'];assert frame<=F and rec['time_s']<=row['time_s']+1e-7;data=np.load(D/'frames'/f'frame_{frame:03d}.npz');pixel=np.asarray(rec['query_pixel_id'],int)[ids];depth=data['left_depth_m'].ravel()[pixel];x=pixel%640;y=pixel//640;ray=np.c_[(x-319.5)/584.8888888888889,(y-239.5)/584.8888888888889,np.ones(len(ids))];T=data['left_camera_usd_world'];teacher=T[:3,3]+(ray*depth[:,None])@(T[:3,:3]@np.diag([1,-1,-1])).T;valid=np.isfinite(depth)&(depth>0);floor=valid&(teacher[:,2]<=.05);hazard=valid&(teacher[:,2]>.05)&(teacher[:,2]<=1.5)
  summaries.append(dict(source_frame=frame,query_count=len(ids),predicted_z_min=float(world[ids,2].min()),predicted_z_max=float(world[ids,2].max()),teacher_floor_count=int(floor.sum()),teacher_hazard_count=int(hazard.sum()),teacher_unlabelled_count=int((~(floor|hazard)).sum()),teacher_z_min=float(teacher[valid,2].min()) if valid.any() else None,teacher_z_max=float(teacher[valid,2].max()) if valid.any() else None))
 result=dict(first_route_loss_frame=F,start_cell=cells[0].tolist(),goal_cell=cells[1].tolist(),start_clearance_m=float(clearance[tuple(cells[0])]),goal_clearance_m=float(clearance[tuple(cells[1])]),frozen_clearance_required_m=.6+.2+res*np.sqrt(2)/2,nearest_hazard_cell=nearest.tolist(),nearest_cell_center_world=(origin+nearest*res).tolist(),nearest_cell_grid_distance_m=float(dist.min()),nearest_cell_traceable_sources=summaries,source_records_may_not_cover_old_persistent_cell=not bool(summaries),teacher_not_actor_input=True,no_grid_filter_or_policy_replay=True,no_new_physics=True,scope='One first blocked query and nearest cell only; not whole source reliability, alternate route feasibility or safety proof',protocol_sha256=sha(cfg))
 p=H/'forward_intent_first_block_audit_v1.json';assert not p.exists();p.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
if __name__=='__main__':main()
