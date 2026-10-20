"""Two cells selected by already-frozen finite prefix audit, retrospective only."""
from pathlib import Path
import json,hashlib,numpy as np
H=Path(__file__).resolve().parent
D=H/'reverse_main_navigation_e1_b0_run_v1'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def main():
 scope=H/'current_forward_blocking_source_scope_v1.json'; prior=H/'current_forward_root_geometry_assessment_v1.json';a=json.loads(prior.read_text())
 for n,h in a['source_and_input_sha256'].items():assert sha(Path(n))==h
 cells=sorted(set(tuple(b['hazard_cell']) for c in a['checks'] for b in c['blocked']));assert cells==[(229,192),(229,193)]
 F=120;mp=D/'maps/frame_120.npz';sp=D/'raw_point_memory_snapshots/frame_120.json';m=np.load(mp);snap=json.loads(sp.read_text());origin=m['origin'];res=float(m['resolution']);clock=a['current_time_s'];out=[]
 inputs={str(p):sha(p) for p in [scope,prior,mp,sp,Path(__file__)]}
 for cell in cells:
  assert m['labels'][cell]==1
  records=[]
  for rec in snap['records']:
   points=np.asarray(rec['world']).reshape(-1,3);grid=np.rint((points[:,:2]-origin)/res).astype(int);ids=np.flatnonzero(np.all(grid==cell,axis=1))
   if not len(ids):continue
   f=int(rec['frame']);t=float(rec['time_s']);assert f<=F and t<=clock+1e-7
   fp=D/'frames'/f'frame_{f:03d}.npz';inputs[str(fp)]=sha(fp);data=np.load(fp);pix=np.asarray(rec['query_pixel_id'],int)[ids];depth=data['left_depth_m'].ravel()[pix];x=pix%640;y=pix//640
   ray=np.c_[(x-319.5)/584.8888888888889,(y-239.5)/584.8888888888889,np.ones(len(ids))];T=data['left_camera_usd_world'];teacher=T[:3,3]+(ray*depth[:,None])@(T[:3,:3]@np.diag([1,-1,-1])).T
   valid=np.isfinite(depth)&(depth>0);floor=valid&(teacher[:,2]<=.05);hazard=valid&(teacher[:,2]>.05)&(teacher[:,2]<=1.5)
   records.append(dict(source_frame=f,time_s=t,age_s=clock-t,queries=len(ids),query_pixel_id=pix.tolist(),predicted_world_xyz=points[ids].tolist(),teacher_valid_depth=valid.tolist(),teacher_world_xyz=[[float(v) if np.isfinite(v) else None for v in q] for q in teacher],teacher_floor=int(floor.sum()),teacher_hazard=int(hazard.sum()),teacher_unlabelled=int((~(floor|hazard)).sum()),teacher_xy_discrepancy_m=[float(v) if np.isfinite(v) else None for v in np.linalg.norm(points[ids,:2]-teacher[:,:2],axis=1)]))
  out.append(dict(cell=list(cell),world_xy=(origin+np.asarray(cell)*res).tolist(),source_records=records,traceable=bool(records),queries=sum(v['queries'] for v in records),teacher_floor=sum(v['teacher_floor'] for v in records),teacher_hazard=sum(v['teacher_hazard'] for v in records),teacher_unlabelled=sum(v['teacher_unlabelled'] for v in records)))
 result=dict(boundary_frame=F,current_time_s=clock,cells=out,input_sha256=inputs,same_exposure_retrospective_teacher_only=True,no_actor_physics_solver_fit_or_mutation=True,scope_not_source_admission_or_full3D_safety=True,whole_mission_complete=False)
 p=H/'current_forward_blocking_source_assessment_v1.json';assert not p.exists();p.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n');print(json.dumps({'cells':[{k:c[k] for k in ('cell','queries','teacher_floor','teacher_hazard','teacher_unlabelled','traceable')} for c in out]}))
if __name__=='__main__':main()
