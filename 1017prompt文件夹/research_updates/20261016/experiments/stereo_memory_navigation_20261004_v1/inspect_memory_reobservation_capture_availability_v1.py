"""One fixed first-capture schema/ground-support probe per sealed whole sequence.
Retrospective teacher pixels are development labels only, never runtime inputs.
"""
from pathlib import Path
import json,hashlib,numpy as np
H=Path(__file__).resolve().parent
if __name__=='__main__':
 registry=json.loads((H/'classical_baseline_registry_v3.json').read_text());result=[]
 for row in [r for layer in registry['layers'].values() for r in layer]:
  d=H/row['run'];p=d/'frames/frame_000.npz';a=np.load(p);seal=json.loads(Path(row['original_manifest']).read_text())['artifact_sha256'];digest=hashlib.sha256(p.read_bytes()).hexdigest();assert seal[str(p)]==digest
  depth=a['left_depth_m'];T=a['left_camera_usd_world'];yy,xx=np.mgrid[0:480:8,0:640:8];z=depth[yy,xx].ravel();rays=np.stack([(xx.ravel()-319.5)/584.8888888888889,(yy.ravel()-239.5)/584.8888888888889,np.ones(len(z))],axis=1);world=(rays*z[:,None])@((T[:3,:3]@np.diag([1.,-1.,-1.])).T)+T[:3,3];finite=np.isfinite(world).all(axis=1)&np.isfinite(z)&(z>=.5)&(z<=3.)
  ground=finite&(world[:,2]>=-.02)&(world[:,2]<=.05);obstacle=finite&(world[:,2]>.05)&(world[:,2]<=1.5)
  result.append(dict(sequence=row['run'],frame=0,time_s=float(a['time_s']),sampled_pixels=len(z),teacher_ground_support_pixels=int(ground.sum()),teacher_elevated_surface_pixels=int(obstacle.sum()),original_capture_sha256=digest))
 out=dict(scope='Fixed first-capture grid-stride8 schema/label-availability probe only; GT teacher depth/transform retrospective, no source accuracy/free-volume/same-cell visibility/OOD/fresh validation claim',sequences=result,new_physics=0,new_fit=0,whole_mission_complete=False,next_requirement='A temporal same-cell independent ray visibility/mask and ground-vs-elevated-surface target must be specified and tested before constructing learner dataset. Ground support pixels are not observed-free 3D cells.')
 p=H/'memory_reobservation_capture_availability_v1.json';assert not p.exists();p.write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(result))
