"""Offline cause measurement for two fresh hazard cells at local-arm divergence."""
from pathlib import Path
import json,hashlib,numpy as np
from pxr import Usd,UsdGeom
H=Path(__file__).resolve().parent;O=H/'prior_rgb_history_run';t=json.loads((O/'task_episode.json').read_text());scene=Path('/home/an/project/NavDP/assets/scenes/benchmark_e1/e1_detour.usda');assert hashlib.sha256(scene.read_bytes()).hexdigest()==t['original_episode']['scene_sha256'];stage=Usd.Stage.Open(str(scene));cache=UsdGeom.XformCache();cyl=[]
for prim in stage.Traverse():
 if prim.IsA(UsdGeom.Cylinder):
  c=UsdGeom.Cylinder(prim);a=np.array(cache.GetLocalToWorldTransform(prim));assert np.allclose(a[:3,:3],np.eye(3));assert c.GetAxisAttr().Get()=='Z';cyl.append((a[3,:3],float(c.GetRadiusAttr().Get()),float(c.GetHeightAttr().Get())))
assert cyl
def nearest_surface(points):
 d=np.abs(points[:,2])
 for c,r,h in cyl:
  q=np.c_[np.linalg.norm(points[:,:2]-c[:2],axis=1)-r,np.abs(points[:,2]-c[2])-h/2];sd=np.minimum(q.max(1),0)+np.linalg.norm(np.maximum(q,0),axis=1);d=np.minimum(d,np.abs(sd))
 return d
p=np.load(O/'sensor_packets/source_135.npz');m=np.load(O/'maps/frame_135.npz');cells=np.rint((p['world'][:,:2]-m['origin'])/float(m['resolution'])).astype(int);wanted=np.rint((np.array([[-1.1,-.2],[-1.0,-.2]])-m['origin'])/float(m['resolution'])).astype(int);selected=np.any(np.all(cells[:,None,:]==wanted[None,:,:],axis=2),axis=1);selected&=(p['world'][:,2]>.05)&(p['world'][:,2]<=1.5);ids=np.flatnonzero(selected);truth=np.load(O/'offline_truth/frame_135.npz');yy=p['query_pixel_id'][ids]//640;xx=p['query_pixel_id'][ids]%640;z=truth['left_depth_m'][yy,xx];pose=truth['left_camera_usd_world'];ray=np.c_[(xx-319.5)/584.8888888888889,(yy-239.5)/584.8888888888889,np.ones(len(ids))];actual=pose[:3,3]+ray*z[:,None]@(pose[:3,:3]@np.diag([1,-1,-1])).T;finite=np.isfinite(actual).all(1);ds=nearest_surface(p['world'][ids]);da=nearest_surface(actual[finite]);err=np.linalg.norm(p['world'][ids][finite]-actual[finite],axis=1)
r=dict(scope='offline two fresh offending cells only; exact authoredstatic-cylinder/ground surface distance, not fullrender/robot/lighting geometry',scene_sha256=hashlib.sha256(scene.read_bytes()).hexdigest(),cylinders=len(cyl),frame=135,source_time_s=float(p['time_s']),points=len(ids),cells_world_xy=(m['origin']+wanted*float(m['resolution'])).tolist(),predicted_world=p['world'][ids].tolist(),query_pixels=p['query_pixel_id'][ids].tolist(),finite_truth=int(finite.sum()),truth_world=actual[finite].tolist(),model_error_m=err.tolist(),predicted_nearest_static_surface_distance_m=ds.tolist(),truth_nearest_static_surface_distance_m=da.tolist(),source_sigma_sensitivity_m=p['sigma_m'][ids].tolist(),conclusion='measure where fresh estimated endpoints lie; do not remove points or alter frozen source gates')
(H/'rgb_fresh_blocking_cells_diagnosis.json').write_text(json.dumps(r,indent=2)+'\n');print('two fresh cells: points',len(ids),'finite truth',finite.sum(),'model errors',err,'static distance pred',ds,'truth',da)
