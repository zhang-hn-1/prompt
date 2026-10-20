"""Offline sealed-render geometry diagnostic; never an online perception source."""
from pathlib import Path
import sys,json,hashlib,time
import numpy as np
from scipy.spatial import cKDTree
from pxr import Usd,UsdGeom
D=Path(__file__).resolve().parent; W=D.parents[1]; R=W/'experiments/wide_pinhole_camera_20261015_v1/native_validation_run'
sys.path.insert(0,str(W/'experiments/camera_exposure_contract_20261015_v1'))
from exposure_contract import local_transform,AX
sys.path.insert(0,str(W/'experiments/stereo_memory_navigation_20261004_v1'))
from dynamic_self_mask import RobotSelfMask
F=584.8888888888889; FW=F/3; H=480; WIDTH=640
origin=np.load('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/data/benchmark_v1/occupancy/e1_detour_bodyband.npz')['global_origin_xy']
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save(name,x): (D/name).write_text(json.dumps(x,indent=2,allow_nan=False)+'\n')
def cell(w): return np.unique(np.floor((w[:,:2]-origin)/.1+.5).astype(np.int64),axis=0)
def st(c): return set(map(tuple,c.tolist()))
def arr(s):return np.array(sorted(s),dtype=np.int64).reshape(-1,2)
def quant(x):return dict(zip(['p50','p90','p99','max'],map(float,np.quantile(x,[.5,.9,.99,1])))) if len(x) else None
stage=Usd.Stage.Open(str(R/'stage_root_initial.usda'));cache=UsdGeom.XformCache();objects=[]
for p in stage.Traverse():
 if p.IsA(UsdGeom.Cylinder) and UsdGeom.Imageable(p).ComputeVisibility()!='invisible':
  c=UsdGeom.Cylinder(p);T=np.array(cache.GetLocalToWorldTransform(p)).T
  assert np.allclose(T[:3,:3],np.eye(3)) and c.GetAxisAttr().Get()=='Z'
  objects.append(dict(path=str(p.GetPath()),center=T[:3,3].tolist(),radius=float(c.GetRadiusAttr().Get()),height=float(c.GetHeightAttr().Get())))
assert objects
save('scene_cylinders.json',objects)
y,x=np.indices((H,WIDTH));rays={f:np.stack([(x.ravel()-319.5)/f,(y.ravel()-239.5)/f,np.ones(H*WIDTH)],axis=1) for f in [F,FW]}
def trace(T,f):
 d=rays[f]@(T[:3,:3]@AX).T; o=T[:3,3]; best=np.full(len(d),np.inf)
 for ob in objects:
  c=np.array(ob['center']);rad=ob['radius'];half=ob['height']/2;v=o-c
  A=np.sum(d[:,:2]**2,axis=1);B=2*(d[:,:2]@v[:2]);C=v[:2]@v[:2]-rad**2;disc=B*B-4*A*C
  sq=np.sqrt(np.maximum(disc,0))
  for sign in [-1,1]:
   t=(-B+sign*sq)/np.maximum(2*A,1e-30)
   good=(disc>=0)&(A>1e-20)&(t>=.01)&(np.abs(v[2]+t*d[:,2])<=half)
   best=np.minimum(best,np.where(good,t,np.inf))
  for zcap in [-half,half]:
   t=np.divide(zcap-v[2],d[:,2],out=np.full(len(d),np.inf),where=abs(d[:,2])>1e-20)
   xy=v[:2]+t[:,None]*d[:,:2]; good=(t>=.01)&(np.sum(xy*xy,axis=1)<=rad**2)
   best=np.minimum(best,np.where(good,t,np.inf))
 return best,d

def inview(w,T,f):
 c=(w-T[:3,3])@(T[:3,:3]@AX);z=c[:,2];u=f*c[:,0]/np.maximum(z,1e-20)+319.5;v=f*c[:,1]/np.maximum(z,1e-20)+239.5
 return (z>=.5)&(z<=3)&(u>=-.5)&(u<639.5)&(v>=-.5)&(v<479.5)

seal=json.loads((R/'capture_seal.json').read_text());cad=RobotSelfMask();rows=[];allerrors=[];clock=[];beg=time.monotonic();inputs={str(R/'capture_seal.json'):sha(R/'capture_seal.json'),str(R/'stage_root_initial.usda'):sha(R/'stage_root_initial.usda'),str(D/'protocol.json'):sha(D/'protocol.json')};cadrows=[]
for rr in seal['rows']:
 fp=R/'frames'/rr['frame'];assert sha(fp)==rr['sha256'];inputs[str(fp)]=sha(fp);a=np.load(fp)
 B=a['eye0_camera_usd_world_label']@np.linalg.inv(local_transform([.45,0,.05],0));root=a['root_position_physics_label'];Ts=[a[f'eye{e}_camera_usd_world_label'] for e in range(2)]
 original=[B@local_transform([.4,yy,.05],0) for yy in [.3,-.3]]
 arms=[];maps=[];images={};old=[]
 for e,T in enumerate(Ts):
  expected,d=trace(T,FW);z=a[f'eye{e}_depth_label'].ravel().astype(float);finite=np.isfinite(z);valid=finite&(z>=.5)&(z<=3);w=np.zeros((len(z),3));w[finite]=T[:3,3]+z[finite,None]*d[finite]
  near=np.linalg.norm(w-root,axis=1)>=.55;band=(w[:,2]>.05)&(w[:,2]<=1.5);supported=np.zeros(len(z),bool);both=finite&np.isfinite(expected);error=np.full(len(z),np.nan);error[both]=abs(z[both]-expected[both]);supported[both]=error[both]<=.02
  expected_domain=np.isfinite(expected)&(expected>=.5)&(expected<=3);errors=error[expected_domain&finite];allerrors.extend(errors.tolist())
  obstacle=valid&near&band&supported; cc=cell(w[obstacle]);maps.append(st(cc));images[f'eye{e}_gt_z']=z.reshape(H,WIDTH);images[f'eye{e}_cylinder_expected_z']=expected.reshape(H,WIDTH);images[f'eye{e}_error']=error.reshape(H,WIDTH);images[f'eye{e}_supported_mask']=obstacle.reshape(H,WIDTH);images[f'eye{e}_cells']=cc
  arms.append(dict(eye=e,pixels=len(z),finite_pixels=int(finite.sum()),range_pixels=int(valid.sum()),range_root_bodyband_pixels=int((valid&near&band).sum()),cylinder_supported_bodyband_pixels=int(obstacle.sum()),unidentified_bodyband_pixels=int((valid&near&band&~supported).sum()),near_ground_height_band_pixels=int((valid&near&(w[:,2]<=.05)&(w[:,2]>=-.05)).sum()),analytic_cylinder_inrange_rays=int(expected_domain.sum()),analytic_rays_finite_render=int((expected_domain&finite).sum()),analytic_ray_errors_m=quant(errors),analytic_rays_within_02m=int((expected_domain&supported).sum()),visible_cylinder_cells=len(cc)))
 for T in original:
  z,d=trace(T,F);valid=np.isfinite(z)&(z>=.5)&(z<=3);w=T[:3,3]+z[valid,None]*d[valid];w=w[(w[:,2]>.05)&(w[:,2]<=1.5)&(np.linalg.norm(w-root,axis=1)>=.55)];old.append(st(cell(w)))
 un=maps[0]|maps[1];oldun=old[0]|old[1];ret=len(oldun&un);q=arr(oldun);uu=arr(un)
 nearest=cKDTree(uu*.1).query(q*.1)[0] if len(uu) else np.full(len(q),np.inf)
 # Exact all CAD vertices at each captured current joint state, independent of FOV.
 transforms=cad.body_transforms(a['joints_current']);ext=[]
 for k,tris in cad.triangles.items():
  t=transforms[k];xx=tris.reshape(-1,3)@t[0,:3]+t[0,3];ext +=[float(xx.min()),float(xx.max())]
 cadrows.append(dict(frame=rr['frame'],front_max_z=max(ext)-.45,rear_max_z=-min(ext)-.45,all_triangles_behind_near_clip=bool(max(max(ext)-.45,-min(ext)-.45)<.01)))
 theta=np.arange(3600)*2*np.pi/3600;local=np.stack([2*np.cos(theta),2*np.sin(theta),np.full(3600,.05)],axis=1);world=local@B[:3,:3].T+B[:3,3];uv=[inview(world,T,FW) for T in Ts];ov=[inview(world,T,F) for T in original]
 geom=dict(equal_angle_samples=3600,new_front=int(uv[0].sum()),new_rear=int(uv[1].sum()),new_union=int((uv[0]|uv[1]).sum()),new_overlap=int((uv[0]&uv[1]).sum()),new_blind=int((~(uv[0]|uv[1])).sum()),old_union=int((ov[0]|ov[1]).sum()))
 imu=rr['imu'];exposure=float(a['eye0_render_time']);dt=exposure-imu['begin_raw_s'];clock.append(dict(frame=rr['frame'],logical=float(a['source_time_s']),exposure_raw=exposure,raw_logical_offset_s=exposure-float(a['source_time_s']),IMU_interval_begin_raw=imu['begin_raw_s'],IMU_interval_end_raw=imu['end_raw_s'],exposure_minus_interval_end_s=exposure-imu['end_raw_s'],exposure_minus_sensor_stamp_s=exposure-imu['sensor_begin_stamp_s'],interval_duration_s=dt,gyro_norm_times_interval_rad=float(np.linalg.norm(imu['gyro_body_rad_s'])*dt)))
 row=dict(frame=rr['frame'],time_s=float(a['source_time_s']),arms=arms,front_cells=len(maps[0]),rear_cells=len(maps[1]),overlap_cells=len(maps[0]&maps[1]),union_cells=len(un),rear_unique_cells=len(maps[1]-maps[0]),old_analytic_cells=len(oldun),old_analytic_exact_retained=ret,old_analytic_within_15cm=int((nearest<=.15).sum()),old_analytic_grid_distance_m=quant(nearest[np.isfinite(nearest)]),equal_angle_geometry=geom)
 rows.append(row);images.update(union_cells=arr(un),old_analytic_cells=arr(oldun),origin=origin,root=root,base=B,angle_visible=(uv[0]|uv[1]),angle_points=world)
 np.savez_compressed(D/(fp.stem+'_diagnostic.npz'),**images)
 print(rr['frame'],[len(m) for m in maps],len(un),len(oldun),ret,flush=True)
agg={k:sum(r[k] for r in rows) for k in ['front_cells','rear_cells','overlap_cells','union_cells','rear_unique_cells','old_analytic_cells','old_analytic_exact_retained','old_analytic_within_15cm']};agg['old_analytic_exact_retention']=agg['old_analytic_exact_retained']/agg['old_analytic_cells'];agg['old_analytic_within_15cm_retention']=agg['old_analytic_within_15cm']/agg['old_analytic_cells']
result=dict(frames=rows,aggregate_grid_instances=agg,analytic_cylinder_ray_errors_m=quant(allerrors),CAD=cadrows,clock=clock,objects=len(objects),wall_s=time.monotonic()-beg,physical=0,fit=0,model=0,source_navigation_admitted=False,limitations=['All geometry uses renderer labels and actual scene cylinders offline, no learned prediction','Remote default environment cannot be resolved from export in offline USD; full-scene nearest-hit and full-scene ground semantics NOT certified','Old narrow reference is hypothetical same-pose analytic cylinder-only reference, not same-time original RGB/depth or actual original visible denominator','Visible grids require saved depth agreement <=.02m with exported analytic cylinder; unmatched finite bodyband points unknown and counted','Only 8 frames/4 seconds native zero-request motion, no scene/trajectory generalization; clipped CAD certificate does not certify hardware mount','No learned ground false-positive rate, BEV accuracy, memory or navigation metrics generated'],input_sha256=inputs)
save('result.json',result);save('CAD_identity.json',cad.identity);print(agg,flush=True)
