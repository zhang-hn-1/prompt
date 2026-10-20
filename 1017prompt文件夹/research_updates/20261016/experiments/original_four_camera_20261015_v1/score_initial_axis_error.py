"""Complete four-native-view sensor/GT visibility audit; no learned predictions."""
from pathlib import Path
import json,sys,hashlib,time,numpy as np
from scipy.spatial import cKDTree
from four_camera import NAMES,nominal_transforms,validate_complete
from cad_masks import FourCAD
D=Path(__file__).resolve().parent;R=D/sys.argv[1];cfg=json.loads((D/'protocol.json').read_text());status=json.loads((D/(R.name+'_status.json')).read_text());seal=validate_complete(status,R);cad=FourCAD();ts=nominal_transforms(.2,.3,.05);F=cfg['focal_px'];AX=np.diag([1,-1,-1]);occ=np.load(cfg['occupancy']);origin=occ['global_origin_xy'];print('reference grid labels',np.unique(occ['grid']),flush=True)
# The saved occupancy convention is obtained from the accompanying bodyband raster: occupied=1.
assert set(np.unique(occ['grid'])).issubset({0,1});yy,xx=np.where(occ['grid']>0);ref=np.c_[xx,yy]*float(occ['resolution_m'])+origin;tree=cKDTree(ref)
y,x=np.indices((480,640));rays=np.stack([(x-319.5)/F,(y-239.5)/F,np.ones((480,640))],axis=-1)
cell=lambda w:np.unique(np.floor((w[:,:2]-origin)/.1+.5).astype(np.int64),axis=0)
st=lambda c:set(map(tuple,c.tolist()))
arr=lambda s:np.array(sorted(s),dtype=np.int64).reshape(-1,2)
rows=[];start=time.monotonic();maxinverse=0.;maxrelative=0.;first=True
for rr in seal['rows']:
 a=np.load(R/'frames'/rr['frame']);masks=cad.masks(a['joints_current']);sets=[];arms=[];save={};Bs=[]
 for j,name in enumerate(NAMES):
  T=a[name+'_camera_usd_world_label'];B=T@np.linalg.inv(ts[j]);Bs.append(B);z=a[name+'_depth_label'];finite=np.isfinite(z);domain=finite&(z>=.5)&(z<=3);eligible=domain&~masks[name];w=T[:3,3]+(rays[eligible]*z[eligible,None])@(T[:3,:3]@AX).T;outside=np.linalg.norm(w-a['root_position_physics_label'],axis=1)>=.55;w=w[outside];body=(w[:,2]>.05)&(w[:,2]<=1.5);bw=w[body];cc=cell(bw);sets.append(st(cc));dist=tree.query(cc*.1+origin)[0] if len(cc) else np.array([]);good=dist<=.15
  # Independent inverse pinhole reprojection of all eligible pixels before root filtering.
  wy=T[:3,3]+(rays[eligible]*z[eligible,None])@(T[:3,:3]@AX).T;c=(wy-T[:3,3])@(T[:3,:3]@AX);uv=c[:,:2]/c[:,2,None]*F+[319.5,239.5];pix=np.column_stack([x[eligible],y[eligible]]);inv=float(np.max(abs(uv-pix))) if len(uv) else 0.;maxinverse=max(maxinverse,inv)
  arms.append(dict(name=name,all_pixels=int(z.size),CAD_pixels=int(masks[name].sum()),finite_pixels=int(finite.sum()),range_pixels=int(domain.sum()),range_nonCAD_pixels=int(eligible.sum()),root_excluded_points=int((~outside).sum()),bodyband_points=int(body.sum()),ground_height_band_points=int(((w[:,2]>=-.05)&(w[:,2]<=.05)).sum()),above_bodyband_points=int((w[:,2]>1.5).sum()),visible_bodyband_cells=len(cc),near_bodyband_reference_cells=int(good.sum()),far_from_reference_cells=int((~good).sum()),reference_distance_p90_m=float(np.quantile(dist,.9)) if len(dist) else None))
  save[name+'_cells']=cc;save[name+'_CAD']=masks[name];save[name+'_far_cells']=cc[~good]
 pairmax=max(float(np.max(abs(B-Bs[0]))) for B in Bs);maxrelative=max(maxrelative,pairmax)
 front=sets[0]|sets[1];rear=sets[2]|sets[3];union=front|rear;near_union=tree.query(arr(union)*.1+origin)[0]<=.15 if union else np.array([],bool)
 # Body-plane ring measures FOV/range/CAD eligibility; no scene visibility claim.
 theta=np.arange(3600)*2*np.pi/3600;p=np.c_[2*np.cos(theta),2*np.sin(theta),np.full(3600,.05)];world=p@Bs[0][:3,:3].T+Bs[0][:3,3];vis=[]
 for j,name in enumerate(NAMES):
  T=a[name+'_camera_usd_world_label'];c=(world-T[:3,3])@(T[:3,:3]@AX);u=F*c[:,0]/np.maximum(c[:,2],1e-20)+319.5;v=F*c[:,1]/np.maximum(c[:,2],1e-20)+239.5;k=(c[:,2]>=.5)&(c[:,2]<=3)&(u>=-.5)&(u<639.5)&(v>=-.5)&(v<479.5);ids=np.where(k)[0];k[ids]&=~masks[name][np.clip(np.rint(v[ids]).astype(int),0,479),np.clip(np.rint(u[ids]).astype(int),0,639)];vis.append(k)
 ring=np.any(vis,axis=0);row=dict(frame=rr['frame'],time_s=float(a['source_time_s']),arms=arms,front_union=len(front),rear_union=len(rear),front_rear_overlap=len(front&rear),all_union=len(union),rear_new=len(rear-front),union_near_reference=int(near_union.sum()),union_far_reference=int((~near_union).sum()),body_transform_agreement=pairmax,ring_visible=int(ring.sum()),ring_samples=3600,IMU_sensor_to_exposure_s=float(a['front_left_render_time'])-rr['imu']['sensor_begin_stamp_s'],IMU_end_to_exposure_s=float(a['front_left_render_time'])-rr['imu']['end_raw_s'])
 rows.append(row);save.update(front_union=arr(front),rear_union=arr(rear),all_union=arr(union),origin=origin,root=a['root_position_physics_label'],reference_xy=ref,ring_points=world,ring_visible=ring);np.savez_compressed(D/(Path(rr['frame']).stem+'_audit.npz'),**save);print(rr['frame'],[ar['visible_bodyband_cells'] for ar in arms],len(front),len(rear),len(union),flush=True)
agg={k:sum(r[k] for r in rows) for k in ['front_union','rear_union','front_rear_overlap','all_union','rear_new','union_near_reference','union_far_reference']};term=json.loads((R/'terminal.json').read_text());result=dict(complete=True,frames=rows,aggregate_grid_instances=agg,maximum_homogeneous_roundtrip_pixel_error=maxinverse,maximum_four_derived_body_matrix_difference=maxrelative,physics_ticks=len(np.load(R/'physics.npz')['simulation_time_s'])-1,contacts=term['contacts'],wall_s=time.monotonic()-start,model=0,fit=0,navigation=0,scope='GT-only native four-view visibility geometry, not online source qualification, metric depth accuracy or full scene recall. Bodyband is GT world height diagnostic; no predicted semantics/free. Front .20m configuration differs from historical .40m development data; no cross-run causal comparison.',source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [D/'protocol.json',D/'score.py',D/'cad_masks.py',R/'capture_seal.json',R/'physics.npz',Path(cfg['occupancy'])]})
(D/'geometry_result.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n');print(agg,flush=True)
