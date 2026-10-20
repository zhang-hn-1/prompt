from pathlib import Path
import numpy as np,json,hashlib,datetime,importlib.util,importlib.metadata
from mvs_causal_packet_bridge import build_request
from temporal_ray_sweep import project_depth
H=Path(__file__).resolve().parent;O=H/'qualified_hazard_navigation_run_v1';D=H/'mvsanywhere_source_preflight_v1';sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();inputs={};rows=[]
def load(p):inputs[str(p.resolve())]=sha(p);return np.load(p)
for i in [235,245,255]:
 for eye in ['left','right']:
  def packet(j):
   rgb=load(O/'online_inputs'/f'frame_{j:03d}.npz');pose=load(O/'known_poses'/f'frame_{j:03d}.npz');mask=load(O/'masks'/f'frame_{j:03d}.npz');assert float(rgb['time_s'])==float(pose['time_s']);return dict(time_s=float(rgb['time_s']),rgb=rgb[eye+'_rgb'],pose=pose[eye+'_camera_usd_world'],mask=mask[eye])
  cur=packet(i);src=[packet(i-10),packet(i-20)];r=build_request(cur,src,cur['time_s']);nativeK=np.array([[584.8888888888889,0,319.5],[0,584.8888888888889,239.5],[0,0,1]]);uv=np.array([[50.,60],[319.5,239.5],[550,400]]);z=np.array([.5,1.5,3]);errors=[]
  for k,s in enumerate(src):
   native,_=project_depth(uv,z,cur['pose'],s['pose'],nativeK);ray=np.c_[uv+.5,np.ones(3)]@np.linalg.inv(r['cur_intrinsics'][:3,:3]).T;world=r['cur_pose'][:3,3]+(ray*z[:,None])@r['cur_pose'][:3,:3].T;local=(world-r['src_pose'][k,:3,3])@r['src_pose'][k,:3,:3];p=local@r['src_intrinsics'][k,:3,:3].T;projected=p[:,:2]/p[:,2,None]-.5;errors.append(float(np.max(np.abs(projected-native))))
  rows.append(dict(frame=i,eye=eye,current_time_s=r['current_time_s'],source_times_s=r['source_time_s'].tolist(),projection_max_abs_native_pixel_error=max(errors),future_information_not_loaded=True,raw_RGB_unchanged=bool(np.array_equal(r['cur_image'],cur['rgb']) and all(np.array_equal(a,b['rgb']) for a,b in zip(r['src_image'],src))),CAD_only_metadata_not_model_token_mask=True))
names=['torch','lightning','timm','kornia','einops','torchvision','cv2','numpy','moviepy','loguru','antialiased_cnns'];deps={n:bool(importlib.util.find_spec(n)) for n in names};path=D/'packet_bridge_audit_v1.json';assert not path.exists();path.write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),input_sha256=inputs,bridge_sha256=sha(H/'mvs_causal_packet_bridge.py'),rows=rows,dependencies_present=deps,no_truth_or_labels_read=True,no_weights_or_inference=True),indent=2)+'\n');print(rows,deps)
