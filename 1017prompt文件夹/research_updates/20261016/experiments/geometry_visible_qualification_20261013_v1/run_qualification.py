from pathlib import Path
import sys,json,hashlib,numpy as np
from scipy.spatial.transform import Rotation
D=Path(__file__).resolve().parent;W=D.parents[1];L=W/'experiments/learned_visible_obstacles_20261013_v1';C=W/'experiments/source_confidence_calibration_20261012_v1';V=W/'experiments/viewpoint_confirmation_navigation_20261012_v1';H=W/'experiments/stereo_memory_navigation_20261004_v1';P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')
sys.path[:0]=[str(D),str(L),str(V),str(H),str(P)]
from projection import backproject,project_world,root_from_camera
from visible_target import rasterize
from rig_adapter import Rig,RigMask
s=(L/'score.py').read_text();ns={};exec('import numpy as np\nfrom scipy.ndimage import maximum_filter,distance_transform_edt\n'+s[s.index('def metrics('):s.index('\nrows=[]')],ns);metrics=ns['metrics']
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
old=json.loads((L/'artifact_manifest_post_close.json').read_text());assert all(sha(Path(p))==h for p,h in old['artifact_sha256'].items())
cad=RigMask(Rig('forward',.4,.3,.05));rows=[];inputs={};details=[]
for name,R in [('H',W/'experiments/risk_height_navigation_20261012_v1/candidate_e2_run'),('E0',C/'data_e0_run'),('E1',C/'data_e1_run')]:
 states=json.loads((R/'states.json').read_text());Y=np.load(L/'data'/f'{name}.npz')['labels'];variants={k:[] for k in ['all','potential','verified','native']};pixelcounts={k:0 for k in ['all','potential','verified','opposite_occluded','opposite_nohit','opposite_unavailable']}
 for j,i in enumerate(range(0,901,5)):
  p=R/'frames'/f'frame_{i:06d}.npz'
  # The source file names are read from the established manifests, not guessed.
  p=sorted((R/'frames').glob('*.npz'))[j] if not p.exists() else p
  assert int(p.stem.split('_')[1])==i
  f=np.load(p);npth=R/'known_poses'/p.name;n=np.load(npth);sp=R/'source_inputs'/p.name;source=np.load(sp);t=float(f['time_s']);assert t==float(n['time_s'])==float(source['time_s']);assert abs(t-float(states[i]['time_s']))<1e-6
  for eye in ['left','right']:assert np.array_equal(f[eye+'_rgb'],source[eye+'_rgb'])
  root=f['root_position_world_m'];q=f['root_quaternion_world_wxyz'];RR=Rotation.from_quat(q[[1,2,3,0]]).as_matrix();yaw=np.arctan2(RR[1,0],RR[0,0]);NR,nroot=root_from_camera(n['left_camera_usd_world']);nyaw=np.arctan2(NR[1,0],NR[0,0]);m=cad.project(np.array(states[i]['joints']));cloud={k:[] for k in variants};framecounts={k:0 for k in pixelcounts}
  for eye,other in [('left','right'),('right','left')]:
   depth=f[eye+'_depth_m'];vy,vx=np.where(np.isfinite(depth)&(depth>=.5)&(depth<=3)&~m[eye]);z=depth[vy,vx];world=backproject(vx,vy,z,f[eye+'_camera_usd_world']);outside=np.linalg.norm(world-root,axis=1)>=.55;vx=vx[outside];vy=vy[outside];z=z[outside];world=world[outside]
   u,v,zother=project_world(world,f[other+'_camera_usd_world']);inside=np.isfinite(u)&np.isfinite(v)&(u>=0)&(u<=639)&(v>=0)&(v<=479)&(zother>=.5)&(zother<=3);ix=np.clip(np.rint(np.nan_to_num(u)).astype(int),0,639);iy=np.clip(np.rint(np.nan_to_num(v)).astype(int),0,479);potential=inside&~m[other][iy,ix];od=f[other+'_depth_m'][iy,ix];finite=potential&np.isfinite(od)&(od>=.5)&(od<=3);verified=finite&(np.abs(od-zother)<=.03)
   cloud['all'].append(world);cloud['potential'].append(world[potential]);cloud['verified'].append(world[verified]);nw=backproject(vx,vy,z,n[eye+'_camera_usd_world']);cloud['native'].append(nw)
   fc=dict(all=len(world),potential=int(potential.sum()),verified=int(verified.sum()),opposite_occluded=int((finite&(od<zother-.03)).sum()),opposite_nohit=int((potential&np.isposinf(od)).sum()),opposite_unavailable=int((potential&~finite&~np.isposinf(od)).sum()))
   for k,value in fc.items():framecounts[k]+=value;pixelcounts[k]+=value
  for k in variants:variants[k].append(rasterize(np.concatenate(cloud[k]),nroot if k=='native' else root,nyaw if k=='native' else yaw))
  assert np.array_equal(variants['all'][-1],Y[j]),(name,i,'GT target differs')
  for pp in [p,npth,sp]:inputs[str(pp)]=sha(pp)
  details.append(dict(group=name,frame=i,time_s=t,**framecounts))
  if j%45==0:print(name,j,framecounts,flush=True)
 for pp in [R/'states.json',L/'data'/f'{name}.npz',R/'termination.json']:inputs[str(pp)]=sha(pp)
 arrays={k:np.stack(v) for k,v in variants.items()};np.savez_compressed(D/f'{name}_oracle_maps.npz',**arrays)
 row=dict(group=name,frames=181,pixel_counts=pixelcounts,metrics={k:metrics(Y,v) for k,v in arrays.items() if k!='all'})
 x=row['metrics'];row['qualification_pass']=bool(x['potential']['recall']>=.8 and x['verified']['recall']>=.8 and x['native']['precision']>=.9 and x['native']['recall']>=.8 and x['native']['localization_p90_m']<=.15);rows.append(row);print('RESULT',json.dumps(row),flush=True)
result=dict(rows=rows,qualification_pass=all(r['qualification_pass'] for r in rows),fit=0,physical=0,scope='GT oracle domain/registration qualification only; not learned prediction, not source admission',input_sha256=inputs,source_sha256={str(p):sha(p) for p in [D/'run_qualification.py',D/'projection.py',D/'protocol.json',L/'visible_target.py',L/'score.py']})
(D/'qualification.json').write_text(json.dumps(result,indent=2)+'\n');(D/'frame_coverage.json').write_text(json.dumps(details,indent=2)+'\n')
