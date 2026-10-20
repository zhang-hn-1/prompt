from pathlib import Path
import json,hashlib,numpy as np,sys
from scipy.spatial.transform import Rotation
H=Path(__file__).resolve().parent.parent;sys.path.insert(0,str(H));D=H/'RGB_memory_frozen_pose_stress_v1';O=H/'online_RGB_memory_fresh_run_v1';OUT=Path(__file__).parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
checks={}; mismatches=[];counts={}
for rel,keys in [('protocol.json',['source_sha256']),('prediction_completion.json',['output_sha256','input_sha256']),('getter_contract_source_identity_v1.json',['source_sha256']),('oracle_component_audit_v1/completion.json',['inputs_sha256','outputs_sha256']),('projection_observability_v1/source_only_result.json',['input_sha256'])]:
 j=json.loads((D/rel).read_text());c=0
 for key in keys:
  for p,s in j[key].items():
   c+=1
   if sha(p)!=s:mismatches.append([rel,key,p])
 counts[rel]=c
checks['manifest_checked_entries']=counts;checks['manifest_mismatches']=mismatches
S=np.load(D/'ideal_sensor_rows.npz');p=np.load(O/'physics.npz');actual=Rotation.from_quat(p['root_quaternion_world_wxyz'][:,[1,2,3,0]]).as_matrix();np.testing.assert_allclose(S['gyro_body_rad_s'],np.einsum('nji,nj->ni',actual,p['angular_velocity_world_rad_s']),rtol=0,atol=0)
A=np.load(D/'all_pose_samples.npz');delta=A['position']-p['root_position_world_m'][::5];angles=np.rad2deg(Rotation.from_matrix(actual[::5].transpose(0,2,1)@A['rotation']).magnitude());checks['pose_metrics']=dict(XY_RMS=float(np.sqrt(np.mean(np.sum(delta[:,:2]**2,axis=1)))),XY_peak=float(np.linalg.norm(delta[:,:2],axis=1).max()),Z_RMS=float(np.sqrt(np.mean(delta[:,2]**2))),RMS3D=float(np.sqrt(np.mean(np.sum(delta**2,axis=1)))),attitude_peak=float(angles.max()),event_position=delta[190].tolist(),event_attitude=float(angles[190]))
assess=json.loads((D/'assessment_v1.json').read_text()); rows=[]
for r in assess['rows']:
 i=r['frame'];z=np.load(D/'memory_predictions'/f'frame_{i:03d}.npz');sel=z['appearance_corroborated'];w=z['past_world'][sel];n=len(w);assert n==r['support']; bins=len(np.unique(np.floor(w[:,:2]/.1).astype(int),axis=0));assert bins==r['unique_10cm_XY_bins']
 if not n:continue
 gt=np.load(O/'offline_truth'/f'frame_{i:03d}.npz'); errs=[];axs=[]
 for eye in ['left','right']:
  x,y=np.rint(z[eye+'_uv'][sel]).astype(int).T;depth=gt[eye+'_depth_m'][y,x];T=gt[eye+'_camera_usd_world'];R=T[:3,:3]@np.diag([1,-1,-1]);rays=np.column_stack(((x-319.5)/584.8888888888889,(y-239.5)/584.8888888888889,np.ones(n)))
  with np.errstate(invalid='ignore'):world=T[:3,3]+(rays*depth[:,None])@R.T
  errs.append(np.linalg.norm(world-w,axis=1));axs.append(np.abs(depth-z[eye+'_z'][sel]))
 e=np.array(errs).T;a=np.array(axs).T;finite=np.isfinite(e).all(axis=1);correct=finite&(e<=.1).all(axis=1);ax=np.isfinite(a).all(axis=1)&(a<=.1).all(axis=1)
 assert int(correct.sum())==r['bilateral_absolute_world_correct'];assert int((~finite).sum())==r['label_insufficient'];assert int(ax.sum())==r['bilateral_axial_range_correct'];rows.append(dict(frame=i,n=n,bins=bins,correct=int(correct.sum()),label_insufficient=int((~finite).sum())))
checks['geometry_rows']=rows;checks['decision']=assess['decision'];assert assess['decision']=='REJECT_FROZEN_POSE_CONSUMER_DOMAIN'
ref=np.load(O/'rgb_memory_predictions/frame_190.npz');ids=ref['past_query_id'];xy=ref['past_world'][:,:2];common=json.loads((D/'oracle_component_audit_v1/common_frame_coverage_v1.json').read_text());checks['common_coverage']={}
for name,path in [('known',O/'rgb_memory_predictions/frame_190.npz'),('estimated',D/'memory_predictions/frame_190.npz')]+[(name,D/'oracle_component_audit_v1'/name/'frame_190.npz') for name in ['true_position','true_orientation','full_true_pose']]:
 z=np.load(path);sel=z['appearance_corroborated'];q=z['past_query_id'][sel];idx={int(k):i for i,k in enumerate(ids)};ww=np.array([xy[idx[int(k)]] for k in q]);bins=len(np.unique(np.floor(ww/.1).astype(int),axis=0));span=np.ptp(ww,axis=0);r=common['rows'][name];assert bins==r['same_known_source_XY_bins'];np.testing.assert_allclose(span,r['same_known_XY_span_m'],rtol=0,atol=0);checks['common_coverage'][name]=dict(n=len(q),bins=bins,span=span.tolist(),known_id_intersection=len(set(q)&set(ref['past_query_id'][ref['appearance_corroborated']])))
from rolling_odometry import exponential
rot=np.load(D/'gyro_interval_contract_v1/all_rotations_before_labels.npz');gyroresult=json.loads((D/'gyro_interval_contract_v1/assessment_v1.json').read_text());checks['gyro']={}
for name in ['previous_held','endpoint','midpoint']:
 R=S['initial_rotation_world'].copy();rs=[R.copy()]
 for i in range(1,len(S['time_s'])):
  g=S['gyro_body_rad_s'];w=g[i-1] if name=='previous_held' else g[i] if name=='endpoint' else .5*(g[i-1]+g[i]);R=R@exponential(w*(S['time_s'][i]-S['time_s'][i-1]));rs.append(R.copy())
 np.testing.assert_array_equal(np.array(rs),rot[name]);a=np.rad2deg(Rotation.from_matrix(actual.transpose(0,2,1)@rot[name]).magnitude());checks['gyro'][name]=dict(RMS=float(np.sqrt(np.mean(a*a))),peak=float(a.max()),event=float(a[950]));assert abs(checks['gyro'][name]['RMS']-gyroresult['results'][name]['integrated_RMS_deg'])<1e-12
pr=json.loads((D/'projection_observability_v1/source_only_result.json').read_text());checks['projection']={}
for r in pr['rows']:
 J=np.load(D/'projection_observability_v1'/f"frame_{r['frame']:03d}_jacobian.npz")['J'];ss=np.linalg.svd(J,compute_uv=False);rotss=np.linalg.svd(J[:,3:],compute_uv=False);std=np.sqrt(np.diag(np.linalg.inv(J.T@J)));norm=J/np.linalg.norm(J,axis=0);cross=norm[:,:3].T@norm[:,3:];np.testing.assert_allclose(std,r['full6DOF']['optimistic_independent_1pixel_std'],rtol=1e-8,atol=1e-10);checks['projection'][r['frame']]=dict(full_condition=float(ss[0]/ss[-1]),rotation_condition=float(rotss[0]/rotss[-1]),std=std.tolist(),coupling=float(np.abs(cross).max()))
(OUT/'checks.json').write_text(json.dumps(checks,indent=2)+'\n');print(json.dumps(checks,indent=2))
