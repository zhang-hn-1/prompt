"""Independent current FK and renderer-instance verification, GT only offline."""
from pathlib import Path
import json,datetime,hashlib,numpy as np,cv2
from scipy.spatial.transform import Rotation
from dynamic_self_mask import RobotSelfMask,joints_from_proprio
H=Path(__file__).resolve().parent;O=H/'dynamic_mask_probe_run_v2';p=json.loads((H/'dynamic_mask_probe_protocol_v2.json').read_text());st=json.loads((H/'dynamic_mask_probe_status_v2.json').read_text());assert st['status']=='finished' and st['exit_code']==0;sha=lambda f:hashlib.sha256(Path(f).read_bytes()).hexdigest();n=0
for f,s in p['source_sha256'].items():assert sha(f)==s,f;n+=1
for f,s in json.loads((O/'artifact_manifest_post_close.json').read_text())['artifact_sha256'].items():assert sha(f)==s,f;n+=1
cad=RobotSelfMask();rows=[];all_ok=True;out=H/'dynamic_mask_probe_assessment';out.mkdir(exist_ok=False)
def err(A,B):return dict(position_m=float(np.linalg.norm(A[:3,3]-B[:3,3])),rotation_deg=float(Rotation.from_matrix(A[:3,:3].T@B[:3,:3]).magnitude()*180/np.pi))
for i in p['validation_frames']:
 d=np.load(O/'self_validation'/f'frame_{i:03d}.npz');meta=json.loads((O/'self_validation'/f'frame_{i:03d}.json').read_text());assert meta['joint_names']==list(cad.joint_names);q=joints_from_proprio(d['proprio_history']);np.testing.assert_equal(q,d['joint_position_rad']);fk=cad.body_transforms(q);base=d['base_link_usd_world'];measured={name:np.linalg.inv(base)@d[name+'_usd_world'] for name in fk};errors={name:err(fk[name],measured[name]) for name in fk};m=cad.project(q);camera={};views={};admitted=all(e['position_m']<=p['FK_position_gate_m'] and e['rotation_deg']<=p['FK_rotation_gate_deg'] for e in errors.values())
 for eye,y in [('left',.3),('right',-.3)]:
  C=np.eye(4);C[:3,:3]=np.array([[0,0,-1],[0,1,0],[1,0,0]]);C[:3,3]=[.2,y,.05];camera[eye]=err(C,np.linalg.inv(base)@d[eye+'_camera_usd_world']);admitted&=camera[eye]['position_m']<=.001 and camera[eye]['rotation_deg']<=.1
  mapping=meta['segmentation'][eye].get('info',{}).get('idToLabels',{});ids=[int(k) for k,v in mapping.items() if '/World/BallRobot/' in str(v)];available=bool(ids) and eye+'_instance_ids' in d
  if available:
   raw=np.squeeze(d[eye+'_instance_ids']);assert raw.shape==(480,640);own=np.isin(raw,ids);num=int(own.sum());coverage=float((m[eye]&own).sum()/num) if num else None;extra=m[eye]&~cad.frozen[eye];false=float((extra&~own).mean());views[eye]=dict(mapping_available=True,actual_self_pixels=num,predicted_mask_pixels=int(m[eye].sum()),self_coverage=coverage,newmask_nonself_fullimage_fraction=false,old_base_coverage=float((cad.frozen[eye]&own).sum()/num) if num else None);admitted&=coverage is not None and coverage>=p['self_occlusion_coverage_gate'] and false<=p['extra_mask_nonself_fullimage_fraction_gate'];np.savez_compressed(out/f'{i:03d}_{eye}.npz',CAD_mask=m[eye],frozen_base_mask=cad.frozen[eye],renderer_self_mask=own,extra_mask=extra)
  else:views[eye]=dict(mapping_available=False);admitted=False
 row=dict(frame=i,time_s=float(d['time_s']),current_measured_joints=q.tolist(),FK_errors=errors,camera_rig_errors=camera,views=views,local_geometry_pass=bool(admitted));rows.append(row);all_ok&=admitted
ph=np.load(O/'physics.npz');states=json.loads((O/'states.json').read_text());clocks=json.loads((O/'camera_clocks.json').read_text());actions=json.loads((O/'policy_records.json').read_text());assert len(actions)==120 and len(states)==len(clocks)==121 and len(ph['simulation_time_s'])==601
for i,r in enumerate(actions):
 t=i*.1;expected=np.tile([0. if t<3 or t>=9 else .3,0. if t<3 or t>=9 else (.15 if t<6 else -.15)],5).astype(np.float32);np.testing.assert_equal(r['request'],expected)
video=[]
for name in ('stereo_view','third_person'):
 c=cv2.VideoCapture(str(O/(name+'.mp4')));k=0
 while True:
  ok,_=c.read()
  if not ok:break
  k+=1
 fps=float(c.get(cv2.CAP_PROP_FPS));c.release();assert k==121 and fps==10;video.append(dict(name=name,frames=k,fps=fps))
contacts=sum(bool(x['contact']) for x in json.loads((O/'physics_contacts.json').read_text()));result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),rows=rows,candidate_selfmask_geometry_pass=bool(all_ok),source_gate_not_reassessed=True,navigation_actions_not_changed=True,diagnostic_predeclared_movement_only=True,identity_checks=n,physics_samples=601,RGB_video_frames=121,validation_records=4,videos=video,model_inferences=0,physical_contacts=contacts,max_render_clock_delta_s=max(abs(x['raw_time_s']-v) for x in clocks for v in x['eyes'].values()),previous_v1_failure_zero_s_retained=True,whole_research_complete=False)
(H/'dynamic_mask_probe_assessment.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
