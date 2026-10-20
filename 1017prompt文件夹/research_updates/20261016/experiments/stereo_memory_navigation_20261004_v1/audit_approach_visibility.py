"""Bounded retrospective geometry diagnosis. Never called by runtime or policy."""
from pathlib import Path
import json,hashlib,datetime,re
import numpy as np
H=Path(__file__).resolve().parent
scene=Path('/home/an/project/NavDP/assets/scenes/benchmark_e1/e1_detour.usda')
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
cases=['fresh_rolling_prior_shadow_current_run','occluded_obstacle_memory_navigation_history_run']
frames=[60,100,140,180,220,260,295];inputs=[scene,H/'fast_stereo.py',H/'stereo_geometry.py',H/'third_party/fast_foundation_stereo/core/foundation_stereo.py',Path(__file__).resolve()]
for case in cases:
 for i in frames:
  for folder,name in [('offline_truth',f'frame_{i:03d}.npz'),('masks',f'frame_{i:03d}.npz'),('source_predictions',f'frame_{i:03d}.npz'),('sensor_packets',f'source_{i:03d}.npz')]:inputs.append(H/case/folder/name)
protocol=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),scope='exact fourteen historical exposures; no new inference/control',inputs={str(p):sha(p) for p in inputs},surface_definition='offline endpoint proxy within declared divider cylinder radius+.005m and height bounds; not renderer object ID',right_depth_tolerance_m=.02,old_gates_unchanged=True,truth_never_online=True)
(H/'approach_visibility_preanalysis.json').write_text(json.dumps(protocol,indent=2)+'\n')
blocks=re.findall(r'def Cylinder "divider_\d+".*?\n        \}',scene.read_text(),re.S);assert len(blocks)>0
centres=np.array([[float(x) for x in re.search(r'xformOp:translate = \(([^)]+)\)',b)[1].split(',')] for b in blocks]);radius=float(re.search(r'double radius = ([\d.]+)',blocks[0])[1]);height=float(re.search(r'double height = ([\d.]+)',blocks[0])[1])
f=584.8888888888889;cx=319.5;cy=239.5;yy,xx=np.indices((480,640));rays=np.stack([(xx-cx)/f,(yy-cy)/f,np.ones_like(xx)],axis=-1);C=np.diag([1,-1,-1]);out=[]
for case in cases:
 for i in frames:
  o=H/case;t=np.load(o/'offline_truth'/f'frame_{i:03d}.npz');mask=np.load(o/'masks'/f'frame_{i:03d}.npz');pred=np.load(o/'source_predictions'/f'frame_{i:03d}.npz');packet=np.load(o/'sensor_packets'/f'source_{i:03d}.npz');L=t['left_camera_usd_world'];R=t['right_camera_usd_world'];z=t['left_depth_m'].astype(float)
  with np.errstate(invalid='ignore'):
   xyz=L[:3,3]+(rays*z[...,None])@(L[:3,:3]@C).T
  finite=np.isfinite(xyz).all(-1);surface=np.zeros_like(finite)
  for c in centres:surface|=finite&(np.linalg.norm(xyz[...,:2]-c[:2],axis=-1)<=radius+.005)&(xyz[...,2]>=c[2]-height/2-.005)&(xyz[...,2]<=c[2]+height/2+.005)
  ij=np.argwhere(surface);p=xyz[surface];rz=(p-R[:3,3])@(R[:3,:3]@C)
  u=f*rz[:,0]/rz[:,2]+cx;v=f*rz[:,1]/rz[:,2]+cy
  inside=(rz[:,2]>0)&(u>=0)&(u<=639)&(v>=0)&(v<=479);ui=np.rint(np.clip(u,0,639)).astype(int);vi=np.rint(np.clip(v,0,479)).astype(int)
  lm=~mask['left'][surface];both=inside&lm&~mask['right'][vi,ui];rdepth=t['right_depth_m'][vi,ui];visible=both&np.isfinite(rdepth)&(np.abs(rdepth-rz[:,2])<=.02)
  disp=np.abs(v-ij[:,0]);conf=pred['confidence'][surface].astype(bool);pz=pred['depth_m'][surface];correct=conf&np.isfinite(pz)&(np.abs(pz-z[surface])*np.linalg.norm(rays[surface],axis=1)<=.1)
  pq=packet['query_pixel_id'].astype(int);psurf=surface.ravel()[pq];err=np.linalg.norm(packet['world']-xyz.reshape(-1,3)[pq],axis=1)
  out.append(dict(case=case,frame=i,time_s=float(t['time_s']),root=t['root_position_world_m'].tolist(),divider_left_visible_pixels=len(ij),left_unmasked=int(lm.sum()),right_inside=int(inside.sum()),both_unmasked=int(both.sum()),right_depth_consistent=int(visible.sum()),right_depth_consistent_and_nominal_search=int((visible&(disp<=416)).sum()),left_confident_surface=int(conf.sum()),left_confident_correct_10cm=int(correct.sum()),source_surface_queries=int(psurf.sum()),source_surface_correct_10cm=int((psurf&np.isfinite(err)&(err<=.1)).sum()),source_queries=len(pq),surface_axial_depth_p10_p50_p90=np.percentile(z[surface],[10,50,90]).tolist() if len(ij) else [],disparity_p10_p50_p90=np.percentile(disp,[10,50,90]).tolist() if len(ij) else [],rig_baseline_m=float(np.linalg.norm(L[:3,3]-R[:3,3]))))
result=dict(protocol=protocol,divider_count=len(centres),divider_centres=centres.tolist(),radius_m=radius,nominal_search_depth_m=f*.6/416,full_native_overlap_necessary_axial_depth_m=f*.6/479,maxdisp_is_costvolume_not_hard_output_clip=True,physical_overlap_depends_on_pose_and_occlusion=True,rows=out,whole_research_complete=False)
(H/'approach_visibility_diagnostic.json').write_text(json.dumps(result,indent=2)+'\n')
for r in out:print(r['case'],r['frame'],{k:r[k] for k in ['divider_left_visible_pixels','left_unmasked','both_unmasked','right_depth_consistent','right_depth_consistent_and_nominal_search','left_confident_correct_10cm','source_surface_queries','source_surface_correct_10cm']})
