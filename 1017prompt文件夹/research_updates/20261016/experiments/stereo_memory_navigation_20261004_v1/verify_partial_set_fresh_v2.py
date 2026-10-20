"""After process close: exact causal reconstruction and frozen source-kind gates."""
from pathlib import Path
import json,hashlib,datetime
import numpy as np
from scipy.spatial import cKDTree
from partial_cue_source import PartialCueSetSource
from risk_grid import ExplorationGrid
from route_geometry_audit import hazard_clearance
H=Path(__file__).resolve().parent;D=H/'partial_set_fresh_run_v1';sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
seal=json.loads((D/'artifact_manifest_post_close.json').read_text());assert all(sha(f)==s for f,s in seal['artifact_sha256'].items())
actor=json.loads((H/'partial_set_uncertainty_actor_v1.json').read_text());model=PartialCueSetSource(actor['radius_m']);prefix=PartialCueSetSource(actor['radius_m']);op=json.loads((D/'source_opportunity_records.json').read_text());root0=np.load(D/'policy_inputs/frame_000.npz')['position_world_xy'];goal=np.array([2.2,0.]);truth=np.load(D/'offline_occupancy_label.npz');truthxy=truth['origin']+np.argwhere(truth['labels']>0)*float(truth['resolution']);rows=[];equal_fields=0;class_rows=[];traps=0;class_seen=set()
Y,X=np.mgrid[:480,:640];ray=np.stack([(X-319.5)/584.8888888888889,(Y-239.5)/584.8888888888889,np.ones_like(X)],axis=-1)
class Bomb(dict):
 def __getitem__(self,k):
  if k!='time_s':raise AssertionError('future payload touched')
  return super().__getitem__(k)
for i in range(0,81,5):
 p=dict(np.load(D/'sensor_packets'/f'source_{i:03d}.npz'));rgb=dict(np.load(D/'online_inputs'/f'frame_{i:03d}.npz'));poses=dict(np.load(D/'known_poses'/f'frame_{i:03d}.npz'));mask=dict(np.load(D/'masks'/f'frame_{i:03d}.npz'));root=np.load(D/'policy_inputs'/f'frame_{i:03d}.npz')['position_world_xy'];qualified=op[i//5]['qualified_seed'];saved=dict(np.load(D/'possible_sets'/f'frame_{i:03d}.npz'));re=model.update(p,rgb,poses,mask,qualified,i*.1,i,root,goal,root0)
 for k in saved:np.testing.assert_equal(re[k],saved[k]);equal_fields+=1
 if i<=30:
  pr=prefix.update(p,rgb,poses,mask,qualified,i*.1,i,root,goal,root0)
  for k in pr:np.testing.assert_equal(pr[k],re[k])
 try:model.update(Bomb(time_s=i*.1+.5),None,None,None,False,i*.1,i,None,None,None)
 except ValueError:traps+=1
 else:raise AssertionError('future source accepted')
 for (vf,eye),cl in model.class_masks.items():
  file=D/'partial_class_masks'/f'birth_{i:03d}_{eye}_{vf:03d}.npz';np.testing.assert_equal(cl,np.load(file)['potential_hazard_mask'])
  if (vf,eye) in class_seen:continue
  class_seen.add((vf,eye));source_mask=np.load(D/'masks'/f'frame_{vf:03d}.npz')[eye]
  gt=np.load(D/'offline_truth'/f'frame_{vf:03d}.npz');z=gt[eye+'_depth_m'];T=gt[eye+'_camera_usd_world']
  with np.errstate(invalid='ignore'):w=T[:3,3]+(ray*z[:,:,None])@(T[:3,:3]@np.diag([1,-1,-1])).T
  finite=np.isfinite(z)&np.isfinite(w).all(-1);haz=finite&(z>=.5)&(z<=3)&(w[:,:,2]>.05)&(w[:,:,2]<=1.5)&(np.linalg.norm(w-gt['root_position_world_m'],axis=-1)>.55)&~source_mask
  n=int(cl.sum());prec=float((cl&haz).sum()/n) if n else 0.;class_rows.append(dict(frame=vf,eye=eye,pixels=n,precision=prec,pass_ge95=prec>=.95))
 if re['pending_seed']:continue
 xy=re['possible_xy'];contain=float((cKDTree(xy).query(truthxy)[0]<=np.sqrt(2)*.05+1e-9).mean()) if len(xy) else 0.;area=len(xy)*.01
 grid=ExplorationGrid(root-20,shape=(400,400),resolution=.1);cells=grid.cells(xy);cells=cells[grid.inside(cells)];grid.labels[tuple(cells.T)]=1;route,why=grid.plan(root,goal);length=float(np.linalg.norm(np.diff(route,axis=0),axis=1).sum()) if route is not None else None;clear=float(hazard_clearance(route,np.array([[0.,0.]]))-.75) if route is not None else None
 checks=dict(nonempty=bool(len(xy)),containment_ge95=contain>=.95,area_le6=area<=6,route_le7=length is not None and length<=7,continuous_body_clearance_ge0=clear is not None and clear>=0)
 rows.append(dict(frame=i,containment=contain,area_m2=area,cells=len(xy),unexecuted_route_length_m=length,unexecuted_true_clearance_m=clear,route_reason=why,checks=checks))
assert model.birth_packet is not None
birth=dict(np.load(D/'immutable_possible_set_birth.npz'))
for k in birth:np.testing.assert_equal(birth[k],model.birth_packet[k])
requests=np.load(D/'episode_arrays.npz')['requests'] if 'requests' in np.load(D/'episode_arrays.npz').files else np.array([r['local']['request'] for r in json.loads((D/'policy_records.json').read_text())]);assert len(requests)==80 and np.all(requests==0)
clocks=json.loads((D/'camera_clocks.json').read_text());assert len(clocks)==81;exposure=max(abs(c['raw_time_s']-t) for c in clocks for t in c['eyes'].values());assert exposure<=1e-5
import cv2
videos={}
for name in ['stereo_view','third_person']:
 cap=cv2.VideoCapture(str(D/(name+'.mp4')));n=0
 while True:
  ok,im=cap.read()
  if not ok:break
  assert im is not None;n+=1
 cap.release();assert n==81;videos[name]=n
passed=bool(rows and class_rows) and all(all(r['checks'].values()) for r in rows) and all(r['pass_ge95'] for r in class_rows)
r=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),decision='PASS_LOCAL_SOURCE_HYPOTHESIS_PREREQUISITES_ONLY' if passed else 'REJECT_CALIBRATED_PARTIAL_SET_FRESH',rows=rows,class_rows=class_rows,exact_reconstructed_fields=equal_fields,prefix_source_epochs=7,future_payload_rejections=traps,immutable_birth_exact=True,requests_zero=80,exposure_error_max_s=exposure,decoded_video_frames=videos,renderer_GT_used_post_close_only=True,knownpose_privileged=True,not_observed_free_or_occupied=True,not_navigation_or_formal_success=True,old_source_and_class_completeness_REJECT_preserved=True,whole_research_complete=False,manifest_sha256=sha(D/'artifact_manifest_post_close.json'),script_sha256=sha(__file__))
(H/'partial_set_fresh_assessment_v2.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
