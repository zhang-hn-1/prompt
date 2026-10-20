"""Postsealed bounded pose/source attribution, no candidate changes or gate replacement."""
from pathlib import Path
import json,hashlib,numpy as np,cv2
H=Path(__file__).resolve().parent;D=H/'paired_current_A2_run_v1';rows=[]
for f in [30,35,40]:
 p=np.load(D/'sensor_packets'/f'source_{f:03d}.npz');e=np.load(D/'current_evidence'/f'evidence_{f:03d}.npz')['eligible'];g=np.load(D/'offline_truth'/f'frame_{f:03d}.npz');est=np.load(D/'estimated_poses'/f'frame_{f:03d}.npz');ids=p['query_pixel_id'];x=ids%640;y=ids//640;z=g['left_depth_m'][y,x];valid=np.isfinite(z)&(z>0);ray=np.column_stack(((x-319.5)/584.8888888888889,(y-239.5)/584.8888888888889,np.ones(len(x))));T=g['left_camera_usd_world'];E=est['left_camera_usd_world'];truth=(T[:3,:3]@np.diag([1,-1,-1])@(ray[valid]*z[valid,None]).T).T+T[:3,3];pred=p['world'][valid];local=(E[:3,:3].T@(pred-E[:3,3]).T).T;cf=(T[:3,:3]@local.T).T+T[:3,3];error=np.linalg.norm(truth-pred,axis=1);cferror=np.linalg.norm(truth-cf,axis=1);bad=e[valid]&(error>.1)
 rows.append(dict(frame=f,eligible_errors10cm=int(bad.sum()),errors_after_GTpose_only=int((e[valid]&(cferror>.1)).sum()),pose_translation_m=float(np.linalg.norm(T[:3,3]-E[:3,3])),wrong_error_median=float(np.median(error[bad])) if bad.any() else None,wrong_error_peak=float(error[bad].max()) if bad.any() else None,wrong_pred_height_range=[float(pred[bad,2].min()),float(pred[bad,2].max())] if bad.any() else None,wrong_true_height_range=[float(truth[bad,2].min()),float(truth[bad,2].max())] if bad.any() else None,wrong_pred_range_median=float(np.median(np.linalg.norm(pred[bad]-est['root_position_world_m'],axis=1))) if bad.any() else None))
 if f==35:
  rgb=np.load(D/'online_inputs'/f'frame_{f:03d}.npz');print('rgbkeys',rgb.files)
  im=rgb['left_rgb'].copy();vi=np.flatnonzero(valid)[bad]
  for k in vi:cv2.circle(im,(int(x[k]),int(y[k])),3,(255,0,0),1)
  cv2.imwrite(str(D/'failed_epoch035_left_errors.png'),cv2.cvtColor(im,cv2.COLOR_RGB2BGR))
  np.savez_compressed(D/'failed_epoch035_error_points.npz',query_pixel_id=ids[valid][bad],predicted_world=pred[bad],truth_world=truth[bad],GTpose_counterfactual_world=cf[bad],errors_m=error[bad])
pol=json.loads((D/'policy_records.json').read_text());st=json.loads((D/'states.json').read_text());cs=json.loads((D/'camera_clocks.json').read_text());pcs=json.loads((D/'physics_contacts.json').read_text());ph=np.load(D/'physics.npz');flags=[]
for f in range(0,41,5):
 e=np.load(D/'current_evidence'/f'evidence_{f:03d}.npz');flags.append(bool(e['raw_source_unchanged']) and bool(e['physical_state_unchanged']))
r=dict(scope='POSTSEAL_DIAGNOSTIC_ONLY',rows=rows,all9_raw_and_physical_readonly=all(flags),requests=len(pol),captures=len(cs),physics_ticks=len(ph['raw_command_target']),all40_requests_zero=all(np.all(np.asarray(p['local']['request'])==0) for p in pol),all201_raw_targets_zero=bool(np.all(ph['raw_command_target']==0)),contacts=sum(bool(c['contact']) for c in pcs),available_contact_samples=sum(bool(c['available']) for c in pcs),max_XY_displacement_m=float(np.linalg.norm(np.array([s['position'][:2] for s in st])-np.array(st[0]['position'][:2]),axis=1).max()),video_verification={'stereo_view':[41,10.],'third_person':[41,10.]},source_gate_reject_kept=True,no_requalification_or_model_or_actor=True,code_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
(D/'fresh_failure_layer_diagnosis_v1.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
