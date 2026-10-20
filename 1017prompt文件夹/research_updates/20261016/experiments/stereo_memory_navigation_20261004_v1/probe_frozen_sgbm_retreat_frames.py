"""Four-exposure old frozen matcher diagnostic; prediction saved before truth."""
from pathlib import Path
import numpy as np,json,hashlib,datetime
from stereo_geometry import SGBMStereo,restore_native
from dynamic_self_mask import RobotSelfMask
from masked_source_pair import paired_sources
H=Path(__file__).resolve().parent;o=H/'retreat_observation_run';out=H/'retreat_frozen_sgbm_probe';out.mkdir(exist_ok=False);frames=[0,60,120,180];maskp=H.parent/'visible_surface_connectivity_20261004_v1/base_link_masks.npz';inputs=[Path(__file__).resolve(),H/'stereo_geometry.py',H/'masked_source_pair.py',H/'dynamic_self_mask.py',maskp,o/'states.json'];sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
for i in frames:
 for folder in ['online_inputs','known_poses','history','masks','offline_truth']:inputs.append(o/folder/f'frame_{i:03d}.npz')
protocol=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),input_sha256={str(p):sha(p) for p in inputs},scope='exact four existing actual retreat exposures; frozen existing SGBM448/block5/LR1.5; no fit/new network/parameter scan',truth_after_all_predictions_sealed=True,old_SGBM_gate_unchanged=True,new_navigation_not_run=True)
(out/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n');base=np.load(maskp);states=json.loads((o/'states.json').read_text());model=SGBMStereo();cad=RobotSelfMask();packets={}
for i in frames:
 rgb=np.load(o/'online_inputs'/f'frame_{i:03d}.npz');pose=np.load(o/'known_poses'/f'frame_{i:03d}.npz');hist=np.load(o/'history'/f'frame_{i:03d}.npz');pred=model.predict(rgb['left_rgb'],rgb['right_rgb'],base['left'],base['right']);t=float(rgb['time_s']);np.savez_compressed(out/f'prediction_{i:03d}.npz',**pred,frame=i,time_s=t)
 known=dict(left_camera_usd_world=pose['left_camera_usd_world'],right_camera_usd_world=pose['right_camera_usd_world'],root_position_world_m=states[i]['position'],time_s=t);raw=dict(depth_m=pred['depth_m'],confidence=pred['confidence'],disparity_native_px=restore_native(pred['disparity_px']),frame=i,time_s=t);_,packet,_=paired_sources(raw,known,hist['proprio_history'],cad,t);np.savez_compressed(out/f'packet_{i:03d}.npz',**packet);packets[i]=packet
seal={str(p):sha(p) for p in out.glob('*.npz')};(out/'predictions_pretruth_sha256.json').write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),sha256=seal),indent=2)+'\n')
rows=[]
for i in frames:
 s=packets[i];truth=np.load(o/'offline_truth'/f'frame_{i:03d}.npz');q=s['query_pixel_id'].astype(int);yy=q//640;xx=q%640;z=truth['left_depth_m'][yy,xx];T=truth['left_camera_usd_world'];rays=np.c_[(xx-319.5)/584.8888888888889,(yy-239.5)/584.8888888888889,np.ones(len(q))]
 with np.errstate(invalid='ignore'):actual=T[:3,3]+(rays*z[:,None])@(T[:3,:3]@np.diag([1,-1,-1])).T;err=np.linalg.norm(s['world']-actual,axis=1)
 rows.append(dict(frame=i,queries=len(q),finite_truth=int(np.isfinite(err).sum()),correct_10cm=int((np.isfinite(err)&(err<=.1)).sum()),error_p50_p90=np.percentile(err[np.isfinite(err)],[50,90]).tolist() if np.isfinite(err).any() else [],unknown_not_free=True))
for p,h in seal.items():assert sha(Path(p))==h
(out/'assessment.json').write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),rows=rows,no_online_matcher_substitution=True,old_rejection_not_rescored=True,whole_research_complete=False),indent=2)+'\n');print(json.dumps(rows))
