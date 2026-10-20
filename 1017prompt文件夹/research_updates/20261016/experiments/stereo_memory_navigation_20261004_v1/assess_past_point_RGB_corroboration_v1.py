from pathlib import Path
import numpy as np,json,hashlib,datetime
H=Path(__file__).resolve().parent;O=H/'anchor_validation_fresh_run_v1';P=H/'past_point_RGB_corroboration_predictions_v1';seal=json.loads((P/'all_predictions_seal.json').read_text());sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
for manifest in [seal['input_sha256'],seal['prediction_sha256']]:
 for f,s in manifest.items():assert sha(f)==s
rows=[]
for frame in [185,190,240,350]:
 d=np.load(P/f'frame_{frame:03d}.npz');gt=np.load(O/'offline_truth'/f'frame_{frame:03d}.npz');sel=d['appearance_corroborated'];point=d['past_world'][sel];errors=[]
 for eye in ['left','right']:
  uv=d[eye+'_uv'][sel];xx,yy=np.rint(uv).astype(int).T;z=gt[eye+'_depth_m'][yy,xx];T=gt[eye+'_camera_usd_world'];R=T[:3,:3]@np.diag([1,-1,-1]);rays=np.c_[(xx-319.5)/584.8888888888889,(yy-239.5)/584.8888888888889,np.ones(len(xx))]
  with np.errstate(invalid='ignore'):actual=T[:3,3]+(rays*z[:,None])@R.T;errors.append(np.linalg.norm(actual-point,axis=1))
 e=np.array(errors).T;finite=np.isfinite(e).all(axis=1);correct=finite&np.all(e<=.1,axis=1);source=np.load(O/'sensor_packets'/f'source_{frame:03d}.npz');haz=(source['world'][:,2]>.05)&(source['world'][:,2]<=1.5);rows.append(dict(frame=frame,past_points=len(d['past_world']),bilateral_fov_and_mask_valid=int(d['bilateral_fov_and_mask_valid'].sum()),appearance_accepted=int(sel.sum()),both_eye_geometry_correct=int(correct.sum()),finite_geometry_mismatch=int((finite&~correct).sum()),label_insufficient=int((~finite).sum()),conservative_precision=float(correct.sum()/sel.sum()) if sel.sum() else None,current_frozenFast_hazard_points=int(haz.sum()),original_geometry_time_s=float(d['source_geometry_time_s']),appearance_time_s=float(d['current_appearance_time_s'])))
gates=dict(moving_frame190_nonzero_support=rows[1]['appearance_accepted']>0,accepted_precision_all_nonempty_ge95=all(r['conservative_precision']>=.95 for r in rows if r['appearance_accepted']))
result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),seal_utc=seal['utc'],rows=rows,gates=gates,decision='READONLY_APPEARANCE_DIAGNOSTIC_PASS_ONLY' if all(gates.values()) else 'REJECT',not_new_depth_measurement=True,no_geometry_age_refresh=True,not_memory_or_navigation_admission=True,knownpose_privileged=True,development_not_fresh=True)
(H/'past_point_RGB_corroboration_assessment_v1.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
