"""Postseal failure attribution, no predictor/gate/configuration change."""
from pathlib import Path
import numpy as np,json,datetime
H=Path(__file__).resolve().parent;D=H/'mvs_hero_temporal_development_v2_api_scale_recovery';O=H/'qualified_hazard_navigation_run_v1';rows=[]
for r in json.loads((D/'records.json').read_text()):
 f=np.load(D/r['file']);w=f['world'];errors=[];finite=[];truth_depth=[]
 for frame,uv in zip([r['frame'],*f['source_frames']],[f['current_uv'],*f['past_uv']]):
  t=np.load(O/'offline_truth'/f'frame_{frame:03d}.npz');x,y=np.rint(uv).astype(int).T;z=t[r['eye']+'_depth_m'][y,x];T=t[r['eye']+'_camera_usd_world'];ray=np.c_[(x-319.5)/584.8888888888889,(y-239.5)/584.8888888888889,np.ones(len(x))]
  with np.errstate(invalid='ignore'):truth=T[:3,3]+(ray*z[:,None])@(T[:3,:3]@np.diag([1,-1,-1])).T;e=np.linalg.norm(truth-w,axis=1)
  errors.append(e);finite.append(np.isfinite(e));truth_depth.append(z)
 e=np.stack(errors,axis=1);fin=np.stack(finite,axis=1);currentok=fin[:,0]&(e[:,0]<=.1);allfinite=fin.all(1);allok=allfinite&(e<=.1).all(1);curwrong=fin[:,0]&~currentok;pastonlywrong=allfinite&currentok&~allok;insuff=~allfinite
 ed=e[:,0][fin[:,0]];td=truth_depth[0];bias=f['selected_depth_m'][fin[:,0]]-td[fin[:,0]]
 rows.append(dict(frame=r['frame'],eye=r['eye'],n=len(w),current_geometry_correct=int(currentok.sum()),current_finite_wrong=int(curwrong.sum()),current_insufficient=int((~fin[:,0]).sum()),current_precision_with_insuff=float(currentok.mean()),allfinite_past_only_wrong=int(pastonlywrong.sum()),allfinite_current_wrong=int((allfinite&curwrong).sum()),allfinite_threeendpoint_correct=int(allok.sum()),insuff_only_past=int((insuff&fin[:,0]).sum()),insuff_current=int((~fin[:,0]).sum()),current_world_error_median=float(np.median(ed)),current_world_error_p90=float(np.quantile(ed,.9)),current_axial_depth_bias_median=float(np.median(bias)),current_gt_depth_ge3=int((np.isfinite(td)&(td>3)).sum()),current_gt_depth_le05=int((np.isfinite(td)&(td<.5)).sum()),GT_only_postseal_diagnostic=True))
p=D/'endpoint_error_attribution_v1.json';assert not p.exists();p.write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),rows=rows,old_three_endpoint_gate_and_REJECT_preserved=True,no_current_only_admission=True,no_predictor_or_threshold_change=True),indent=2)+'\n');print(rows)
