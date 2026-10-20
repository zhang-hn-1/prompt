"""Disjoint rejected-mask label ownership; no rescoring old gate or edits."""
from pathlib import Path
import numpy as np,json,datetime
H=Path(__file__).resolve().parent;D=H/'source_event_handoff_C1_run_v1';O=H/'qualified_appearance_later_motion_v1';a=json.loads((O/'assessment_v1.json').read_text());assert a['decision']=='REJECT_LATER_MOTION_APPEARANCE';yy,xx=np.mgrid[:480,:640];F=584.8888888888889;ray=np.stack([(xx-319.5)/F,(yy-239.5)/F,np.ones_like(xx)],-1);rows=[];totals={}
for r in a['all_historical_motion_rows']:
 if r['pass_row']:continue
 i=r['frame'];eye=r['eye'];p=np.load(O/f'{eye}_{i:03d}.npz')['potential_hazard_mask'];gt=np.load(D/'offline_truth'/f'frame_{i:03d}.npz');z=gt[eye+'_depth_m'];T=gt[eye+'_camera_usd_world']
 with np.errstate(invalid='ignore'):w=T[:3,3]+(ray*z[:,:,None])@(T[:3,:3]@np.diag([1,-1,-1])).T
 finite=np.isfinite(z)&np.isfinite(w).all(-1);near=np.linalg.norm(w-gt['root_position_world_m'],axis=-1)<=.55;height=(w[:,:,2]>.05)&(w[:,:,2]<=1.5);rangeok=(z>=.5)&(z<=3);remaining=p.copy();categories={}
 for name,condition in [('insufficient_GT',~finite),('within_root_055_not_body_identity_proof',near),('outside_height_domain',~height),('physical_hazard_outside_axial_domain',~rangeok),('correct_in_frozen_domain',np.ones(p.shape,bool))]:
  sel=remaining&condition;categories[name]=int(sel.sum());remaining[sel]=False;totals[name]=totals.get(name,0)+categories[name]
 assert sum(categories.values())==r['predicted_pixels'];assert categories['correct_in_frozen_domain']==r['correct_hazard_pixels'];rows.append(dict(frame=i,eye=eye,categories=categories,old_precision=r['precision'],old_coverage=r['coverage']))
result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),GT_posthoc_only=True,old_REJECT_preserved=True,no_domain_rescore_or_threshold_retry=True,category_order_disjoint=True,within_root_radius_does_not_identify_robot_prim=True,rows=rows,total_categories=totals,whole_research_complete=False)
(H/'appearance_later_motion_failure_layers_v1.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(totals,indent=2))
