"""Bounded existing-label attribution; never change original gate/predictions."""
from pathlib import Path
import json,hashlib,datetime,numpy as np
from cell_proxy_metrics import confusion
H=Path(__file__).resolve().parent;D=H/'frozen_head_paired_texture_domain_predictions_v1';O=H/'frozen_head_paired_texture_failure_layers_v1';O.mkdir(exist_ok=False);s=json.loads((D/'all_predictions_pretruth_sha256.json').read_text());sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
for g in ('inputs','outputs','source_sha256'):
 for f,x in s[g].items():assert sha(Path(f))==x,f
label=np.load(H/'fixed_appearance_fixture_v1/occupancy.npz');G=label['grid'];origin=label['global_origin_xy'];res=float(label['resolution_m']);Y=[];A=[];B=[];R=[];AGE=[]
for f in sorted((D/'frames').glob('*.npz')):
 d=np.load(f);xy=d['origin']+d['cells']*float(d['resolution']);ij=np.rint((xy-origin)/res).astype(int);assert ((ij>=0).all(1)&(ij<G.shape).all(1)).all();y=G[ij[:,0],ij[:,1]]>0;age=float(d['time_s'])-d['last_query_times'];baseline=d['baseline_history_labels']==1
 # Both retain the same .05<z<=1.5 endpoints. History presence is measured, not a learned prediction.
 np.testing.assert_array_equal(baseline,age<=10+1e-9)
 Y.append(y);A.append(d['scores']>=.5);B.append(baseline);R.append(np.linalg.norm(xy,axis=1));AGE.append(age)
y=np.concatenate(Y);a=np.concatenate(A);b=np.concatenate(B);r=np.concatenate(R);age=np.concatenate(AGE);groups={}
for name,m in [('last_support_age_le10',age<=10+1e-9),('last_support_age_gt10',age>10+1e-9)]:
 fp=m&a&~y;groups[name]=dict(eligible=int(m.sum()),model=confusion(a[m],y[m]),explicit10=confusion(b[m],y[m]),all_supported35_diagnostic=confusion(np.ones(m.sum(),bool),y[m]),model_FP_radius_p50_p90_max_m=None if not fp.any() else np.percentile(r[fp],[50,90,100]).tolist(),model_FP_outside_continuous_circle_but_within_half_pointcell=int((fp&(r>.35)&(r<=.35+np.sqrt(2)*.1/2)).sum()))
r=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),original_rejection_preserved=True,model=confusion(a,y),explicit10=confusion(b,y),same_supported35_diagnostic=confusion(np.ones(len(y),bool),y),groups=groups,pointcell_resolution_m=.1,proxy_label_grid_resolution_m=res,FP_spatial_context_is_diagnostic_not_relabel=True,all_supported35_is_not_new_candidate_or_online_model=True,retention_confound_not_isolated_weight_benefit=True,not_independent_frame_cells=True,no_refit_threshold_or_gate_change=True,whole_complete=False);(O/'assessment.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
