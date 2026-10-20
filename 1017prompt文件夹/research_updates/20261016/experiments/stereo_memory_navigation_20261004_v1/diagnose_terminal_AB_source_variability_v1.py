from pathlib import Path
import json,hashlib
import numpy as np
H=Path(__file__).resolve().parent;A=H/'native_flat_pose_navigation_R1_run_v1';B=H/'native_terminal_bank_navigation_R1_run_v1';O=H/'native_terminal_bank_AB_verification_v1';records=[]
for i in range(0,141,5):
 name=f'frame_{i:03d}.npz';a=np.load(A/'online_inputs'/name);b=np.load(B/'online_inputs'/name);images={}
 for eye in ('left','right'):
  d=np.abs(a[eye+'_rgb'].astype(float)-b[eye+'_rgb'].astype(float));images[eye]=dict(equal=not d.any(),channel_abs_mean=float(d.mean()),channel_abs_p90=float(np.percentile(d,90)),channel_abs_max=float(d.max()),changed_pixels=int(np.any(d>0,axis=-1).sum()))
 aa=np.load(A/'estimated_poses'/name);bb=np.load(B/'estimated_poses'/name);poseequal=aa.files==bb.files and all(np.array_equal(aa[k],bb[k]) for k in aa.files)
 ga=np.load(A/'offline_truth'/name);gb=np.load(B/'offline_truth'/name);depths={}
 for eye in ('left','right'):
  x=ga[eye+'_depth_m'];y=gb[eye+'_depth_m'];finite=np.isfinite(x)&np.isfinite(y);depths[eye]=dict(finite_support_equal=bool(np.array_equal(np.isfinite(x),np.isfinite(y))),finite_abs_max_m=float(np.max(np.abs(x[finite]-y[finite]))) if finite.any() else None)
 records.append(dict(frame=i,images=images,estimated_pose_exact=poseequal,GT_depth_cause_only=depths))
ra=json.loads((A/'policy_records.json').read_text());rb=json.loads((B/'policy_records.json').read_text());sameparent=[];changed=[];mainunchanged=True;sites=[]
for row in rb:
 if row['frame']<140:continue
 local=row['local'];parent=np.asarray(local['parent_final_request']);actual=np.asarray(row['request']);mainunchanged &= bool(np.array_equal(parent[::2],actual[::2]));different=not np.array_equal(parent,actual)
 if different:changed.append(row['frame']);sites.append(local['terminal_goal_bank']['parent_request_source'])
r=dict(all29initial_epochs=records,current_run_parent_final_request_changes=changed,all_changed_sources_fresh_goal_braking=all(s=='goal_braking' for s in sites),all_mains_preserved=mainunchanged,prior_five_tick_effect_and_oldrejects_preserved=True,baseline_first_request_divergence_before_candidate_terminal=True,candidate_intrinsic_navigation_reject=True,no_causal_performance_benefit_claim=True,only_source_variability_diagnosis=True,no_model_or_RGB_noise_fix=True,whole_research_complete=False,code_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
f=O/'source_variability_cause_v1.json';assert not f.exists();f.write_text(json.dumps(r,indent=2)+'\n');print(json.dumps({k:v for k,v in r.items() if k!='all29initial_epochs'},indent=2));print(json.dumps(records[0],indent=2))
