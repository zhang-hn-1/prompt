"""Read-only historical intersection diagnosis; never filters or replaces sources."""
from pathlib import Path
import json,hashlib
import numpy as np
from scipy.optimize import linprog
H=Path(__file__).resolve().parent; O=H/'anchor_evidence_development_v1';D=H/'minimum_prefix_transition_navigation_run_v1';groups=[]
for packet in json.loads((O/'predictions.json').read_text()):
 f=packet['frame'];gt=np.load(D/'frames'/f'frame_{f:03d}.npz');T=gt['left_camera_usd_world'];byq={}
 for e in packet['geometries']:
  if e['evidence']['eligible_anchor']:
   r=e['original'];byq.setdefault(tuple(r['query_xy']),[]).append(r)
 for q,rs in byq.items():
  if len(rs)<2:continue
  A=np.concatenate([np.asarray(r['constraint_A']) for r in rs]); b=np.concatenate([np.asarray(r['constraint_b']) for r in rs]);fit=linprog(np.zeros(3),A_ub=A,b_ub=b,bounds=[(None,None)]*3,method='highs');x,y=np.rint(q).astype(int);z=float(gt['left_depth_m'][y,x]);valid=bool(np.isfinite(z) and z>0);inside=None;single_inside=[]
  if valid:
   w=T[:3,3]+(T[:3,:3]@np.diag([1,-1,-1]))@(np.r_[(np.asarray(q)-[319.5,239.5])/584.8888888888889,1]*z);inside=bool(np.all(A@w<=b+1e-7));single_inside=[bool(r['status']=='bounded' and np.all(np.asarray(r['constraint_A'])@w<=np.asarray(r['constraint_b'])+1e-7)) for r in rs]
  groups.append(dict(frame=f,query_xy=list(q),past_frames=[r['source_frame'] for r in rs],pair_count=len(rs),solver_status=int(fit.status),solver_feasible_not_certified=bool(fit.success),GT_valid=valid,GT_inside_all_fixed_cones=inside,single_pair_GT_contained=single_inside))
summary=dict(groups_ge2past=len(groups),GTvalid_groups=sum(r['GT_valid'] for r in groups),solver_feasible_groups=sum(r['solver_feasible_not_certified'] for r in groups),GTvalid_feasible_groups=sum(r['GT_valid'] and r['solver_feasible_not_certified'] for r in groups),GTvalid_feasible_but_outside_groups=sum(r['GT_valid'] and r['solver_feasible_not_certified'] and not r['GT_inside_all_fixed_cones'] for r in groups),GTvalid_all_fixed_cones_contain=sum(r['GT_inside_all_fixed_cones'] is True for r in groups))
(O/'multi_past_consistency_diagnosis_v1.json').write_text(json.dumps(dict(summary=summary,groups=groups,historical_readonly_diagnosis=True,not_candidate_or_admission=True,HiGHS_raw_feasibility_not_geometric_certificate=True,teacher_after_source_seal=True,no_changes_to_original_predictions=True,code_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()),indent=2)+'\n');print(json.dumps(summary,indent=2))
