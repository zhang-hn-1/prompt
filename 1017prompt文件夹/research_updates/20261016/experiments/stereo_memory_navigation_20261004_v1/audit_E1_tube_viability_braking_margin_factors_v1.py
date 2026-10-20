"""One sealed two-factor map/state audit, not new actor requests or future prediction."""
from pathlib import Path
import json,hashlib,numpy as np
from scipy.ndimage import distance_transform_edt
from static_sparse_point_risk_grid_v1 import StaticSparsePointRiskGrid
H=Path(__file__).resolve().parent;D=H/'original_E1_tube_viability_navigation_run_v1';O=H/'E1_tube_viability_braking_margin_factors_v1';O.mkdir(exist_ok=False)
rows=json.loads((D/'policy_records.json').read_text());states=json.loads((D/'states.json').read_text());z=np.load(D/'physics.npz')
for p,s in json.loads((D/'artifact_manifest_post_close.json').read_text())['artifact_sha256'].items():assert hashlib.sha256(Path(p).read_bytes()).hexdigest()==s
pairs=[(265,270),(270,275),(275,280)];results=[]
for lo,hi in pairs:
 values=[];maps={}
 for mf in [lo,hi]:
  q=np.load(D/'maps'/f'frame_{mf:03d}.npz');g=StaticSparsePointRiskGrid(q['origin'],10,static_scene_assumption=True);g.labels=q['labels'];c=distance_transform_edt(np.pad(g.labels!=1,1))[1:-1,1:-1]*g.resolution;maps[mf]=(g,c)
  for pf in [lo,hi]:
   cell=g.cells(np.asarray(states[pf]['position'][:2])[None])[0];v=float(c[tuple(cell)]);values.append(dict(map_frame=mf,position_frame=pf,start_clearance_m=v,passes_existing_planning_margin=v>=.8707106781186548-1e-8,not_online_counterfactual=True))
 oldg=maps[lo][0];newg=maps[hi][0];new=np.argwhere((newg.labels==1)&(oldg.labels!=1));delta=np.asarray(states[hi]['position'][:2])-np.asarray(states[lo]['position'][:2]);results.append(dict(lo=lo,hi=hi,four_factor_values=values,new_observed_hazard_cells=len(new),actual_XY_displacement_m=float(np.linalg.norm(delta)),actual_request_held_zero=all(np.array_equal(r['request'],np.zeros(10)) for r in rows[lo:hi+1]),actual_pending_reverse=all(r['tracker_state_after_act']['pending_direction']=='reverse' for r in rows[lo:hi+1])))
a=dict(decision='FACTUAL_MAP_ARRIVAL_VERSUS_MEASURED_COAST_MARGIN_FACTORS_ONLY',pairs=results,no_new_physics_or_fit=True,no_free_cells_or_threshold_change=True,no_safe_recovery_claim=True,no_alternate_plan_physical_truth=True,whole_research_complete=False)
(O/'assessment.json').write_text(json.dumps(a,indent=2)+'\n');print(json.dumps(a,indent=2))
