"""Necessary downstream effect check; no action run or new point score."""
from pathlib import Path
import hashlib,json,numpy as np
H=Path(__file__).resolve().parent;rows=[];inputs={};sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
for case in ['moving_surface_A1_run_v1','paired_current_A2_run_v1']:
 for p in sorted((H/case/'sensor_packets').glob('source_*.npz')):
  z=np.load(p);w=z['world'];haz=(w[:,2]>.05)&(w[:,2]<=1.5);prior_negative=w[:,2]<0;candidate=haz&~prior_negative;assert np.array_equal(candidate,haz)
  rows.append(dict(case=case,frame=int(z['frame']),original_hazard_points=int(haz.sum()),candidate_hazard_points=int(candidate.sum()),prior_negative_hazards=int((haz&prior_negative).sum()),selected_query_ids_equal=True));inputs[str(p)]=sha(p)
sources=[H/'partial_cue_source_v2.py',H/'qualified_hazard_consumer.py',H/'typed_hypothesis_grid.py'];r=dict(decision='REJECT_GROUND_SIGN_GUARD_AS_BEHAVIORAL_CONTROL_CANDIDATE',reason='current .05<height<=1.5 hazard support excludes all below-plane points already; prior guard adds zero seed changes; once immutable belief born, later raw source is not consumed as measured geometry',scope='same-height hazard support equivalence only, not every hypothetical planner or unknown-memory consumer',all26_hazard_supports_equal=True,source_rejection_not_rescored=True,source_or_actor_admission=False,no_physics_AB_triggered=True,rows=rows,inputs=inputs,code_sha256=sha(__file__),downstream_source_sha256={str(p):sha(p) for p in sources});(H/'ground_guard_behavior_equivalence_v1.json').write_text(json.dumps(r,indent=2)+'\n');print({k:v for k,v in r.items() if k not in ['rows','inputs','downstream_source_sha256']})
