"""Post-run historical comparison; differing images prohibit complete causal claims."""
from pathlib import Path
import json,hashlib,numpy as np
D=Path(__file__).resolve().parent;W=D.parents[1];B=W/'experiments/nearfield_rig_navigation_20261012_v1/candidate_e2_run';C=D/'candidate_e2_run'
a=np.load(B/'requests.npz')['requests'];b=np.load(C/'requests.npz')['requests'];n=min(len(a),len(b));diff=np.flatnonzero(np.any(a[:n]!=b[:n],axis=1));first=int(diff[0]) if len(diff) else None
images=[]
for p in sorted((C/'source_inputs').glob('*.npz')):
 if not (B/'source_inputs'/p.name).exists():continue
 l=np.load(p);r=np.load(B/'source_inputs'/p.name)
 unequal=sum(int(np.count_nonzero(l[e]!=r[e])) for e in ['left_rgb','right_rgb'])
 if unequal:images.append(dict(frame=p.stem,channel_values_different=unequal))
first_removed=None;total=0
for p in sorted((C/'partial_observations').glob('*.npz')):
 x=np.load(p);count=int(x['height_interval_ambiguous_count']);total+=count
 if count and first_removed is None:first_removed=dict(frame=int(x['frame']),time_s=float(x['acquisition_time_s']),count=count)
x=dict(first_filtered_source=first_removed,total_filtered_points=total,historical_request_difference_count=int(len(diff)),first_historical_request_difference=first,first_rgb_difference=images[0] if images else None,full_rgb_prefix_equal=not images,no_full_causal_claim=True)
(D/'intervention_comparison.json').write_text(json.dumps(x,indent=2)+'\n');print(json.dumps(x))
