"""Explain actual failed route queries without changing any planning input."""
import bootstrap
from bootstrap import D
import json, numpy as np
from collections import Counter
from scipy.ndimage import distance_transform_edt
from scipy.spatial import cKDTree
from execution_domain_four_point_consumer_v2 import planning_clearance
from types import SimpleNamespace

R=D/'candidate_e1_run'
rows=json.loads((R/'policy_records.json').read_text())
native=[n for n in json.loads((R/'native_predictions.json').read_text()) if n.get('available')]
goal=np.array(json.loads((R/'task_episode.json').read_text())['actual_episode']['goal_position_world_m'][:2])
ref=np.load(json.loads((D/'candidate_e1_protocol.json').read_text())['occupancy'])
truth=ref['global_origin_xy']+np.argwhere(ref['grid']>0)*float(ref['resolution_m'])
tree=cKDTree(truth);details=[]
for row in rows:
    if not row.get('local_plan_created') or row['plan']['status']!='observed_obstacle_blocks_route':continue
    i=row['frame'];m=np.load(R/'maps'/f'frame_{i:03d}.npz');t=float(row['time_s'])
    n=min(native,key=lambda n:abs(n['time_s']-t));assert abs(n['time_s']-t)<1e-4
    xy=np.asarray(n['position_world'],float)[:2];res=float(m['resolution']);origin=m['origin'];labels=m['labels']
    cells=lambda xy:np.floor((xy-origin)/res+.5).astype(int)
    start,end=cells(xy),cells(goal)
    clearance=distance_transform_edt(np.pad(labels!=1,1))[1:-1,1:-1]*res
    margin=planning_clearance(SimpleNamespace(resolution=res))
    sc,ec=float(clearance[tuple(start)]),float(clearance[tuple(end)])
    risk=np.argwhere(labels==1);world=origin+risk*res;dist=np.linalg.norm(world-(origin+start*res),axis=1)
    causing=dist<margin-1e-8;nearest=int(np.argmin(dist))
    dd=tree.query(world)[0];age=t-m['hazard_acquisition_time'][tuple(risk.T)]
    details.append(dict(frame=i,time_s=t,start_clearance_m=sc,goal_clearance_m=ec,
        start_blocked=sc<margin-1e-8,goal_blocked=ec<margin-1e-8,
        native_xy=xy.tolist(),start_risk_within_margin=int(causing.sum()),
        start_risk_farther015m_reference=int((causing&(dd>.15)).sum()),
        start_risk_old10s=int((causing&(age>10)).sum()),
        nearest_risk_xy=world[nearest].tolist(),nearest_risk_to_reference_m=float(dd[nearest]),
        nearest_risk_age_s=float(age[nearest]),required_clearance_m=margin))
result=dict(scope='Actual candidate global-route failures only; grid-cell clearance is the executed planner criterion. Reference distances are post-run 2D body-band scoring, not ground truth online or complete surface correctness.',
    blocked_queries=len(details),start_blocked=sum(d['start_blocked'] for d in details),goal_blocked=sum(d['goal_blocked'] for d in details),
    all_start_blockers_farther015m=sum(d['start_risk_within_margin']==d['start_risk_farther015m_reference'] for d in details),details=details)
(D/'blocked_queries.json').write_text(json.dumps(result,indent=2)+'\n')
print({k:v for k,v in result.items() if k!='details'})
print('first',details[0]);print('last',details[-1])
