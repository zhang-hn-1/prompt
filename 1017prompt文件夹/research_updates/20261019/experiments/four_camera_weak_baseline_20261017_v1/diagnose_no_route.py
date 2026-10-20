"""ONE postseal first sustained no-route event; GT deletion only counterfactual.
This is 2D global grid cause isolation, NOT physical navigation or GT-fed repair.
"""
from paths import *
import json,numpy as np,hashlib
from scipy.spatial import cKDTree
from execution_domain_four_point_consumer_v2 import TrackingTubeGrid,planning_clearance
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
out=D/'e1_run';manifest=json.loads((out/'artifact_manifest_post_close.json').read_text())
for f,h in manifest['artifact_sha256'].items():assert sha(f)==h,f
rows=json.loads((out/'policy_records.json').read_text());cfg=json.loads((D/'protocol.json').read_text());ep=json.loads((out/'episode.json').read_text());indices=[];chosen=None
for i,row in enumerate(rows):
    no=(row['time_s']>=6 and max(abs(v) for v in row['request'])==0 and row['plan'].get('status')!='success')
    indices=indices+[i] if no else []
    if len(indices)>=50:chosen=indices[0];break
if chosen is None:
    result=dict(complete=True,event=None,scope='No first>=5s contiguous zero/no-success-plan event; no selected other event, no counterfactual run.',physics=0,fit=0)
else:
    row=rows[chosen];m=np.load(out/'maps'/f'frame_{chosen:03d}.npz');occ=np.load(cfg['occupancy']);ref=np.argwhere(occ['grid']>0)*float(occ['resolution_m'])+occ['global_origin_xy'];tree=cKDTree(ref)
    origin=m['origin'];hazard=np.argwhere(m['labels']==1);xy=hazard*.1+origin;bad=tree.query(xy)[0]>.15
    pred=m['labels'].copy();cleared=pred.copy();cleared[tuple(hazard[bad].T)]=-1
    true=np.full(pred.shape,-1,np.int8);cells=np.floor((ref-origin)/.1+.5).astype(int);cells=cells[(cells>=0).all(1)&(cells<np.array(pred.shape)).all(1)];true[tuple(cells.T)]=1
    start=np.array(row['estimated_body'])[:2,3];goal=np.asarray(ep['goal_position_world_m'])[:2];arm={}
    for name,label in [('actual_risk_grid',pred),('GT_remove_only_false_risk',cleared),('GT_static_bodyband_grid',true)]:
        g=TrackingTubeGrid(origin,10);g.labels=label
        path,status=TrackingTubeGrid.plan(g,start,goal)
        arm[name]=dict(status=status,route_exists=path is not None,path_length_m=float(np.linalg.norm(np.diff(path,axis=0),axis=1).sum()) if path is not None else None)
    result=dict(complete=True,event=dict(policy_frame=chosen,time_s=row['time_s'],actual_plan_status=row['plan']['status'],duration_at_least_s=5,estimated_start=start.tolist(),task_goal=goal.tolist(),hazard_cells=len(hazard),false_cells=int(bad.sum())),counterfactual=arm,clearance_m=planning_clearance(g),physics=0,fit=0,source_modified=False,GT_used_only_offline=True,scope='First sustained5s zero/no-success-plan event. Identical .1m grid, unknown traversal and .870711m global clearance. GT-remove arm turns false risk UNKNOWN, never observed free. FullGT arm scene reference raster only. Does not verify local kinodynamic primitives, control, 3D clearance or an online deletion rule.',original_grid_failure_reproduced=arm['actual_risk_grid']['route_exists']==False,false_risk_removal_sufficient_for_global_grid_connectivity=not arm['actual_risk_grid']['route_exists'] and arm['GT_remove_only_false_risk']['route_exists'])
(D/'no_route_diagnosis.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n');print('NO_ROUTE_DIAGNOSIS',result,flush=True)
