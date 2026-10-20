"""Post-run planning effect replay only; no truth feedback or new physics."""
import bootstrap
from bootstrap import D
import json,hashlib,numpy as np
from benchmark.drive_model import DriveModel
from scripts.run_tracking_acceptance import DEFAULT_DRIVE_PROFILE
from reserve import reserve_curve,reserve_screen
from minimum_prefix_transition_consumer_v1 import local_curve_first_direction as original
from reference_continuity_audit import reference_geometry_status
from execution_domain_four_point_consumer_v2 import TrackingTubeGrid

class SavedGrid:
    def __init__(self,z,meta):
        self.labels=z['labels'].copy();self.origin=z['origin'].copy();self.resolution=float(z['resolution'])
        self.exploration_unknown_fraction=meta['unknown_fraction']
    def cells(self,xy):return np.floor((xy-self.origin)/self.resolution+.5).astype(int)
    def inside(self,c):return ((c>=0)&(c<np.asarray(self.labels.shape))).all(axis=1)
    # Actual nonbinding-side grid delegates all route queries to this method.
    plan=TrackingTubeGrid.plan

model=DriveModel.from_json(DEFAULT_DRIVE_PROFILE);results={}
for arm in ['baseline_e1','candidate_e1']:
    R=D/(arm+'_run')
    if not (R/'termination.json').exists():continue
    records=json.loads((R/'policy_records.json').read_text());natives=[x for x in json.loads((R/'native_predictions.json').read_text()) if x.get('available')]
    goal=json.loads((R/'task_episode.json').read_text())['actual_episode']['goal_position_world_m'][:2]
    frames=sorted({r['local_reference_version']:i for i,r in reversed(list(enumerate(records))) if r.get('local_reference_version',0)>0}.values())
    rows=[];hashes={}
    for i in frames:
        f=R/'local_plans'/f'frame_{i:03d}.json';mp=R/'maps'/f'frame_{i:03d}.npz';row=records[i];meta=row['plan']
        saved=json.loads(f.read_text()) if f.exists() else None
        g=SavedGrid(np.load(mp),meta);before=g.labels.tobytes()
        n=min(natives,key=lambda a:abs(float(a['time_s'])-float(row['time_s'])))
        assert abs(float(n['time_s'])-float(row['time_s']))<1e-4
        xy=np.asarray(n['position_world'],float)[:2]
        rotation=np.asarray(n['rotation_world']);yaw=float(np.arctan2(rotation[1,0],rotation[0,0]))
        if saved:
            wp=saved['waypoints'];start=wp[0]
            np.testing.assert_allclose(xy,[start['x_m'],start['y_m']],atol=1e-7,rtol=0)
            assert abs(np.arctan2(np.sin(start['yaw_rad']-yaw),np.cos(start['yaw_rad']-yaw)))<1e-7
        kw=dict(initial_direction=meta['initial_direction'],first_primitive_direction=row['first_primitive_direction_query']['direction'])
        base,bm=original(g,xy,yaw,goal,model,**kw)
        actual,am=(reserve_curve(g,xy,yaw,goal,model,**kw) if arm=='candidate_e1' else (base,bm))
        assert (actual is not None)==(saved is not None)
        assert am['status']==meta['status']
        assert am.get('route_xy')==meta.get('route_xy')
        if saved:
            assert [vars(w) for w in actual.waypoints]==wp and actual.expanded_nodes==saved['expanded_nodes']
        else:
            assert not any(row['request'])
        assert g.labels.tobytes()==before
        changed=actual is not None and (base is None or [vars(w) for w in base.waypoints]!=wp)
        if arm=='candidate_e1' and saved:
            assert am['generated_reference_reserve_screen']['pass']
            assert am['planning_clearance_m']==meta['planning_clearance_m']==1.0707106781186548
        rows.append(dict(frame=i,time_s=row['time_s'],status=am['status'],successful_installation=actual is not None,native_pose_exact=True,actual_plan_exact=True,changed_from_same_input_original=changed,baseline_length_m=None if base is None else base.path_length_m,actual_length_m=None if actual is None else actual.path_length_m,baseline_cusps=None if base is None else base.direction_changes,actual_cusps=None if actual is None else actual.direction_changes,reserve_screen=am.get('generated_reference_reserve_screen')))
        for p in [f,mp]:
            if p.exists():hashes[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest()
        print(arm,i,'changed',changed,flush=True)
    results[arm]=dict(queries=rows,attempted_queries=len(rows),successful_installations=sum(r['successful_installation'] for r in rows),failed_queries=sum(not r['successful_installation'] for r in rows),changed_plan_count=sum(r['changed_from_same_input_original'] for r in rows),input_sha256=hashes)
# Check every candidate retention boundary against actual map/current native state.
R=D/'candidate_e1_run';records=json.loads((R/'policy_records.json').read_text());natives=[x for x in json.loads((R/'native_predictions.json').read_text()) if x.get('available')];retained=[]
for i,row in enumerate(records):
 decision=row.get('reference_decision') or {}
 if i%5 or decision.get('decision')!='retain' or not row.get('planner_act_called'):continue
 n=min(natives,key=lambda a:abs(float(a['time_s'])-float(row['time_s'])));assert abs(float(n['time_s'])-float(row['time_s']))<1e-4
 g=SavedGrid(np.load(R/'maps'/f'frame_{i:03d}.npz'),row['plan']);guard=reference_geometry_status(np.asarray(n['position_world'],float)[:2],np.asarray(records[i-1]['installed_reference_xy_after_act']),g.labels,g.origin,g.resolution);screen=reserve_screen(g,guard);assert screen['pass'];assert abs(screen['minimum_continuous_cell_clearance_m']-decision['replanning_reserve']['minimum_continuous_cell_clearance_m'])<1e-10;retained.append(i)
results['candidate_e1']['retained_boundaries_exact']=retained
(D/'actual_effect.json').write_text(json.dumps(dict(pass_gate=set(results)=={'baseline_e1','candidate_e1'},arms=results,scope='factual current native inputs and actual installed plan identity; counterfactual baseline query only, not a counterfactual trajectory'),indent=2)+'\n')
