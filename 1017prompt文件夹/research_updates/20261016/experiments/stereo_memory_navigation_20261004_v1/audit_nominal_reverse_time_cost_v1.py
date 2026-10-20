"""Exactly four frozen current-map CPU queries. No future outcome labels."""
from pathlib import Path
import json,hashlib
import numpy as np
from scipy.spatial.transform import Rotation
from scripts.run_pointgoal_benchmark import load_action_protocol
from prior_rgb_navigation import PriorRGBPolicy
from repeated_exposure_memory_v1 import RepeatedExposureMemoryGrid
from nominal_reverse_time_cost_v1 import local_curve_nominal_time,SPEED_RATIO
H=Path(__file__).resolve().parent;D=H/'internal_cusp_navigation_e1_b0_run_v1'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()

def nominal_cost(waypoints,initial):
    previous=initial;total=0.;reverse_length=0.
    # Each nonterminal waypoint carries the outgoing edge action.
    for w in waypoints[:-1]:
        direction=w['drive_direction']
        total+=.3*(1.25*SPEED_RATIO if direction=='reverse' else 1.)
        reverse_length+=.3 if direction=='reverse' else 0.
        if previous is not None and direction!=previous:total+=.5
        previous=direction
    return dict(normalized_nominal_cost_m=total,reverse_nominal_length_m=reverse_length)

if __name__=='__main__':
    out=H/'nominal_reverse_time_cost_assessment_v1.json';assert not out.exists()
    design=json.loads((H/'nominal_reverse_time_cost_design_v1.json').read_text())
    identity=json.loads((H/'nominal_reverse_time_cost_probe_identity_v1.json').read_text())
    # Verify every table separately so overlap cannot hide a conflicting SHA.
    for table in [identity['input_sha256'],design['input_sha256'],identity['source_sha256']]:
        for p,s in table.items():assert sha(p)==s,p
    goal=json.loads((D/'task_episode.json').read_text())['actual_episode']['goal_position_world_m'][:2]
    protocol=load_action_protocol(Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/configs/data_contract.yaml'))
    drive=PriorRGBPolicy(protocol,memory_mode='current').tracker.drive_model
    original_load=np.load;frame=None;loaded=[]
    def guarded(p,*a,**kw):
        p=Path(p)
        if p.parent.name in ['frames','history','maps']:
            assert p.name==f'frame_{frame:03d}.npz','current frame only'
            loaded.append(str(p))
        return original_load(p,*a,**kw)
    np.load=guarded
    results=[];count=0
    for q in design['fixed_queries']:
        frame=q['frame'];capture=dict(np.load(D/'frames'/f'frame_{frame:03d}.npz'))
        m=dict(np.load(D/'maps'/f'frame_{frame:03d}.npz'))
        saved=json.loads((D/'local_plans'/f'frame_{frame:03d}.json').read_text())
        t=float(capture['time_s']);assert t==float(m['time_s'])==saved['time_s']
        R=Rotation.from_quat(capture['root_quaternion_world_wxyz'][[1,2,3,0]]).as_matrix()
        yaw=float(np.arctan2(R[1,0],R[0,0]));xy=capture['root_position_world_m'][:2]
        answers={};error=0.
        for arm in ['original','candidate']:
            grid=RepeatedExposureMemoryGrid(m['origin'],10,static_scene_assumption=True)
            grid.labels=m['labels'].copy();grid.time=t;assert grid.resolution==float(m['resolution'])
            plan,meta=local_curve_nominal_time(grid,xy,yaw,goal,drive,
                initial_direction=q['initial_direction'],first_primitive_direction=q['first_primitive_direction'],enabled=arm=='candidate')
            count+=1;assert count<=4
            w=[] if plan is None else [vars(p) for p in plan.waypoints]
            assert np.array_equal(grid.labels,m['labels'])
            answers[arm]=dict(meta=meta,waypoints=w,**nominal_cost(w,q['initial_direction']))
            if arm=='original':
                assert len(w)==len(saved['waypoints'])
                for got,want in zip(w,saved['waypoints']):
                    assert got['drive_direction']==want['drive_direction']
                    for k in ['x_m','y_m','yaw_rad','main_command_rad_s','auxiliary_angle_rad']:error=max(error,abs(got[k]-want[k]))
                assert error<=1e-8
        a,b=answers['original'],answers['candidate']
        results.append(dict(frame=frame,time_s=t,queries=answers,original_error=error,
            route_changed=a['waypoints']!=b['waypoints'],
            candidate_success=b['meta']['status']=='success',
            cost_lower=b['normalized_nominal_cost_m']<a['normalized_nominal_cost_m']-1e-8,
            cusps_not_increased=b['meta'].get('direction_changes',999)<=a['meta'].get('direction_changes',-1)))
    passed=all(r['candidate_success'] and r['cusps_not_increased'] for r in results) and any(r['route_changed'] and r['cost_lower'] for r in results)
    out.write_text(json.dumps(dict(results=results,query_count=count,candidate_cpu_gate=passed,
        decision='CPU_INTERFACE_PASS_ONLY' if passed else 'CPU_COST_CANDIDATE_REJECT',
        current_frame_array_reads=loaded,not_online=True,not_physical_admission=True,
        no_new_fit=True,no_alternate_future_truth=True,whole_mission_complete=False),indent=2)+'\n')
    print(json.dumps(dict(passed=passed,query_count=count,results=[{k:v for k,v in r.items() if k!='queries'}|{'old_cost':r['queries']['original']['normalized_nominal_cost_m'],'new_cost':r['queries']['candidate']['normalized_nominal_cost_m'],'candidate_status':r['queries']['candidate']['meta']['status']} for r in results])))
