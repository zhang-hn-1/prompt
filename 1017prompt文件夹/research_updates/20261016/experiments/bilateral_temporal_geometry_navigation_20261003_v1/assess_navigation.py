"""Actual goal/contact outcomes and image->request evidence, not policy indices."""
import hashlib
import json
import os
from pathlib import Path
import numpy as np

HERE=Path(os.environ.get('BILATERAL_ASSESS_ROOT',Path(__file__).resolve().parent))


def measure(case):
    difficulty=case.rsplit('_',1)[1];episode=f'golden_{difficulty}_0001';root=HERE/case;folder=root/f'episode_{episode}'
    metrics=json.loads((folder/'metrics.json').read_text())
    spec=next(e for e in json.loads((folder/'manifest.json').read_text())['episodes'] if e['episode_id']==episode)
    goal=np.asarray(spec['goal_position_world_m'][:2])
    with np.load(folder/'states.npz') as f:
        xy=f['position_world_xy'];t=f['simulation_time_s'];v=f['linear_velocity_world_m_s']
    with np.load(folder/'actions.npz') as a:actions=a['action_blocks']
    identity=json.loads((root/'experiment_identity.json').read_text())
    matched=all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==h for p,h in identity['source_sha256'].items())
    samples=json.loads((folder/'collision_audit.json').read_text())['samples']
    with np.load(root/'physics.npz') as p:
        available={int(s['physics_step_index']) for s in samples if s['available']}
        covered=all(int(k) in available for k in p['physics_step_index'])
    final_distance=float(np.linalg.norm(xy[-1]-goal))
    contact=any(s['contact'] for s in samples)
    passed=(metrics['success']==1 and final_distance<=.2 and not contact and covered and t[-1]<=35.00001)
    result=dict(case=case,actual_policy=identity['actual_policy'],success=bool(passed),termination=metrics['termination_reason'],
         elapsed_s=float(t[-1]),goal_distance_m=final_distance,path_length_m=float(np.linalg.norm(np.diff(xy,axis=0),axis=1).sum()),
         contact=bool(contact),first_contact_s=next((s['simulation_time_s'] for s in samples if s['contact']),None),
         contact_sensor_coverage_complete=bool(covered),source_hashes_match=matched,episode_budget_s=metrics['episode_budget_s'],
         terminal_xy_speed_m_s=float(np.linalg.norm(v[-1,:2])),runtime_error=metrics.get('runtime_error_message'))
    if case.startswith('candidate'):
        records=json.loads((root/'vision_records.json').read_text())
        assert len(records)==len(actions)
        assert all(np.array_equal(np.asarray(r['request_main_aux'],dtype=np.float32),a[:2]) for r,a in zip(records,actions))
        created=[r for r in records if r['reference_created']]
        result.update(first_visual_route_time_s=created[0]['time_s'] if created else None,
          selected_side=created[0]['side'] if created else None,
          left_depth_predictions=sum(r['eyes']['left']['depth_predictions'] for r in records),
          right_depth_predictions=sum(r['eyes']['right']['depth_predictions'] for r in records),
          inference_time_median_s=float(np.median([r['inference_wall_s'] for r in records])),
          inference_time_p90_s=float(np.percentile([r['inference_wall_s'] for r in records],90)))
    assert matched and covered and metrics['episode_budget_s']==35 and metrics.get('runtime_error_message') is None
    return result


def main():
    results={case:measure(case) for case in ('b0_e2','candidate_e2','b0_e1','candidate_e1')}
    comparisons={}
    for difficulty in ('e2','e1'):
        base=HERE/f'b0_{difficulty}';candidate=HERE/f'candidate_{difficulty}'
        delta={}
        with np.load(base/'physics.npz') as a,np.load(candidate/'physics.npz') as b:
            for key in ('position_world_xy','root_quaternion_world_wxyz','linear_velocity_world_m_s',
                         'joint_position_rad','joint_velocity_rad_s','smoothed_target','applied_action'):
                delta[key]=float(np.max(abs(a[key][0]-b[key][0])))
        episode=f'episode_golden_{difficulty}_0001'
        with np.load(base/episode/'actions.npz') as a,np.load(candidate/episode/'actions.npz') as b:
            n=min(len(a['action_blocks']),len(b['action_blocks']))
            changed=np.flatnonzero(np.any(a['action_blocks'][:n]!=b['action_blocks'][:n],axis=1))
        prefix={};raw_clock_offsets={}
        count=(int(changed[0]) if len(changed) else n)*5+1
        with np.load(base/'physics.npz') as a,np.load(candidate/'physics.npz') as b:
            for key in a.files:
                aa=a[key][:count].astype(float);bb=b[key][:count].astype(float)
                if key in ('raw_simulation_time_s','raw_physics_step_index'):
                    raw_clock_offsets[key]=float(bb[0]-aa[0])
                    prefix[f'{key}_since_reset']=float(np.max(abs((aa-aa[0])-(bb-bb[0]))))
                else:
                    prefix[key]=float(np.max(abs(aa-bb)))
        comparisons[difficulty]=dict(initial_max_abs_differences=delta,same_initial_state=all(x<=1e-6 for x in delta.values()),
                  first_request_difference_block=int(changed[0]) if len(changed) else None,
                  identical_physics_prefix_samples=count,prefix_max_abs_differences=prefix,
                  raw_clock_start_offsets=raw_clock_offsets,
                  prefix_exact=all(x<=1e-6 for x in prefix.values()))
        assert comparisons[difficulty]['same_initial_state']
        assert comparisons[difficulty]['prefix_exact']
    perception=json.loads((HERE/'offline_assessment.json').read_text())
    output=dict(results=results,comparisons=comparisons,perception_gate_passed=perception['accepted'],
                accepted=perception['accepted'] and all(results[f'candidate_{x}']['success'] for x in ('e1','e2')),
                scope='two historical development scenes, map-free local vision with benchmark localization',
                strict_stop_hold_not_tested=True)
    (HERE/'navigation_assessment.json').write_text(json.dumps(output,indent=2)+'\n')
    print(json.dumps(output,indent=2))


if __name__=='__main__':main()
