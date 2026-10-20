"""Independent actual navigation and exact historical behavior comparison."""
import hashlib,json
from pathlib import Path
import numpy as np

HERE=Path(__file__).parent
OLD=HERE.parent/'fixed_rig_visual_pilot_20261001/vision_detour_feasibility_v1'


def main():
    output={}
    for label in ('e0','e1','e2'):
        root=HERE/'legacy_clean'/label;episode=f'episode_golden_{label}_0001';folder=root/episode
        metrics=json.loads((folder/'metrics.json').read_text())
        identity=json.loads((root/'experiment_identity.json').read_text())
        assert all(hashlib.sha256(Path(f).read_bytes()).hexdigest()==h for f,h in identity['source_sha256'].items())
        model=OLD.parent/'illuminated_replay_v1/augmented/model_holdout_e1.pt'
        assert hashlib.sha256(model.read_bytes()).hexdigest()==identity['model_sha256']
        visibility=json.loads((root/'marker_visibility_each_capture.json').read_text())
        assert visibility and all(len(v)==4 and all(r['computed_visibility']=='invisible' for r in v.values()) for v in visibility)
        audit=json.loads((folder/'collision_audit.json').read_text())['samples']
        available={int(s['physics_step_index']) for s in audit if s['available']}
        contact=any(s['contact'] for s in audit)
        with np.load(root/'physics.npz') as f:coverage=all(int(i) in available for i in f['physics_step_index'])
        records=json.loads((root/'vision_records.json').read_text())
        with np.load(folder/'actions.npz') as a,np.load(OLD/f'{label}_continuous'/episode/'actions.npz') as b:
            action_exact=bool(np.array_equal(a['action_blocks'],b['action_blocks']) and np.array_equal(a['controls'],b['controls']))
        with np.load(folder/'states.npz') as a,np.load(OLD/f'{label}_continuous'/episode/'states.npz') as b:
            xy=a['position_world_xy'];velocity=a['linear_velocity_world_m_s'];t=a['simulation_time_s']
            state_exact={k:bool(np.array_equal(a[k],b[k])) for k in ('position_world_xy','yaw_world_rad',
             'linear_velocity_world_m_s','angular_velocity_world_rad_s','joint_position_rad','joint_velocity_rad_s','applied_action')}
        assert len(records)==len(xy)-1
        manifest=json.loads((folder/'manifest.json').read_text())['episodes']
        spec=next(e for e in manifest if e['episode_id']==f'golden_{label}_0001')
        goal=np.asarray(spec['goal_position_world_m'][:2]);distance=float(np.linalg.norm(xy[-1]-goal))
        crossing=np.flatnonzero((xy[:-1,0]<0)&(xy[1:,0]>=0)) if label=='e1' else []
        passed=bool(metrics['success']==1 and not contact and coverage and distance<=.2 and t[-1]<=35)
        assert passed and action_exact and all(state_exact.values()) and metrics['episode_budget_s']==35
        output[label]=dict(success=passed,time_s=float(t[-1]),final_distance_m=distance,
          path_length_m=float(np.linalg.norm(np.diff(xy,axis=0),axis=1).sum()),physical_contact=contact,
          complete_contact_coverage=coverage,all_marker_capture_checks=len(visibility),
          source_model_sha_match=True,actions_exact_to_old=action_exact,states_exact_to_old=state_exact,
          trigger_time_s=next((i/10 for i,r in enumerate(records) if r['detour_triggered']),None),
          e1_actual_crossing_y_m=float(xy[crossing[0]+1,1]) if len(crossing) else None,
          final_xy_speed_m_s=float(np.linalg.norm(velocity[-1,:2])),strict_stop_hold_tested=False)
        print(label,output[label],flush=True)
    (HERE/'legacy_clean_assessment.json').write_text(json.dumps(dict(results=output,
         golden_complete_sequences_success=sum(x['success'] for x in output.values()),
         whole_scene_generalization_claim=False,scope='unchanged left RGB controller; not masked binocular candidate'),indent=2)+'\n')


if __name__=='__main__':main()
