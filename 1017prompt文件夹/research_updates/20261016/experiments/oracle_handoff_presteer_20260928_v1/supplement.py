import json
from pathlib import Path
import numpy as np
from probe import write
HERE=Path(__file__).resolve().parent
results={}
for case in ['0002','0004']:
    results[case]={}
    for group in ['real','recovery']:
        p=HERE/f'{case}_{group}';records=json.loads((p/'tracking.json').read_text());z=np.load(p/'trace.npz');status=json.loads((p/'status.json').read_text());e=status['event'];brake=[i for i,r in enumerate(records) if r['source']=='direction_change_braking']
        first_track=next((i for i in range(e,len(records)) if records[i]['source']=='tracking' and records[i]['reference_drive_direction']=='forward'),None)
        r=records[first_track] if first_track is not None else {}
        results[case][group]=dict(event=e,first_braking=brake[0] if brake else None,preview_matches_actual=bool(brake and brake[0]==e),braking_blocks=len(brake),first_forward_block=first_track,first_forward_time_s=None if first_track is None else .1*first_track,first_forward_auxiliary_rad=r.get('actual_auxiliary_angle_rad'),first_forward_heading_error_rad=r.get('signed_body_heading_error_rad'),first_forward_speed_m_s=r.get('world_xy_speed_m_s'),all_braking_main_zero=all(records[i]['policy_protocol_main_command_rad_s']==0. for i in brake),braking_aux_nonzero_blocks=sum(abs(records[i]['policy_protocol_auxiliary_target_rad'])>1e-6 for i in brake))
        end=first_track if first_track is not None else len(z['position'])-1
        results[case][group]['braking_net_displacement_xy_m']=(z['position'][end]-z['position'][e]).tolist()
        results[case][group]['braking_distance_travelled_m']=float(np.linalg.norm(np.diff(z['position'][e:end+1],axis=0),axis=1).sum())
        results[case][group]['braking_duration_s']=.1*(end-e)
        results[case][group]['five_low_speed_samples_at_release']=None if first_track is None else all(records[i]['world_xy_speed_m_s']<.05 for i in range(first_track-4,first_track+1))
        if group=='real':
            old=np.load(HERE.parent/'oracle_same_direction_recovery_20260928_v1'/f'{case}_real/trace.npz')
            results[case][group]['previous_B0_prefix_exact']={k:bool(np.array_equal(z[k],old[k][:len(z[k])])) for k in ['position','quaternion','proprio','action']}
write(HERE/'supplement.json',results)
print(json.dumps(results,indent=2))
