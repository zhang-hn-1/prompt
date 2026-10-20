"""Post-run descriptive statistics only; no acceptance threshold changes."""
import json
from pathlib import Path
import numpy as np
HERE=Path(__file__).resolve().parent
results={}
for case in ['0002','0004']:
    results[case]={}
    pair=[]
    for group in ['real','reserve']:
        p=HERE/f'{case}_{group}'
        z=np.load(p/'physics.npz'); t=np.load(p/'trace.npz'); pair.append(z)
        r=json.loads((p/'tracking.json').read_text())
        speed=np.linalg.norm(z['linear_velocity_world_m_s'][:,:2],axis=1)
        results[case][group]={
            'physical_samples':len(speed),'xy_speed_mean_m_s':float(speed.mean()),'xy_speed_peak_m_s':float(speed.max()),
            'joint_abs_velocity_mean_rad_s':np.abs(z['joint_velocity_rad_s']).mean(axis=0).tolist(),
            'joint_abs_velocity_peak_rad_s':np.abs(z['joint_velocity_rad_s']).max(axis=0).tolist(),
            'raw_main_abs_mean_rad_s':float(np.abs(z['raw_command_target'][:,0]).mean()),
            'raw_main_abs_peak_rad_s':float(np.abs(z['raw_command_target'][:,0]).max()),
            'applied_main_abs_mean_rad_s':float(np.abs(z['applied_action'][:,0]).mean()),
            'applied_main_abs_peak_rad_s':float(np.abs(z['applied_action'][:,0]).max()),
            'aux_angle_abs_peak_rad_by_joint':np.abs(z['joint_position_rad']).max(axis=0).tolist(),
            'candidate_restored_blocks':sum(x.get('pacing_active',False) and x['pacing_planned_main_magnitude']>x['pacing_baseline_main_magnitude']+1e-8 for x in r),
        }
    a,b=pair
    changed=np.flatnonzero(np.any(np.abs(a['applied_action']-b['applied_action'])>1e-7,axis=1))
    results[case]['first_applied_difference_s']=float(a['simulation_time_s'][changed[0]]) if len(changed) else None
    results[case]['applied_differing_samples']=len(changed)
(HERE/'physical_summary.json').write_text(json.dumps(results,indent=2)+'\n')
print(json.dumps(results,indent=2))
