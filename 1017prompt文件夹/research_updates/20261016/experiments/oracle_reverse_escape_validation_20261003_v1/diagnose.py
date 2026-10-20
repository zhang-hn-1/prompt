"""Retrospective geometry and braking measurements; no new controller."""
import json
from pathlib import Path
import numpy as np

HERE=Path(__file__).resolve().parent


def distance_to_polyline(xy,path):
    start=path[:-1];delta=np.diff(path,axis=0);squared=np.sum(delta*delta,axis=1)
    offset=xy[:,None,:]-start[None,:,:]
    projection=np.clip(np.sum(offset*delta[None,:,:],axis=2)/np.maximum(squared,1e-12),0,1)
    nearest=start[None,:,:]+projection[:,:,None]*delta[None,:,:]
    return np.min(np.linalg.norm(xy[:,None,:]-nearest,axis=2),axis=1)


def main():
    report=json.loads((HERE/'assessment.json').read_text())
    results={}
    for r in report['results']:
        root=HERE/r['episode_id'];folder=root/f"episode_{r['episode_id']}"
        with np.load(root/'physics.npz') as a:
            arrays={k:a[k].copy() for k in a.files}
        t=arrays['simulation_time_s'];xy=arrays['position_world_xy']
        diag={}
        if r['route_built']:
            with np.load(root/'reference_built_at_6s.npz') as p:
                path=p['position_world_xy']
            error=distance_to_polyline(xy,path)
            follow=t>=6.00001
            diag['whole_reference_nearest_polyline_error']=dict(
                tracking_rms_m=float(np.sqrt(np.mean(error[follow]**2))),
                tracking_peak_m=float(error[follow].max()),
                endpoint_m=float(error[-1]),
                warning='global nearest polyline geometry; not monotonic progress or a control score')
        if r['first_goal_time_s'] is not None:
            spec=next(e for e in json.loads((folder/'manifest.json').read_text())['episodes'] if e['episode_id']==r['episode_id'])
            goal=np.asarray(spec['goal_position_world_m'][:2]);distance=np.linalg.norm(xy-goal,axis=1)
            entry=int(np.argmin(abs(t-r['first_goal_time_s'])))
            raw=arrays['raw_command_target'][:,0];smooth=arrays['smoothed_target'][:,0]
            applied=arrays['applied_action'][:,0]
            within=np.flatnonzero((t>10)&(t<=t[entry])&(distance<=.6))
            onset=int(within[0])
            slope=np.diff(smooth[entry:])/np.diff(t[entry:])
            diag['terminal_chain']=dict(
                first_within_slowdown_radius_time_s=float(t[onset]),
                distance_at_slowdown_m=float(distance[onset]),
                slowdown_to_goal_entry_s=float(t[entry]-t[onset]),
                at_entry_raw_main_rad_s=float(raw[entry]),
                at_entry_smoothed_main_rad_s=float(smooth[entry]),
                at_entry_applied_main_rad_s=float(applied[entry]),
                at_entry_measured_main_joint_rad_s=float(arrays['joint_velocity_rad_s'][entry,0]),
                at_entry_world_xy_speed_m_s=float(np.linalg.norm(arrays['linear_velocity_world_m_s'][entry,:2])),
                observed_post_entry_smoother_rate_min_rad_s2=float(slope.min()),
                observed_post_entry_smoother_rate_max_rad_s2=float(slope.max()),
                minimum_time_for_entry_smoothed_main_to_zero_s=abs(float(smooth[entry]))/.5,
                note='smoother timing is execution evidence; not a physical stopping distance prediction')
        results[r['episode_id']]=diag
    (HERE/'diagnostics.json').write_text(json.dumps(results,indent=2)+'\n')
    print(json.dumps(results,indent=2))


if __name__=='__main__':main()
