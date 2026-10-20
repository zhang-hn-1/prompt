"""Independent geometric and scored assessment of the bounded route experiment."""
from pathlib import Path
import json
import numpy as np
h=Path(__file__).resolve().parent
v8=h.parent/'oracle_collision_fix_20260929_v8/pid_0002_contact/episode_hard_0_e1_0002'
cases={
 'old_reference_0002':v8,
 'time_reference_0002':h/'navigation_time_feasible_0002/episode_hard_0_e1_0002',
 'speed_change_0002':h/'navigation_moderate_speed_valid_0002/episode_hard_0_e1_0002',
 'lookahead_change_0002':h/'navigation_lookahead_080_0002/episode_hard_0_e1_0002',
}
p=h/'navigation_time_feasible_0001/episode_hard_0_e1_0001'
if (p/'metrics.json').exists():cases['time_reference_fresh_0001']=p

def project(xy, reference):
    starts=reference[:-1,:2];deltas=np.diff(reference[:,:2],axis=0)
    lengths=np.linalg.norm(deltas,axis=1);s=np.r_[0,np.cumsum(lengths)]
    fraction=np.clip(((xy[:,None,:]-starts[None,:,:])*deltas[None,:,:]).sum(axis=2)/np.maximum(lengths[None,:]**2,1e-12),0,1)
    projected=starts[None,:,:]+fraction[:,:,None]*deltas[None,:,:]
    dist=np.linalg.norm(xy[:,None,:]-projected,axis=2);ix=np.argmin(dist,axis=1)
    return s[ix]+fraction[np.arange(len(xy)),ix]*lengths[ix],dist[np.arange(len(xy)),ix]

out={}
for name,root in cases.items():
 m=json.loads((root/'metrics.json').read_text());s=np.load(root/'states.npz');a=np.load(root/'actions.npz');audit=json.loads((root/'collision_audit.json').read_text())
 xy=s['position_world_xy'];reference=s['kinodynamic_reference_pose'];progress,cte=project(xy,reference)
 contacts=[x for x in audit['samples'] if x['contact']]
 assert all(x['available'] for x in audit['samples'])
 assert m['evaluation_protocol_id']=='ball_pointgoal_v1_2_contact_35s'
 assert np.array_equal(s['physical_contact'],s['collision'])
 out[name]=dict(termination=m['termination_reason'],duration_s=float(s['simulation_time_s'][-1]),final_distance_m=m['final_distance_m'],actual_path_length_m=m['path_length_m'],reference_length_m=float(np.linalg.norm(np.diff(reference[:,:2],axis=0),axis=1).sum()),nearest_reference_progress_m=float(progress[-1]),cross_track_rms_m=float(np.sqrt(np.mean(cte**2))),cross_track_peak_m=float(np.max(cte)),minimum_map_clearance_m=m['minimum_clearance_m'],map_nonpositive_samples=m['map_nonpositive_clearance_samples'],contact_count_states=int(s['physical_contact'].sum()),first_contact=contacts[0] if contacts else None,action_blocks=m['action_blocks'],initial_pose_xy=xy[0].tolist(),reference_sha256=__import__('hashlib').sha256(reference.tobytes()).hexdigest(),main_requests=np.unique(a['action_blocks'][:,0],return_counts=True)[0].tolist())
for name in ('speed_change_0002','lookahead_change_0002'):
 assert out[name]['reference_sha256']==out['time_reference_0002']['reference_sha256']
 assert out[name]['initial_pose_xy']==out['time_reference_0002']['initial_pose_xy']
for n in ['identity_after.json','moderate_valid_identity_after.json','lookahead_identity_after.json']:
 assert all(json.loads((h/n).read_text()).values()),n
(h/'assessment.json').write_text(json.dumps(out,indent=2))
for name,r in out.items():print(name,r['termination'],round(r['final_distance_m'],3),'progress',round(r['nearest_reference_progress_m'],3),'RMS',round(r['cross_track_rms_m'],3),'contact',r['first_contact'])
