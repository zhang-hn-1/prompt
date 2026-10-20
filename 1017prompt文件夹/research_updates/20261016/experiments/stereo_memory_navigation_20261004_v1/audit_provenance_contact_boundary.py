"""Offline causal boundary audit; no extra act(), controller change or simulation."""
import json,hashlib
from pathlib import Path
import numpy as np
from scipy.ndimage import distance_transform_edt
from provenance_grid import ProvenanceGrid
H=Path(__file__).resolve().parent
output={'role':'retrospective contact/stop/map-consumer audit; not runtime predictor input','arms':{}}
for mode in ('current','history'):
    root=H/'online_provenance_runs'/mode;records=json.loads((root/'vision_records.json').read_text());phys=np.load(root/'physics.npz')
    initial=np.load(root/'belief_snapshots/frame_000.npz');grid=ProvenanceGrid(initial['origin'],memory_mode=mode)
    occ=np.load(root/'offline_occupancy_label.npz');truth_distance=distance_transform_edt(occ['labels']==0)*float(occ['resolution'])
    rows=[];snapshot_identity=[]
    for i,record in enumerate(records):
        p=np.load(root/'privileged_sensor_packets'/f'frame_{i:03d}.npz');grid.update_points(p['points_world'],p['root_xyz'],float(p['time_s']));grid.seed_body(p['root_xyz'][:2])
        snap=root/'belief_snapshots'/f'frame_{i:03d}.npz'
        if snap.exists():
            z=np.load(snap);same=all(np.array_equal(a,b,equal_nan=True) for a,b in [(grid.labels,z['labels']),(grid.memory.floor_time,z['floor_time']),(grid.memory.hazard_time,z['hazard_time']),(grid.provenance(),z['provenance'])]);assert same;snapshot_identity.append(i)
        distances=distance_transform_edt(np.pad(grid.labels!=1,1))[1:-1,1:-1]*grid.resolution
        cell=grid.cells(p['root_xyz'][None])[0];truecell=np.rint((p['root_xyz'][:2]-occ['origin'])/float(occ['resolution'])).astype(int)
        rows.append(dict(time_s=record['time_s'],xy=p['root_xyz'][:2].tolist(),speed_xy=record['speed_xy'],request_main=float(record['request'][0]),route_reason=record['route_reason'],measured_label_clearance_at_root_m=float(distances[tuple(cell)]),frozen_occupancy_center_distance_m=float(truth_distance[tuple(truecell)]),history_occupied=record['historical_occupied']))
    active=[i for i,r in enumerate(records) if np.any(r['request'])];last=active[-1];stop=last+1
    t=np.asarray(phys['simulation_time_s']);xy=phys['root_position_world_m'][:,:2]
    contacts=[s for s in json.loads((root/'episode_golden_e1_0001/collision_audit.json').read_text())['samples'] if s.get('contact')]
    ct=contacts[0]['simulation_time_s'];start=rows[stop]['time_s'] if stop<len(rows) else None
    tail=None
    if start is not None:
        begin=int(np.searchsorted(t,start-1e-5));end=int(np.searchsorted(t,ct-1e-5));segment=xy[begin:end+1]
        tail=dict(first_final_hold_request_s=start,first_scored_contact_s=ct,elapsed_to_contact_s=ct-start,path_until_contact_m=float(np.linalg.norm(np.diff(segment,axis=0),axis=1).sum()),net_displacement_to_contact_m=float(np.linalg.norm(segment[-1]-segment[0])),actual_xy_speed_at_hold_m_s=float(np.linalg.norm(phys['linear_velocity_world_m_s'][begin,:2])),main_smoothed_target_at_hold_rad_s=float(phys['smoothed_target'][begin,0]),main_applied_target_at_hold_rad_s=float(phys['applied_action'][begin,0]))
    if tail is not None:
        tail['main_target_at_hold_scope']='physical sample at current observation, before execution of new zero request; not the first applied zero block'
        for key,name in [('smoothed_target','first_main_smoothed_zero_after_hold_s'),('applied_action','first_main_applied_zero_after_hold_s')]:
            indices=np.flatnonzero((t>start+1e-5)&(abs(phys[key][:,0])<1e-6))
            tail[name]=float(t[indices[0]]) if indices.size else None
        count=0;qualified=None
        for k in np.flatnonzero((phys['physics_step_in_block']==5)&(t>=start-1e-5)):
            speed=float(np.linalg.norm(phys['linear_velocity_world_m_s'][k,:2]));count=count+1 if speed<.05 else 0
            if count>=5:qualified=float(t[k]);break
        tail['five_10hz_speed_below_0p05_qualified_before_end_s']=qualified
    output['arms'][mode]=dict(all_saved_live_snapshots_reconstructed_exactly=True,snapshot_indices=snapshot_identity,stop_to_contact=tail,last_5s_rows=[r for r in rows if r['time_s']>=ct-5],minimum_online_measured_label_clearance_at_root_m=min(r['measured_label_clearance_at_root_m'] for r in rows),scope='clearance is to observed2Dpoint-labelled cells, not full visible surface/volume. Frozen occupancy is independent offline proxy only. Final hold means zero raw block, not instant physical stop.')
output['source_sha256']={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [Path(__file__),H/'provenance_grid.py',H/'spatial_memory.py']}
(H/'provenance_contact_boundary.json').write_text(json.dumps(output,indent=2)+'\n')
print(json.dumps({m:{k:v for k,v in a.items() if k not in ('last_5s_rows','snapshot_indices')} for m,a in output['arms'].items()},indent=2))
