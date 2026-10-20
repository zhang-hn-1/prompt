"""Causal packet-prefix audit of the frozen history consumer, not a live rerun."""
import hashlib,json
from pathlib import Path
import numpy as np
from provenance_grid import ProvenanceGrid
H=Path(__file__).resolve().parent
R=H/'online_provenance_runs/history'
records=json.loads((R/'vision_records.json').read_text())
obs=json.loads((H/'observation_assessment.json').read_text())
stop_tail=max(obs[n]['post_request_stop_path_m'] for n in ('bank','forward','reverse'))
speed_peak=max(obs[n]['speed_peak_m_s'] for n in ('bank','forward','reverse'))
b=ProvenanceGrid(np.asarray(records[0]['position_world_xy'])-20.)
rows=[]
for i,r in enumerate(records):
    packet=np.load(R/'privileged_sensor_packets'/f'frame_{i:03d}.npz')
    b.update_points(packet['points_world'],packet['root_xyz'],float(packet['time_s']))
    xy=np.asarray(r['position_world_xy']);occ=b.origin+np.argwhere(b.labels==1)*b.resolution
    nearest=float(np.linalg.norm(occ-xy,axis=1).min()) if len(occ) else None
    target=r['target_base_xy'];curv=None if target is None else float(2*target[1]/np.dot(target,target))
    rows.append(dict(index=i,time_s=r['time_s'],nonzero=bool(np.any(r['request'])),target_pp_curvature_1_m=curv,
        nearest_observed_hazard_cell_center_m=nearest,actual_speed_xy_m_s=r['speed_xy'],
        beyond_empirical_observation_speed=bool(r['speed_xy']>speed_peak),
        stop_disc_hits_observed_hazard=bool(nearest is not None and nearest<=.5+np.sqrt(2)*b.resolution/2+stop_tail),
        aux_at_protocol_boundary=bool(abs(r['request'][1])>=np.pi/6-1e-6)))
active=[r for r in rows if r['nonzero']];curv=np.array([r['target_pp_curvature_1_m'] for r in active]);
out=dict(role='development audit, no new navigation result/no proof of universal curvature or stopping bound',
    physical_tail_source='existing complete12s physical observation sequences; image-derived geometry not used',
    empirical_stop_tail_m=stop_tail,empirical_speed_peak_m_s=speed_peak,
    stop_disc_radius_m=.5+np.sqrt(2)*.1/2+stop_tail,
    active_requests=len(active),aux_boundary_fraction=np.mean([r['aux_at_protocol_boundary'] for r in active]),
    pure_pursuit_demand_abs_quantiles=np.quantile(abs(curv),[0,.5,.9,1]).tolist(),
    demand_above_approximate_historical_point8_fraction=float(np.mean(abs(curv)>.8)),
    historical_bank_inverse_scope='steady response from different main speeds; current speed frequently below calibrated minimum. Cannot certify current response or install it unchanged',
    actual_speed_above_empirical_observation_peak=sum(r['beyond_empirical_observation_speed'] for r in active),
    first_stop_disc_veto=next((r for r in rows if r['nonzero'] and r['stop_disc_hits_observed_hazard']),None),
    rows=rows,source_sha256={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in (R/'vision_records.json',H/'observation_assessment.json',Path(__file__))})
(H/'local_execution_gap.json').write_text(json.dumps(out,indent=2)+'\n')
print({k:v for k,v in out.items() if k not in ('rows','source_sha256')})
