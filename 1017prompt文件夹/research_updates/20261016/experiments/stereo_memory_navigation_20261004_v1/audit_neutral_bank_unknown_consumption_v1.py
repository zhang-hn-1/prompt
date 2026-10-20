"""Current-only reconstruction sealed before offline obstacle proxy evaluation."""
from pathlib import Path
import numpy as np,json,hashlib,datetime
from reference_continuity_audit import reference_geometry_status
from route_geometry_audit import hazard_clearance
from point_support_shadow import swept_indices
H=Path(__file__).resolve().parent;D=H/'neutral_bank_visibility_run_v1';A=H/'neutral_bank_unknown_consumption_v1';A.mkdir(exist_ok=False)
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest();states=json.loads((D/'states.json').read_text());records=json.loads((D/'policy_records.json').read_text());rows=[]
for i,r in enumerate(records):
 if not r['local']['observation_handed_off']:continue
 m=np.load(D/'maps'/f'frame_{i:03d}.npz');plan=np.load(D/'local_plans'/f'frame_{i:03d}.npz');path=plan['pose'][:,:2];xy=np.array(states[i]['position'][:2]);res=float(m['resolution']);guard=reference_geometry_status(xy,path,m['labels'],m['origin'],res)
 # Frozen .6m planned-clearance disk is a coverage diagnostic, not a new gate.
 cells=swept_indices(guard['suffix_xy'],m['origin'],res,.6);inside=((cells>=0)&(cells<np.asarray(m['labels'].shape))).all(1);ci=cells[inside];n=len(ci)
 rows.append(dict(frame=i,time_s=float(m['time_s']),reference_version=int(plan['version']),guard=guard,inside_footprint_cells=n,footprint_unknown_cells=int((m['labels'][ci[:,0],ci[:,1]]<0).sum()),footprint_observed_floor_cells=int((m['labels'][ci[:,0],ci[:,1]]==0).sum()),footprint_observed_hazard_cells=int((m['labels'][ci[:,0],ci[:,1]]==1).sum()),safe_start_prior_cells=int(m['safe_start_prior'][ci[:,0],ci[:,1]].sum()),task_ground_prior_cells=int(m['task_ground_prior'][ci[:,0],ci[:,1]].sum()),actual_final_request=r['local']['request'],probe_active=r['local']['visibility_probe']['active']))
p=A/'current_only_consumer_measurements.json';p.write_text(json.dumps(dict(rows=rows,GT_read=False,code_sha256=sha(__file__)),indent=2)+'\n');(A/'prediction_seal.json').write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),sha256=sha(p),no_future_or_renderer_labels_read=True),indent=2)+'\n')
# Evaluation-only geometry has never participated in the reconstruction above.
g=np.load(D/'offline_occupancy_label.npz');haz=g['origin']+np.argwhere(g['labels']>0)*float(g['resolution']);evals=[]
for r in rows:
 guard=r['guard'];evals.append(dict(frame=r['frame'],time_s=r['time_s'],observed_necessary_screen_pass=guard['retention_necessary_screen_pass'],suffix_offline_proxy_clearance_m=hazard_clearance(guard['suffix_xy'],haz),bridge_offline_proxy_clearance_m=hazard_clearance(guard['bridge_xy'],haz),projection_distance_m=guard['projection_distance_m']))
bad=[e for e in evals if e['observed_necessary_screen_pass'] and e['suffix_offline_proxy_clearance_m']<.6];moving=[r for r in rows if np.any(np.array(r['actual_final_request'])[::2]!=0)]
out=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),diagnostic_only=True,not_bug_proof=True,not_online_AB=True,not_new_controller_or_gate=True,all_frames=evals,frames=len(rows),moving_frames=len(moving),observed_necessary_screen_passes=sum(r['guard']['retention_necessary_screen_pass'] for r in rows),point_map_screen_pass_but_proxy_suffix_under06=len(bad),all_current_only_swept_footprints_without_observed_floor=all(r['footprint_observed_floor_cells']==0 for r in rows),first_screen_proxy_disagreement=bad[0] if bad else None,old_source_and_navigation_rejects_preserved=True)
(A/'offline_evaluation_v1.json').write_text(json.dumps(out,indent=2)+'\n');print({k:v for k,v in out.items() if k!='all_frames'})
