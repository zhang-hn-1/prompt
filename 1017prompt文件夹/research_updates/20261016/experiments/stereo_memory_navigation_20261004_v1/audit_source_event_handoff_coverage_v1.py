"""Bounded diagnosis: actual current map versus immutable reliable seed retention.

No control counterfactual labels. The retained overlay keeps original 10s birth;
it is an observation memory, never a complete obstacle or floor prior.
"""
from pathlib import Path
import json,datetime
import numpy as np
from point_support_shadow import PointSupportMap,swept_indices
from dense_source_points import records
from reference_continuity_audit import reference_geometry_status
H=Path(__file__).resolve().parent
assessment=json.loads((H/'source_event_handoff_assessment_v1.json').read_text());output=[]
for arm in assessment['arms']:
 a=arm['arm'];d=H/f'source_event_handoff_{a}_run_v1';seednp=np.load(d/'qualified_sensor_packets'/f"source_{arm['seed_frame']:03d}.npz");seed={k:seednp[k] for k in seednp.files};select=(seed['world'][:,2]>.05)&(seed['world'][:,2]<=1.5)
 for k in ['world','sigma_m','query_pixel_id','partner_pixel_id']:seed[k]=seed[k][select]
 states=json.loads((d/'states.json').read_text());rows=[];birth=float(seed['time_s'])
 for r in arm['geometry_trace']:
  i=r['frame'];m=np.load(d/'maps'/f'frame_{i:03d}.npz');t=float(m['time_s']);xy=np.asarray(states[i]['position'])[:2];p=np.load(d/'local_plans'/f'frame_{i:03d}.npz')['pose'][:,:2]
  retain=PointSupportMap(m['origin'],m['labels'].shape,float(m['resolution']));retain.update(records(seed),t,0.);s=retain.snapshot(t);merged=m['labels'].copy();merged[s['labels']==1]=1
  overlay=reference_geometry_status(xy,p,merged,m['origin'],float(m['resolution']));current=r['observed_reference'];proxy=r['offline_proxy_reference'];cells=swept_indices(current['suffix_xy'],m['origin'],float(m['resolution']),.6);inside=((cells>=0)&(cells<np.asarray(m['labels'].shape))).all(1);c=cells[inside];labels=m['labels'][c[:,0],c[:,1]]
  rows.append(dict(frame=i,time_s=t,seed_age_s=t-birth,unexpired_seed_hazard_cells=int((s['labels']==1).sum()),actual_hazard_cells=int((m['labels']==1).sum()),current_screen_pass=current['retention_necessary_screen_pass'],immutable_seed_overlay_screen_pass=overlay['retention_necessary_screen_pass'],immutable_seed_overlay_suffix_clearance_m=overlay['suffix_clearance_m'],proxy_suffix_clearance_m=proxy['suffix_clearance_m'],projection_error_m=proxy['projection_distance_m'],inside_swept_cells=len(c),outside_swept_cells=int((~inside).sum()),observed_floor_cells=int((labels==0).sum()),observed_hazard_cells=int((labels==1).sum()),unknown_cells=int((labels<0).sum()),unknown_fraction=float((labels<0).mean()) if len(c) else None))
 first_empty=next((r for r in rows if r['actual_hazard_cells']==0),None);collision=json.loads((d/'physics_contacts.json').read_text());first_contact=next((r['simulation_time_s'] for r in collision if r['contact']),None)
 output.append(dict(arm=a,seed_birth_s=birth,seed_expiry_s=birth+10,first_empty_actual_hazard_map=first_empty,first_contact_s=first_contact,first_proxy_unsafe=next((r for r in rows if r['proxy_suffix_clearance_m'] is not None and r['proxy_suffix_clearance_m']<.6),None),overlay_still_proxy_unsafe_frames=sum(r['immutable_seed_overlay_screen_pass'] and r['proxy_suffix_clearance_m'] is not None and r['proxy_suffix_clearance_m']<.6 for r in rows),all_posthandoff_unknown_swept_fraction_min=min(r['unknown_fraction'] for r in rows if r['unknown_fraction'] is not None),maximum_projection_error_m=max(r['projection_error_m'] for r in rows),trace=rows))
result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),arms=output,offline_GT_proxy_only=True,no_new_physics_or_network=True,no_control_action_changed=True,no_overlay_counterfactual_trajectory_claim=True,immutable_original_birth_TTL=True,earlier_source_candidate_REJECT_preserved=True,whole_research_complete=False)
out=H/'source_event_handoff_coverage_audit_v1.json';assert not out.exists();out.write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps([{k:v for k,v in a.items() if k!='trace'} for a in output],indent=2))
