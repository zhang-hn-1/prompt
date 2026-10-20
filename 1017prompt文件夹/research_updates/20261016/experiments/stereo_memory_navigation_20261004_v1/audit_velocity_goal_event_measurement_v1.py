"""Symmetric closed-run endpoint geometry audit; never revises old gate."""
from pathlib import Path
import numpy as np,json,datetime
H=Path(__file__).resolve().parent;rows={};ref=np.load(H/'fixed_pursuit_reference_v1.npz')['pose'][:,:2];L=float(np.linalg.norm(np.diff(ref,axis=0),axis=1).sum());end={}
for v in ['B0','velocity']:
 a=json.loads((H/('velocity_fixed_reference_'+v+'_assessment_v1.json')).read_text());s=json.loads((H/('velocity_fixed_reference_'+v+'_run_v1/states.json')).read_text());rows[v]=(a,s)
common=min(len(x[1]) for x in rows.values())-1
for v,(a,s) in rows.items():
 xy=np.array(s[-1]['position'][:2]);delta=xy-ref[-1];tangent=(ref[-1]-ref[-2])/np.linalg.norm(ref[-1]-ref[-2]);along=-float(delta@tangent);lateral=abs(float(np.cross(tangent,delta)));radius=float(np.linalg.norm(delta));reconstructed=L-np.sqrt(max(0,radius**2-lateral**2));assert np.isclose(reconstructed,a['reference_progress_end_m'],atol=1e-14)
 end[v]=dict(endpoint_distance_m=radius,lateral_to_last_segment_m=lateral,remaining_along_segment_m=along,progress_reconstructed_from_endpoint_geometry_m=float(reconstructed),progress_at_common_frame_m=a['reference_progress_series'][common],end_time_s=s[-1]['time_s'])
result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),common_frame=common,common_time_s=rows['velocity'][1][common]['time_s'],arms=end,common_time_candidate_minus_B0_progress_m=end['velocity']['progress_at_common_frame_m']-end['B0']['progress_at_common_frame_m'],old_frozen_progress_gate_reject_preserved=True,no_new_gate_or_reranking=True,post_goal_physical_stop_not_measured_in_original_runs=True)
(H/'velocity_goal_event_measurement_audit_v1.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
