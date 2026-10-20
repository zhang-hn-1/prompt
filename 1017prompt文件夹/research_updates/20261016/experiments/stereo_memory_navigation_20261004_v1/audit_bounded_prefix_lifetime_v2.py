"""Postclosed realized partial-reference lifetime, not a counterfactual prediction label."""
from pathlib import Path
import json,hashlib
import numpy as np
from current_reference_epoch_geometry_v1 import project
H=Path(__file__).resolve().parent

def main():
 d=H/'bounded_prefix_e1_candidate_run_v1';rows=json.loads((d/'policy_records.json').read_text());states=json.loads((d/'states.json').read_text());events=[r for r in rows if r.get('bounded_prefix_decision',{}).get('intervention')]
 result=dict(event_count=len(events),whole_mission_complete=False,no_new_solver_or_fit_or_physics=True,not_model_prediction_accuracy=True)
 if events:
  assert len(events)==1
  event=events[0];f=event['frame'];reference=event['current_reference_waypoints_after_act'];end=f+1
  while end<len(rows) and rows[end]['current_reference_waypoints_after_act']==reference:end+=1
  selected=event['bounded_prefix_decision']['selected'];path=selected['samples_xy'];points=np.asarray([states[i]['position'][:2] for i in range(f,end+1)]);axis=np.asarray(event['bounded_prefix_decision']['goal_axis_world']);g=[project(path,p) for p in points]
  result.update(first_frame=f,first_time_s=event['time_s'],last_request_frame_with_reference=end-1,end_state_frame=end,reference_duration_request_s=(end-f)*.1,nominal_axis_progress_m=selected['goal_axis_progress_m'],realized_axis_progress_m=float((points[-1]-points[0])@axis),realized_travel_m=float(np.linalg.norm(np.diff(points,axis=0),axis=1).sum()),sampled_primitive_polyline_cross_rms_m=float(np.sqrt(np.mean([v['distance_m']**2 for v in g]))),sampled_primitive_polyline_cross_peak_m=max(v['distance_m'] for v in g),minimum_terminal_distance_m=min(v['endpoint_distance_m'] for v in g),reference_end_reason='episode_end' if end==len(rows) else 'new_reference_after_unchanged_parent_guard',next_reference_decision=None if end==len(rows) else rows[end]['reference_decision'],old_local_target_reached_claim=False,partial_only=True,projection_geometry='selected nominal dense sampled primitive polyline, not installed three-waypoint reference chords',capture_window='frame120 current root through frame125 pre-state, includes exit state at first new-reference act; not epoch120..124 tracking RMS',installed_reference_metric_source='bounded_prefix_current_reference_epochs_v1.json')
 p=H/'bounded_prefix_realized_lifetime_v2.json';assert not p.exists();p.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
if __name__=='__main__':main()
