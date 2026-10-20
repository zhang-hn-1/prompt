"""Score actual online observer trace separately from actual navigation; no replay."""
from pathlib import Path
import hashlib,json,numpy as np,cv2
from baseline35_metrics_v1 import assess
from local_endpoint_event_v2 import observe_local_endpoint
from benchmark.drive_model import DriveModel
H=Path(__file__).resolve().parent;D=H/'local_endpoint_live_e1_run_v1';sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
 st=json.loads((H/'local_endpoint_live_e1_status_v1.json').read_text());assert st['status']=='finished' and st['exit_code']==0
 sealed=json.loads((D/'artifact_manifest_post_close.json').read_text());assert all(sha(p)==h for p,h in sealed['artifact_sha256'].items());cfg=json.loads((H/'local_endpoint_live_e1_protocol_v1.json').read_text());assert all(sha(p)==h for p,h in cfg['source_sha256'].items());assert all(sha(p)==h for p,h in cfg['input_sha256'].items())
 rows=json.loads((D/'policy_records.json').read_text());native=json.loads((D/'native_predictions.json').read_text());clocks=json.loads((D/'camera_clocks.json').read_text());live=[json.loads(s) for s in (D/'local_endpoint_live_before_act.ndjson').read_text().splitlines()];acts=[r for r in rows if r.get('planner_act_called')];assert len(live)==len(acts);ep=json.loads((D/'task_episode.json').read_text())['actual_episode'];drive=DriveModel.from_json('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/configs/benchmark_v1/drive_dynamics_v1.json');counts={};valid=0;ref_changes=0;stop_to_nonzero=0
 for logged,row in zip(live,acts):
  e=row['local_endpoint_live_before_act'];assert logged==e and abs(e['time_s']-row['time_s'])<1e-8
  i=row['frame'];assert abs(e['time_s']-clocks[i]['episode_time_s'])<1e-8
  if not e['valid']:assert e['reason']=='no_installed_reference';continue
  valid+=1;n=native[i*5];prev=rows[i-1];s=prev['tracker_state_after_act'];snapshot=dict(time_s=e['time_s'],feedback_time_s=n['time_s'],position_xy=n['position_world'][:2],velocity_xy=n['velocity_world_m_s'][:2],reference_xy=prev['installed_reference_xy_after_act'],reference_version=prev['local_reference_version'],path_index=s['path_index'],active_direction=s['active_direction'],pending_direction=s['pending_direction'],braking_state=s['braking_state'],global_goal_xy=ep['goal_position_world_m'][:2]);re=observe_local_endpoint(snapshot,drive.stopping_distance)
  for k,v in re.items():assert e[k]==v,(i,k)
  assert e['created_live_before_policy_act'] and not e['actor_command_issued'];counts[e['event']]=counts.get(e['event'],0)+1
  if e['reference_version']!=row['local_reference_version']:ref_changes+=1
  if e['local_stop_condition'] and any(v!=0. for v in row['request'][:2]):stop_to_nonzero+=1
 states=json.loads((D/'states.json').read_text());contacts=json.loads((D/'physics_contacts.json').read_text());nav=assess(states,[dict(time_s=r['simulation_time_s'],collision=r['contact'],available=r['available']) for r in contacts],ep['start_position_world_m'][:2],ep['goal_position_world_m'][:2]);videos={}
 for name in ['stereo_view','third_person']:
  cap=cv2.VideoCapture(str(D/(name+'.mp4')));fps=cap.get(cv2.CAP_PROP_FPS);n=0
  while True:
   ok,_=cap.read()
   if not ok:break
   n+=1
  cap.release();assert n==len(states) and abs(fps-10)<1e-6;videos[name]=dict(actual_decoded_frames=n,fps=fps)
 result=dict(interface_pass=valid>0,actual_online_before_act=True,valid_current_events=valid,invalid_initial_events=len(live)-valid,event_counts=counts,current_geometry_reconstruction_exact=True,current_request_not_predicted_by_stop_hypothesis=stop_to_nonzero,references_replaced_after_observer=ref_changes,online_shadow_changed_actor=False,structural_no_actor_change_only_not_fullcausal_sameRGB_AB=True,on_off_is_unit_test_not_same_physics_pair=True,no_worldmodel_or_actor_acceptance=True,navigation=nav,videos=videos,whole_mission_complete=False);(D/'local_endpoint_live_assessment_v1.json').write_text(json.dumps(result,indent=2)+'\n');(D/'artifact_manifest_post_assessment_v1.json').write_text(json.dumps(dict(artifact_sha256={str(p):sha(p) for p in D.rglob('*') if p.is_file()}),indent=2)+'\n');print(json.dumps(result,indent=2))
if __name__=='__main__':main()
