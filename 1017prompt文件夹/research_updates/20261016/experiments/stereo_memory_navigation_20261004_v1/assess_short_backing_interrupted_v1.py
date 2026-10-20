"""Preserve interrupted evidence; never call incomplete 35-second run a navigation result."""
from pathlib import Path
import json,hashlib,datetime,numpy as np,cv2
H=Path(__file__).resolve().parent;D=H/'short_initial_backing_e1_run_v1';B=H/'native_pose_baseline35_e1_coverage_run_v1'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
 failure=(D/'failure.txt').read_text();assert 'SystemExit: 143' in failure
 c=json.loads((H/'short_initial_backing_e1_protocol_v1.json').read_text())
 for k in ('source_sha256','input_sha256'):assert all(sha(p)==v for p,v in c[k].items())
 states=json.loads((D/'states.json').read_text());rows=json.loads((D/'policy_records.json').read_text());contacts=json.loads((D/'physics_contacts.json').read_text());recover=json.loads((D/'journal_recovery.json').read_text());q=np.load(D/'requests.npz')['requests'];bq=np.load(B/'requests.npz')['requests'];p=np.load(D/'physics.npz');bp=np.load(B/'physics.npz')
 events=[r for r in rows if r.get('short_initial_backing_this_call')];assert not events
 assert len(states)==len(q)+1 and len(p['simulation_time_s'])==len(q)*5+1
 assert recover['executed_rows']==len(rows) and recover['pending']==[len(rows)]
 n=min(len(q),len(bq));diff=np.flatnonzero(np.any(q[:n]!=bq[:n],axis=1));first=int(diff[0]) if len(diff) else None;length=(n if first is None else first)*5+1
 prefix={k:bool(np.array_equal(p[k][:length],bp[k][:length])) for k in p.files if not k.startswith('raw_') and k not in ('action_block_index','physics_step_in_block')}
 ep=json.loads((D/'task_episode.json').read_text())['actual_episode'];start=np.asarray(ep['start_position_world_m'][:2]);goal=np.asarray(ep['goal_position_world_m'][:2]);end=np.asarray(states[-1]['position'][:2]);axis=(goal-start)/np.linalg.norm(goal-start)
 videos={}
 for name in ('stereo_view','third_person'):
  v=cv2.VideoCapture(str(D/(name+'.mp4')));fps=v.get(cv2.CAP_PROP_FPS);count=0
  while True:
   ok,_=v.read()
   if not ok:break
   count+=1
  v.release();videos[name]=dict(decoded_frames=count,fps=fps)
 result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),status='interrupted_before_candidate_intervention',failure='SIGTERM SystemExit143 inside source capture; close subsequently XIO/pulse abort',origin_of_signal_not_proven=True,actual_process_absent=True,old_running_status_stale_preserved=True,actual_completed_request_intervals=len(q),completed_policy_rows=len(rows),pending_row_not_recovered_as_complete=recover['pending'],captured_states=len(states),physics_ticks=len(p['simulation_time_s']),contact_checks=len(contacts),contacts_available=all(x['available'] for x in contacts),actual_contacts=sum(bool(x['contact']) for x in contacts),last_capture_s=states[-1]['time_s'],partial_true_goal_distance_m=float(np.linalg.norm(end-goal)),partial_true_axis_progress_m=float((end-start)@axis),candidate_events=len(events),candidate_assessed=False,local_gate_pass=None,formal_navigation_result=None,original35_not_completed=True,first_request_difference_frame=first,nonraw_physics_before_request_fork_exact=prefix,videos=videos,no_retry=True,whole_mission_complete=False)
 (D/'interrupted_assessment_v1.json').write_text(json.dumps(result,indent=2)+'\n');(D/'artifact_manifest_after_interruption_v1.json').write_text(json.dumps(dict(artifact_sha256={str(x):sha(x) for x in D.rglob('*') if x.is_file()}),indent=2)+'\n');print(json.dumps(result,indent=2))
if __name__=='__main__':main()
