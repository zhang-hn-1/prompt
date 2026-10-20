from pathlib import Path
import json,hashlib,datetime,sys
import numpy as np
H=Path(__file__).resolve().parent;P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm');sys.path.insert(0,str(P))
from execution_snapshot import ExecutionClone
from bank_response_measurement import schedule
from roll_pid import bank_from_quaternion
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
p=json.loads((H/'bank_response_protocol.json').read_text());status=json.loads((H/'bank_response_status.json').read_text());out=H/'bank_response_run';manifest=json.loads((out/'artifact_manifest_post_close.json').read_text())['artifact_sha256']
for group in ['source_sha256','input_sha256']:
 for file,digest in p[group].items():assert sha(file)==digest,file
for file,digest in manifest.items():assert sha(file)==digest,file
assert p['sealed_utc']<status['start_utc']<status['end_utc'];assert status['exit_code']==0
z=np.load(out/'physics.npz');req=np.load(out/'requests.npz')['requests'];np.testing.assert_array_equal(req,schedule());assert len(z['simulation_time_s'])==1601
np.testing.assert_allclose(z['simulation_time_s'],np.arange(1601)*.02,atol=1e-6,rtol=0)
initial=json.loads((out/'initial_execution_after_reset.json').read_text());assert initial['pid_config']==p['pid_config'] and initial['inner_config']==p['inner_config'];clone=ExecutionClone.from_snapshot(initial)
feedback=json.loads(next((out/'feedback').glob('*.json')).read_text());assert len(feedback)==1600
for i in range(1600):
 raw=req.reshape(-1,2)[i];np.testing.assert_array_equal(raw,z['raw_command_target'][i+1])
 applied,smooth=clone.step(raw,bank_rad=bank_from_quaternion(z['root_quaternion_world_wxyz'][i]),aux_position_rad=z['joint_position_rad'][i,1],aux_velocity_rad_s=z['joint_velocity_rad_s'][i,1])
 np.testing.assert_array_equal(applied,z['applied_action'][i+1]);np.testing.assert_array_equal(smooth,z['smoothed_target'][i+1])
 assert abs(clone.pid.last['bank_rad']-feedback[i]['bank_rad'])<1e-10
 assert abs(clone.pid.last['desired_bank_rad']-feedback[i]['desired_bank_rad'])<1e-12
 # Measured feedback reconstruction, never recursive physics prediction.
clocks=json.loads((out/'camera_clocks.json').read_text());states=json.loads((out/'states.json').read_text());assert len(clocks)==len(states)==321
for i,c in enumerate(clocks):
 assert c['frame']==i and abs(c['episode_time_s']-.1*i)<1e-6 and c['markers_hidden']
 assert all(abs(t-c['raw_time_s'])<=1e-5 for t in c['eyes'].values())
 np.testing.assert_array_equal(states[i]['position'][:2],z['position_world_xy'][i*5])
 if i:
  np.testing.assert_array_equal(json.loads((out/'execution'/f'frame_{i:03d}.json').read_text())['inner']['current_action'],z['applied_action'][i*5])
 assert (out/'history'/f'frame_{i:03d}.npz').exists()
assert len(list((out/'frames').glob('*.npz')))==65 and len(list((out/'execution').glob('*.json')))==321
import cv2
videos=[]
for f in sorted(out.glob('*.mp4')):
 cap=cv2.VideoCapture(str(f));n=0
 while True:
  ok,frame=cap.read()
  if not ok:break
  n+=1
 fps=cap.get(cv2.CAP_PROP_FPS);cap.release();assert n==321 and abs(fps-10)<1e-6;videos.append(dict(file=str(f),decoded_frames=n,fps=fps))
contacts=json.loads((out/'physics_contacts.json').read_text());assert len(contacts)==1921 and all(v['available'] for v in contacts);contactcount=sum(v['contact'] for v in contacts)
# descriptive event measurement follows frozen historical definitions; not an extra admission gate.
banks=np.asarray([bank_from_quaternion(z['root_quaternion_world_wxyz'][i*5]) for i in range(321)]);speeds=np.asarray([np.linalg.norm(z['linear_velocity_world_m_s'][i*5,:2]) for i in range(321)])
reached=None;count=0
for i in range(60,260):
 count=count+1 if abs(banks[i]+np.pi/8)<=np.radians(2) else 0
 if count==5:reached=i/10;break
stop=None;count=0
for i in range(260,321):
 count=count+1 if speeds[i]<.05 else 0
 if count==5:stop=i/10;break
peak=float(np.max(speeds[260:]));radial=float(np.max(np.linalg.norm(z['position_world_xy'][1300:]-z['position_world_xy'][1300],axis=1)))
r=dict(verified_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),source_checks=len(p['source_sha256']),input_checks=len(p['input_sha256']),postclose_artifacts=len(manifest),actual_requests=320,physics_samples=1601,captures=321,rgb_samples=65,complete_execution_snapshots=321,feedback_execution_ticks_exact=1600,measured_feedback_reconstruction_only=True,not_recursive_physical_prediction=True,camera_clock_markers_physics_alignment=True,decoded_videos=videos,contact_samples=1921,physical_contacts=int(contactcount),historical_target_reached_definition='actualbank within2deg desired five consecutive10Hz duringactive',target_reached_time_s=reached,stop_qualified_time_s=stop,stop_rule='five consecutive current10Hz XYspeeds<.05, resetcount onrebound',stop_tail_xy_peak_speed_m_s=peak,stop_tail_radial_peak_m=radial,assessment_sha256=sha(H/'bank_response_assessment.json'),new_physical_sequences=1,no_fit_or_control_candidate=True,navigation_admitted=False,whole_research_complete=False,automation='ACTIVE')
(H/'bank_response_verification.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
