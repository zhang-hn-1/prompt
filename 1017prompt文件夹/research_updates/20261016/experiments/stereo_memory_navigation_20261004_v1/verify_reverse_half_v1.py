from pathlib import Path
import json,hashlib,datetime,sys,argparse
import numpy as np
H=Path(__file__).resolve().parent;P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm');sys.path.insert(0,str(P))
from execution_snapshot import ExecutionClone
from reverse_half_schedule_v1 import schedule
from roll_pid import bank_from_quaternion
from assess_sgbm_risk_lifetime_v1 import complete_execution_evidence
from scipy.spatial.transform import Rotation
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
parser=argparse.ArgumentParser();parser.add_argument('--arm',required=True,choices=['development','validation']);arm=parser.parse_args().arm;p=json.loads((H/'reverse_half_protocol_v1.json').read_text());status=json.loads((H/('reverse_half_'+arm+'_status_v1.json')).read_text());out=H/('reverse_half_'+arm+'_run_v1');manifest=json.loads((out/'post_close_manifest.json').read_text())['artifact_sha256']
for group in ['source_sha256','input_sha256']:
 for file,digest in p[group].items():assert sha(file)==digest,file
for file,digest in manifest.items():assert sha(file)==digest,file
assert p['utc']<status['start_utc']<status['end_utc'];assert status['child_returncode']==0 and status['status']=='finished'
z=np.load(out/'physics.npz');req=np.load(out/'requests.npz')['requests'];np.testing.assert_array_equal(req,schedule());assert len(z['simulation_time_s'])==1601
np.testing.assert_array_equal(z['simulation_time_s'],np.arange(1601)*float(np.float32(.02)))
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
 assert c['frame']==i and c['episode_time_s']==z['simulation_time_s'][5*i] and c['markers_hidden']
 assert all(abs(t-c['raw_time_s'])<=1e-5 for t in c['eyes'].values())
 np.testing.assert_array_equal(states[i]['position'][:2],z['position_world_xy'][i*5])
 if i:
  np.testing.assert_array_equal(np.asarray(json.loads((out/'execution'/f'frame_{i:03d}.json').read_text())['inner']['current_action'],np.float32),z['applied_action'][i*5])
 assert (out/'history'/f'frame_{i:03d}.npz').exists()
assert len(list((out/'frames').glob('*.npz')))==65 and len(list((out/'execution').glob('*.json')))==321
import cv2
videos=[]
for f in [out/'stereo_view.mp4',out/'third_person.mp4']:
 assert f.exists(),f
 cap=cv2.VideoCapture(str(f));n=0
 while True:
  ok,frame=cap.read()
  if not ok:break
  n+=1
 fps=cap.get(cv2.CAP_PROP_FPS);cap.release();assert n==321 and abs(fps-10)<1e-6;videos.append(dict(file=str(f),decoded_frames=n,fps=fps))
contacts=json.loads((out/'physics_contacts.json').read_text());complete_execution_evidence(states,contacts,z['simulation_time_s'],320);contactcount=sum(v['contact'] for v in contacts)
q=np.asarray(states[0]['quaternion']);yaw=Rotation.from_quat(q[[1,2,3,0]]).as_euler('xyz')[2]
assert abs(np.arctan2(np.sin(yaw-np.radians(p['arms'][arm]['initial_yaw_deg'])),np.cos(yaw-np.radians(p['arms'][arm]['initial_yaw_deg']))))<1e-5,'actual initial yaw'
# descriptive event measurement follows frozen historical definitions; not an extra admission gate.
banks=np.asarray([bank_from_quaternion(z['root_quaternion_world_wxyz'][i*5]) for i in range(321)]);speeds=np.asarray([np.linalg.norm(z['linear_velocity_world_m_s'][i*5,:2]) for i in range(321)])
reached=[]
for start,end,target in [(60,260,0.)]:
 count=0;event=None
 for i in range(start,end):
  count=count+1 if abs(banks[i]-target)<=np.radians(2) else 0
  if count==5:event=i/10;break
 reached.append(dict(request_start_s=start/10,target_bank_rad=target,actual_bank_reached_s=event))
stop=None;count=0
for i in range(260,321):
 count=count+1 if speeds[i]<.05 else 0
 if count==5:stop=i/10;break
peak=float(np.max(speeds[260:]));radial=float(np.max(np.linalg.norm(z['position_world_xy'][1300:]-z['position_world_xy'][1300],axis=1)))
r=dict(verified_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),source_checks=len(p['source_sha256']),input_checks=len(p['input_sha256']),postclose_artifacts=len(manifest),actual_requests=320,physics_samples=1601,captures=321,rgb_samples=65,complete_execution_snapshots=321,feedback_execution_ticks_exact=1600,measured_feedback_reconstruction_only=True,not_recursive_physical_prediction=True,camera_clock_markers_physics_alignment=True,decoded_videos=videos,contact_samples=1921,physical_contacts=int(contactcount),historical_target_reached_definition='actualbank within2deg desired five consecutive10Hz duringactive',target_reached_time_s=reached,stop_qualified_time_s=stop,stop_rule='five consecutive current10Hz XYspeeds<.05, resetcount onrebound',stop_tail_xy_peak_speed_m_s=peak,stop_tail_radial_peak_m=radial,assessment_sha256=sha(H/('reverse_half_'+arm+'_assessment_v1.json')),new_physical_sequences=1,no_fit_or_control_candidate=True,navigation_admitted=False,whole_research_complete=False,automation='ACTIVE')
(H/('reverse_half_'+arm+'_verification_v1.json')).write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
