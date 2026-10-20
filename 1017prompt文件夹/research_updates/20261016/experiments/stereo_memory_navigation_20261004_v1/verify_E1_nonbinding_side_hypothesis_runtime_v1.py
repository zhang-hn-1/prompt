from pathlib import Path
import json,hashlib,datetime,sys
import numpy as np
H=Path(__file__).resolve().parent;P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm');sys.path.insert(0,str(P))
from execution_snapshot import ExecutionClone

sys.path.insert(0,str(H.parent/'oracle_feasible_route_20260929_v9/hybrid'))
from roll_pid import bank_from_quaternion
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
p=json.loads((H/'original_E1_nonbinding_side_hypothesis_navigation_protocol_v1.json').read_text());status=json.loads((H/'original_E1_nonbinding_side_hypothesis_navigation_stage_status_v1.json').read_text());out=H/'original_E1_nonbinding_side_hypothesis_navigation_run_v1';manifest=json.loads((out/'artifact_manifest_post_close.json').read_text())['artifact_sha256']
for group in ['source_sha256','input_sha256']:
 for file,digest in p[group].items():assert sha(file)==digest,file
for file,digest in manifest.items():assert sha(file)==digest,file
assert p['utc']<status['start_utc']<status['end_utc'];assert status['exit_code']==0 and not status['wall_timeout'] and status['stage']=='ACTUAL_CLOSED_NEEDS_VERIFICATION'
z=np.load(out/'physics.npz');req=np.load(out/'requests.npz')['requests'];N=len(req);NT=5*N;NP=NT+1;NC=N+1;NF=N//5+1
assert req.shape==(N,10) and 0<N<=1200
assert not json.loads((out/'native_imu_identity.json').read_text())['predictions_used_by_control']
source_rows=json.loads((out/'source_rows.json').read_text())
assert len(z['simulation_time_s'])==NP
np.testing.assert_array_equal(z['simulation_time_s'],np.arange(NP)*float(np.float32(.02)))
initial=json.loads((out/'initial_execution_after_reset.json').read_text());assert initial['pid_config']==p['pid_config'] and initial['inner_config']==p['inner_config'];clone=ExecutionClone.from_snapshot(initial)
feedback=json.loads(next((out/'feedback').glob('*.json')).read_text());assert len(feedback)==NT
for i in range(NT):
 raw=req.reshape(-1,2)[i];np.testing.assert_array_equal(raw,z['raw_command_target'][i+1])
 applied,smooth=clone.step(raw,bank_rad=bank_from_quaternion(z['root_quaternion_world_wxyz'][i]),aux_position_rad=z['joint_position_rad'][i,1],aux_velocity_rad_s=z['joint_velocity_rad_s'][i,1])
 np.testing.assert_array_equal(applied,z['applied_action'][i+1]);np.testing.assert_array_equal(smooth,z['smoothed_target'][i+1])
 assert abs(clone.pid.last['bank_rad']-feedback[i]['bank_rad'])<1e-10
 assert abs(clone.pid.last['desired_bank_rad']-feedback[i]['desired_bank_rad'])<1e-12
 # Measured feedback reconstruction, never recursive physics prediction.
clocks=json.loads((out/'camera_clocks.json').read_text());states=json.loads((out/'states.json').read_text());assert len(clocks)==len(states)==NC
for i,c in enumerate(clocks):
 assert c['frame']==i and c['episode_time_s']==z['simulation_time_s'][5*i] and c['markers_hidden']
 assert all(abs(t-c['raw_time_s'])<=1e-5 for t in c['eyes'].values())
 np.testing.assert_array_equal(states[i]['position'][:2],z['position_world_xy'][i*5])
 if i:
  np.testing.assert_array_equal(np.asarray(json.loads((out/'execution'/f'frame_{i:03d}.json').read_text())['inner']['current_action'],np.float32),z['applied_action'][i*5])
 assert (out/'history'/f'frame_{i:03d}.npz').exists()
assert len(list((out/'frames').glob('*.npz')))==NF and len(list((out/'execution').glob('*.json')))==NC
import cv2
videos=[]
assert {f.name for f in out.glob('*.mp4')}=={'stereo_view.mp4','third_person.mp4'}
for f in sorted(out.glob('*.mp4')):
 cap=cv2.VideoCapture(str(f));n=0
 while True:
  ok,frame=cap.read()
  if not ok:break
  n+=1
 fps=cap.get(cv2.CAP_PROP_FPS);cap.release();assert n==NC and abs(fps-10)<1e-6;videos.append(dict(file=str(f),decoded_frames=n,fps=fps))
assert [r['frame'] for r in source_rows]==list(range(0,NC,5))
assert len(source_rows)==NF and len(list((out/'source_predictions').glob('*.npz')))==NF
contacts=json.loads((out/'physics_contacts.json').read_text());assert len(contacts)==len(z['simulation_time_s'])+len(clocks)-1 and all(v['available'] for v in contacts);contactcount=sum(v['contact'] for v in contacts)
r=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),source_checks=len(p['source_sha256']),input_checks=len(p['input_sha256']),artifacts=len(manifest),requests=N,physics_samples=NP,captures=NC,rgb_samples=NF,delivered_source_samples=len(source_rows),execution_ticks_exact=NT,videos=videos,contact_samples=len(contacts),physical_contacts=int(contactcount),actor_is_current_GT_known_pose_feedback=True,source_registration_current_GT=True,original35_notrun=True,full_volume_not_admitted=True,whole_research_complete=False,not_whole_causal_AB=True,conditional_gated120_sequence_complete=True,static_scene_assumption=True)
(H/'original_E1_nonbinding_side_hypothesis_sparse_memory_navigation_runtime_verification_v1.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
