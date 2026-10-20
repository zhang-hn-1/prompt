from pathlib import Path
import json,hashlib,datetime,sys
import numpy as np
H=Path(__file__).resolve().parent;P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm');sys.path.insert(0,str(P))
from execution_snapshot import ExecutionClone

from roll_pid import bank_from_quaternion
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
p=json.loads((H/'partial_surface_source_protocol_v1.json').read_text());status=json.loads((H/'partial_surface_source_stage_status_v1.json').read_text());out=H/'partial_surface_source_validation_run_v1';manifest=json.loads((out/'artifact_manifest_post_close.json').read_text())['artifact_sha256']
for group in ['source_sha256','input_sha256']:
 for file,digest in p[group].items():assert sha(file)==digest,file
for file,digest in manifest.items():assert sha(file)==digest,file
assert p['utc']<status['start_utc']<status['end_utc'];assert status['exit_code']==0 and status['stage']=='ACTUAL_CLOSED_NEEDS_VERIFICATION'
z=np.load(out/'physics.npz');req=np.load(out/'requests.npz')['requests'];np.testing.assert_array_equal(req,np.asarray(json.loads(Path(p['command_plan']).read_text()),np.float32));assert len(z['simulation_time_s'])==3001
np.testing.assert_array_equal(z['simulation_time_s'],np.arange(3001)*float(np.float32(.02)))
initial=json.loads((out/'initial_execution_after_reset.json').read_text());assert initial['pid_config']==p['pid_config'] and initial['inner_config']==p['inner_config'];clone=ExecutionClone.from_snapshot(initial)
feedback=json.loads(next((out/'feedback').glob('*.json')).read_text());assert len(feedback)==3000
for i in range(3000):
 raw=req.reshape(-1,2)[i];np.testing.assert_array_equal(raw,z['raw_command_target'][i+1])
 applied,smooth=clone.step(raw,bank_rad=bank_from_quaternion(z['root_quaternion_world_wxyz'][i]),aux_position_rad=z['joint_position_rad'][i,1],aux_velocity_rad_s=z['joint_velocity_rad_s'][i,1])
 np.testing.assert_array_equal(applied,z['applied_action'][i+1]);np.testing.assert_array_equal(smooth,z['smoothed_target'][i+1])
 assert abs(clone.pid.last['bank_rad']-feedback[i]['bank_rad'])<1e-10
 assert abs(clone.pid.last['desired_bank_rad']-feedback[i]['desired_bank_rad'])<1e-12
 # Measured feedback reconstruction, never recursive physics prediction.
clocks=json.loads((out/'camera_clocks.json').read_text());states=json.loads((out/'states.json').read_text());assert len(clocks)==len(states)==601
for i,c in enumerate(clocks):
 assert c['frame']==i and c['episode_time_s']==z['simulation_time_s'][5*i] and c['markers_hidden']
 assert all(abs(t-c['raw_time_s'])<=1e-5 for t in c['eyes'].values())
 np.testing.assert_array_equal(states[i]['position'][:2],z['position_world_xy'][i*5])
 if i:
  np.testing.assert_array_equal(np.asarray(json.loads((out/'execution'/f'frame_{i:03d}.json').read_text())['inner']['current_action'],np.float32),z['applied_action'][i*5])
 assert (out/'history'/f'frame_{i:03d}.npz').exists()
assert len(list((out/'frames').glob('*.npz')))==121 and len(list((out/'execution').glob('*.json')))==601
import cv2
videos=[]
for f in sorted(out.glob('*.mp4')):
 cap=cv2.VideoCapture(str(f));n=0
 while True:
  ok,frame=cap.read()
  if not ok:break
  n+=1
 fps=cap.get(cv2.CAP_PROP_FPS);cap.release();assert n==601 and abs(fps-10)<1e-6;videos.append(dict(file=str(f),decoded_frames=n,fps=fps))
contacts=json.loads((out/'physics_contacts.json').read_text());assert len(contacts)==3601 and all(v['available'] for v in contacts);contactcount=sum(v['contact'] for v in contacts)
r=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),source_checks=len(p['source_sha256']),input_checks=len(p['input_sha256']),artifacts=len(manifest),requests=600,physics_samples=3001,captures=601,rgb_samples=121,execution_ticks_exact=3000,videos=videos,contact_samples=3601,physical_contacts=int(contactcount),actor_is_preknown_requests=True,not_navigation=True,not_fresh_heldout=True,whole_research_complete=False)
(H/'partial_surface_source_runtime_verification_v1.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
