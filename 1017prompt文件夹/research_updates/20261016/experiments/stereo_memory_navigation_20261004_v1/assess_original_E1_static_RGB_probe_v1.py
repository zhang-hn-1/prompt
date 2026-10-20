from pathlib import Path
import json,hashlib,datetime,numpy as np,cv2
H=Path(__file__).resolve().parent;D=H/'original_E1_static_RGB_probe_run_v1';sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();status=json.loads((H/'original_E1_static_RGB_probe_status_v1.json').read_text());assert status['exit_code']==0 and status['stage']=='ACTUAL_CLOSED_NEEDS_VERIFICATION' and not status['wall_timeout'];seal=D/'artifact_manifest_post_close.json';assert sha(seal)==status['post_close_manifest_sha256']
for f,s in json.loads(seal.read_text())['artifact_sha256'].items():assert sha(f)==s,f
cfg=json.loads((H/'original_E1_static_RGB_probe_protocol_v1.json').read_text())
for g in ['source_sha256','input_sha256']:
 for f,s in cfg[g].items():assert sha(f)==s,f
identity=json.loads((D/'identity.json').read_text());assert identity['complete'] and identity['no_requests'] and identity['no_NN'] and identity['actual_render_only_snapshots']==24
frames=sorted(D.glob('render_*.npz'));assert len(frames)==24;initial=np.load(frames[0]);adjacent=[];depth=True;pose=True;clock=True
for i,f in enumerate(frames):
 current=np.load(f);previous=np.load(frames[max(0,i-1)]);r=dict(render_index=i,eyes={})
 for eye in ['left','right']:
  clock &= float(current[eye+'_rendering_time'])==identity['clock'][2]
  depth &= np.array_equal(current[eye+'_depth_m'],initial[eye+'_depth_m'],equal_nan=True);pose &= np.array_equal(current[eye+'_camera_usd_world'],initial[eye+'_camera_usd_world'])
  delta=np.abs(current[eye+'_rgb'].astype(float)-previous[eye+'_rgb'].astype(float));r['eyes'][eye]=dict(adjacent_RGB_exact=bool(np.array_equal(current[eye+'_rgb'],previous[eye+'_rgb'])),RGB_changed_channel_fraction=float((delta>0).mean()),RGB_absolute_mean=float(delta.mean()),RGB_absolute_p90=float(np.percentile(delta,90)))
 adjacent.append(r)
cap=cv2.VideoCapture(str(D/'stereo_static_render_sequence.mp4'));n=0
while cap.read()[0]:n+=1
cap.release();assert n==24
changed=sum(not x['adjacent_RGB_exact'] for r in adjacent[1:] for x in r['eyes'].values());a=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),decision='PROVEN_RENDER_ONLY_RGB_VARIATION_AT_FROZEN_PHYSICS_CLOCK' if changed and clock and pose and depth else 'NO_RENDER_ONLY_RGB_VARIATION_PROOF',snapshots=24,RGB_changed_adjacent_eye_pairs=changed,GT_depth_all24_exact=bool(depth),GT_camera_pose_all24_exact=bool(pose),rendering_time_all24_frozen_raw_time=bool(clock),runtime_asserted_physical_state_clock_every_render=True,no_physics_or_commands_after_reset=True,no_NN_or_quality_seed_changes=True,video_decode_frames=n,not_complete_crossrun_cause_proof=True,not_a_fix_or_source_gate=True,known_initial_scene_diagnostic_only=True,whole_research_complete=False,post_close_manifest_sha256=sha(seal))
(H/'original_E1_static_RGB_probe_assessment_v1.json').write_text(json.dumps(a,indent=2)+'\n');(H/'original_E1_static_RGB_probe_delta_rows_v1.json').write_text(json.dumps(adjacent,indent=2)+'\n');print(json.dumps(a,indent=2))
