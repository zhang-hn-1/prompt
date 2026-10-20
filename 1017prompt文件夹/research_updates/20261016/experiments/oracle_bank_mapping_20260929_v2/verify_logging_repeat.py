from pathlib import Path
import json,numpy as np,cv2
h=Path(__file__).resolve().parent;a=h/'navigation_golden/episode_golden_e1_0001';b=h/'logging_verification/episode_golden_e1_0001'
z=np.load(a/'states.npz');zz=np.load(b/'states.npz');actions=np.load(a/'actions.npz')['action_blocks'];again=np.load(b/'actions.npz')['action_blocks'];assert np.array_equal(actions,again)
keys=['position_world_xy','yaw_world_rad','linear_velocity_world_m_s','angular_velocity_world_rad_s','joint_position_rad','joint_velocity_rad_s','applied_action','physical_contact','collision'];states={k:bool(np.array_equal(z[k],zz[k])) for k in keys};assert all(states.values())
x=json.loads((a/'metrics.json').read_text());y=json.loads((b/'metrics.json').read_text());metric_keys=['success','collision','timeout','elapsed_s','final_distance_m','path_length_m','termination_reason'];assert all(x[k]==y[k] for k in metric_keys)
online=json.loads((h/'logging_verification/feedback/mapping_golden_e1_0001.json').read_text());reconstructed=json.loads((h/'navigation_golden/feedback/mapping_reconstructed_golden_e1_0001.json').read_text());assert len(online)==len(reconstructed)==350;errors=[]
for o,r in zip(online,reconstructed):
 assert o['active']==r['active']
 if o['active']:
  for key in ['bank_rad','encoded_aux_rad','measured_speed_m_s','requested_curvature_body_1_m','outer_yaw_correction_rad','new_aux_rad']:
   errors.append(abs(o[key]-r[key]));assert abs(o[key]-r[key])<1e-8,(key,o[key],r[key])
for p in b.glob('*.mp4'):
 cap=cv2.VideoCapture(str(p));n=int(cap.get(cv2.CAP_PROP_FRAME_COUNT));ok,_=cap.read();cap.set(cv2.CAP_PROP_POS_FRAMES,n-1);end,_=cap.read();cap.release();assert n>0 and ok and end
result=dict(same_actions=True,same_measured_states=states,same_core_metrics=True,online_mapping_rows=len(online),offline_mapping_matches_online_max_error=max(errors),videos_readable=True)
(h/'logging_repeat_verification.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
