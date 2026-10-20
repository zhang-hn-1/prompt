"""Bounded posthoc attribution on the common measured physical prefix only."""
from pathlib import Path
import json, hashlib, datetime, copy
import numpy as np
from native_rolling_shadow import NativeRollingShadow
H = Path(__file__).resolve().parent
A = H / 'native_flat_pose_navigation_R1_run_v1'
B = H / 'provided_model_native_identity_B0_run_v1'
O = H / 'provided_model_sensor_prefix_causal_v1'
O.mkdir(exist_ok=False)
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
n = 748
sa = json.loads((A/'native_rolling_sensors.json').read_text())[:n]
sb = json.loads((B/'native_rolling_sensors.json').read_text())[:n]
ra = json.loads((A/'native_imu_reads.json').read_text())[:n]
rb = json.loads((B/'native_imu_reads.json').read_text())[:n]
pa = np.load(A/'physics.npz'); pb = np.load(B/'physics.npz')
physical = ['root_position_world_m','root_quaternion_world_wxyz','linear_velocity_world_m_s','angular_velocity_world_rad_s','joint_position_rad','joint_velocity_rad_s','applied_action','smoothed_target']
assert all(np.array_equal(pa[k][:n],pb[k][:n]) for k in physical)
assert json.loads((A/'initial_gauge.json').read_text()) == json.loads((B/'initial_gauge.json').read_text())
normal = json.loads((A/'native_imu_identity.json').read_text())['initial_normal']
assert normal == json.loads((B/'native_imu_identity.json').read_text())['initial_normal']
def differences(a,b):
 a=np.asarray(a);b=np.asarray(b);d=np.abs(a-b);bad=np.flatnonzero((a!=b).reshape(len(a),-1).any(1))
 return dict(first=int(bad[0]) if len(bad) else None,differing_samples=len(bad),max_absolute=float(d.max()))
channels={k:differences([s[k] for s in sa],[s[k] for s in sb]) for k in sa[0]}
clock={k:differences([r[k] for r in ra],[r[k] for r in rb]) for k in ['time_s','begin_raw_s','end_raw_s','sensor_begin_stamp_s']}
clock['absolute_begin_offset_B_minus_A_s']=float(rb[0]['begin_raw_s']-ra[0]['begin_raw_s'])
clock['relative_begin_exact']=np.array_equal(np.array([r['begin_raw_s'] for r in ra])-ra[0]['begin_raw_s'],np.array([r['begin_raw_s'] for r in rb])-rb[0]['begin_raw_s'])
def replay(s):
 m=NativeRollingShadow()
 for row in s:m.consume(row,initial_normal=normal)
 return {k:np.asarray([r[k] for r in m.records]) for k in ['position','rotation','velocity']}
aa=replay(sa);bb=replay(sb)
for d,rr in [(A,aa),(B,bb)]:
 saved=json.loads((d/'rolling_predictions.json').read_text())[:n]
 assert all(np.array_equal(rr[k],np.array([r[k] for r in saved])) for k in rr)
probes={}
for key in ['gyro_body_rad_s','linear_acceleration_body_m_s2','main_encoder_rad_s','gyro_packet_zero_only']:
 hybrid=copy.deepcopy(sb)
 if key=='gyro_packet_zero_only':hybrid[0]['gyro_body_rad_s']=copy.deepcopy(sa[0]['gyro_body_rad_s'])
 else:
  for i in range(n):hybrid[i][key]=copy.deepcopy(sa[i][key])
 rr=replay(hybrid)
 probes[key]={k:dict(exact_A=bool(np.array_equal(rr[k],aa[k])),exact_B=bool(np.array_equal(rr[k],bb[k])),max_abs_difference_A=float(np.abs(rr[k]-aa[k]).max())) for k in rr}
eligibility={}
for label,s in [('A',sa),('B',sb)]:
 mag=np.linalg.norm([v['linear_acceleration_body_m_s2'] for v in s],axis=1)
 eligible=np.abs(mag-9.81)<=.5
 eligibility[label]=dict(gravity_eligible_count=int(eligible.sum()),first_eligible=int(np.flatnonzero(eligible)[0]) if eligible.any() else None)
eligible_same=np.array_equal(np.abs(np.linalg.norm([v['linear_acceleration_body_m_s2'] for v in sa],axis=1)-9.81)<=.5,np.abs(np.linalg.norm([v['linear_acceleration_body_m_s2'] for v in sb],axis=1)-9.81)<=.5)
r=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),scope='POSTHOC_COMMON_MEASURED_PREFIX_ONLY',samples=n,last_index=747,physical_eight_fields_exact=True,initial_gauge_and_normal_exact=True,channels=channels,clock=clock,gravity_eligibility=eligibility,gravity_eligibility_identical=eligible_same,one_channel_replay_probes=probes,no_actual_repeat=True,no_future_packets=True,no_actor_change=True,no_alternative_trajectory_labels=True,old_identity_reject_preserved=True,whole_research_complete=False,input_sha256={str(d/f):sha(d/f) for d in [A,B] for f in ['native_rolling_sensors.json','native_imu_reads.json','rolling_predictions.json','physics.npz','initial_gauge.json','native_imu_identity.json']},code_sha256=sha(__file__))
(O/'assessment.json').write_text(json.dumps(r,indent=2)+'\n')
print(json.dumps(r,indent=2))
