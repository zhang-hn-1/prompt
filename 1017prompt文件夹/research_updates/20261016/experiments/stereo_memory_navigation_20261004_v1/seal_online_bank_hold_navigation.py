import datetime,hashlib,json
from pathlib import Path
import numpy as np
H=Path(__file__).resolve().parent;r=H/'local_stopping_run';p=np.load(r/'physics.npz');records=json.loads((r/'vision_records.json').read_text())
events=[i for i in range(1,len(records)) if records[i-1]['request'][0]!=0 and records[i]['request'][0]==0]
assert events==[284];i=events[0];j=i*5;time=p['simulation_time_s'];xy=p['position_world_xy'];speed=np.linalg.norm(p['linear_velocity_world_m_s'][:,:2],axis=1)
windows=[]
for horizon in (1.5,3.,6.):
 segment=xy[j:j+round(horizon*50)+1];radial=np.linalg.norm(segment-segment[0],axis=1)
 windows.append(dict(horizon_s=horizon,path_m=float(np.linalg.norm(np.diff(segment,axis=0),axis=1).sum()),radial_peak_m=float(radial.max())))
prev=json.loads((H/'local_stopping_assessment.json').read_text())['candidate']
files=[H/n for n in ['online_bank_hold.py','online_bank_hold_policy.py','run_online_bank_hold_navigation.py','test_online_bank_hold.py','stopping_consumer.py','stopping_policy.py','run_stopping_navigation.py','execution_snapshot.py','provenance_policy.py','provenance_grid.py','spatial_memory.py','capture_transaction.py','render_flush.py','online_provenance_evaluation_reference_e1.npz','local_stopping_assessment.json']]
files.extend(r/n for n in ['physics.npz','vision_records.json','experiment_identity.json','episode_golden_e1_0001/metrics.json'])
protocol=dict(frozen_at_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),max_new_runs=1,candidate_count=1,budget_s=35.,episode='golden_e1_0001',baseline='archived local_stopping_run; no rerun',
  event='after parent act exactly once, previous emitted request has nonzero main and current parent final request has allzero main; no ID/time trigger',
  request_source='hold previous emitted raw moving-request auxiliary values throughout zero-main interval; main unchanged; no measured/applied angle or future actual command input',
  resume='parent nonzero-main passes completely unchanged immediately, clears hold; no wait for qualification or force reversal',
  unchanged=['camera rig/masks/lighting','A* and map/provenance','PID .4/.1/.7','protocol/smoother/supervisor','main policy andcap','direction latch','five-low threshold and reboundreset','old stopping veto radius heuristic'],
  prefix_gate='17 nonraw physical fields+relativeclock+allrequests+commonpolicy/map throughfirstevent exact; baseline completeexecution reconstructed under pastmeasuredfeedback then compareactualcandidate snapshot; raw origin disclosed',
  first_stop_baseline=dict(event_index=i,event_time_s=records[i]['time_s'],qualification_delay_s=prev['actual_five_sample_stop_after_veto_s']-records[i]['time_s'],windows=windows,speed_peak_m_s=float(speed[j:j+301].max()),end_one_second_mean_speed_m_s=float(speed[j+250:j+301].mean())),
  brake_gate='only comparable if main remainszero through6s. Qualify no later/no postqualification rebound before resume; all1.5/3/6path/radial/speedpeak/endmean nonregression; at least10%less6sradial OR.2searlierqualification',
  numeric_tolerances=dict(space_m=1e-5,speed_m_s=1e-5,time_s=1e-4),
  whole_reference_gate=dict(min_final_progress_m=prev['independent_reference_progress_final_m'],max_lateral_rms_m=prev['cross_track_rms_m'],max_lateral_peak_m=prev['cross_track_peak_m'],max_final_goal_distance_m=prev['final_distance_m'],collision_allowed=False,meaningful='at least.05m more finalreferenceprogress OR10%less lateralRMS, while allnonregressionchecks pass'),
  navigation_acceptance='actual original goal reached within35s with no contact; never parking/delaying failure or index progression. Component/whole diagnostic gate not navigation success',
  permissions='renderer depth+truepose diagnostic only; no Oracle/fullmap/reference providedonline; unobservedrisk explicit; old stopbound remainsuncertified',
  stop_rule='one actual sequence only; failure preserved, no gain/radius/network/table scan or rerun',
  source_sha256={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in files})
path=H/'online_bank_hold_navigation_protocol.json';assert not path.exists() and not (H/'online_bank_hold_navigation_run').exists();path.write_text(json.dumps(protocol,indent=2)+'\n');print(json.dumps({k:v for k,v in protocol.items() if k!='source_sha256'},indent=2))
