"""Post-run provenance and execution evidence; no fitting or changed thresholds."""
from pathlib import Path
import json,hashlib,numpy as np
from bank_mapping import BankMap
from roll_pid import BankPID
h=Path(__file__).resolve().parent;model=BankMap(json.loads((h/'bank_mapping.json').read_text()));out=[]
for main in [-1.0,1.0]:
 p=h/f'validation_{main}';cfg=json.loads((p/'config.json').read_text());assert cfg['main']==main and cfg['mode']=='pid' and cfg['gains']=='.8,.1,.35'
 queries=json.loads((p/'mapping_queries.json').read_text());assert len(queries)==650
 for q in queries:
  expected=model.query(main,q['measured_speed_m_s'],q['requested_curvature_body_1_m'])
  for k in ['bank_rad','encoded_aux_rad','speed_weight']:assert abs(expected[k]-q[k])<1e-12
  block=q['block'];target=0 if block<50 else [.4,-.4,.7,-.7][min(3,(block-50)//150)];assert q['requested_curvature_body_1_m']==target
 fb=json.loads(next(p.glob('bank_feedback_*.json')).read_text());assert len(fb)==3250
 pid=BankPID();blocked=False
 for i,r in enumerate(fb):
  expected_ff=0 if i//5<50 else queries[i//5]['encoded_aux_rad']
  assert abs(r['aux_feedforward_rad']-expected_ff)<1e-7
  pid.step([main,r['aux_feedforward_rad']],r['bank_rad'],downstream_blocked=blocked)
  assert abs(pid.last['pid_protocol_aux_rad']-r['pid_protocol_aux_rad'])<1e-12
  blocked=r['supervisor_active']
 z=np.load(p/'physics.npz');t=z['simulation_time_s'];v=z['linear_velocity_world_m_s'];heading=z['yaw_world_rad'];signed=v[:,0]*np.cos(heading)+v[:,1]*np.sin(heading)
 windows=[]
 for end in [20,35,50,65]:
  mask=(t>=end-5)&(t<=end-.5);indices=np.flatnonzero(mask);records=[fb[i-1] for i in indices if i>0];fraction=float(np.mean(signed[mask]*main>0));assert fraction>=.9
  windows.append(dict(end_s=end,direction_match_fraction=fraction,pid_clip_fraction=float(np.mean([r['pid_clipped'] for r in records])),supervisor_fraction=float(np.mean([r['supervisor_active'] for r in records])),bank_error_rms_deg=float(np.degrees(np.sqrt(np.mean([r['error_rad']**2 for r in records]))))))
 out.append(dict(main=main,current_map_reproduces_all_saved_queries=True,constant_gains_reproduce_all_saved_pid_outputs=True,all_physics_finite=bool(all(np.isfinite(z[k]).all() for k in z.files)),speed_clamp_fraction=float(np.mean([q['speed_clamped'] for q in queries])),windows=windows))
result=dict(runs=out,checked_unique_main_values=[-1.,1.],same_artifact_supported_by_query_replay=True,hashes_after_run={str(h/n):hashlib.sha256((h/n).read_bytes()).hexdigest() for n in ['bank_mapping.json','bank_mapping.py','roll_pid.py','run_validation.py','run_validations.py']},note='Hashes captured after independent runs. Saved causal input/output replay validates mapping and PID behavior; no claim of pre-run signed hash record.')
(h/'validation_provenance.json').write_text(json.dumps(result,indent=2));print('Validated both directions, 1300 mapping queries and6500 PID tick outputs')
cal=[]
for p in sorted(h.glob('calibration_*/physics.npz')):
 fb=json.loads(next(p.parent.glob('bank_feedback_*.json')).read_text());cal.append(dict(case=p.parent.name,pid_clip_fraction=float(np.mean([r['pid_clipped'] for r in fb])),supervisor_fraction=float(np.mean([r['supervisor_active'] for r in fb]))))
(h/'calibration_execution_summary.json').write_text(json.dumps(cal,indent=2))
