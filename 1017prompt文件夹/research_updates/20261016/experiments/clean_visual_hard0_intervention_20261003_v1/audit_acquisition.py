"""Independent verification of camera clocks, physical parity, and real outcomes."""
from pathlib import Path
import hashlib,json,numpy as np
H=Path(__file__).parent;E=H.parent
RUNS={**{s:(H/'synchronized_v2'/s,E/'fixed_rig_self_mask_and_curve_qualification_20261003_v1/legacy_clean'/s,'golden_'+s+'_0001') for s in ('e0','e1','e2')},**{s:(H/'synchronized_v2'/s,H/'baseline'/s,s) for s in ('hard_0_e1_0003','hard_0_e1_0004')}}
def summarize_clocks(root):
 v=json.loads((root/'camera_clocks.json').read_text());result={}
 for eye in ('left','right'):
  same=sum(v[i]['eyes'][eye]['rendering_frame']==v[i-1]['eyes'][eye]['rendering_frame'] for i in range(1,len(v)))
  result[eye]={'captures':len(v),'repeated_sdk_frames':same,'max_age_s':max(abs(x['eyes'][eye]['age_s']) for x in v),'stale_captures':sum(abs(x['eyes'][eye]['age_s'])>1e-5 for x in v),'world_sample_dt_s':float(np.median(np.diff([x['raw_physics_time_s'] for x in v]))),'distinct_acquired_intervals_s':np.unique(np.round(np.diff([x['eyes'][eye]['rendering_time_s'] for x in v]),8)).tolist()}
 return result
summary={'scope':'same frozen visual B0, acquisition interface only; all known development episodes','original_timestamp_run':summarize_clocks(H/'acquisition_pair/original'),'runs':{}}
for label,(new,old,episode) in RUNS.items():
 dest=new/f'episode_{episode}';before=old/f'episode_{episode}';metrics=json.loads((dest/'metrics.json').read_text());env=json.loads((dest/'environment.json').read_text());checks=json.loads((new/'marker_visibility_each_capture.json').read_text());contact=json.loads((dest/'collision_audit.json').read_text());identity=json.loads((new/'experiment_identity.json').read_text());parity={}
 for file in ('actions.npz','states.npz','physics.npz'):
  a_path=dest/file if file!='physics.npz' else new/file;b_path=before/file if file!='physics.npz' else old/file
  with np.load(a_path) as a,np.load(b_path) as b:parity[file]={key:bool(np.array_equal(a[key],b[key])) for key in a.files if key in b.files}
 clocks=summarize_clocks(new);records=json.loads((new/'vision_records.json').read_text());invariants=json.loads((new/'initial_render_invariants.json').read_text())
 assert all(c['stale_captures']==0 and c['repeated_sdk_frames']==0 for c in clocks.values())
 assert env['episode_budget_s']==35 and env['collision_mode']=='physical_contact_v2'
 assert all(len(check)==4 and all(x['computed_visibility']=='invisible' for x in check.values()) for check in checks)
 assert all(x['available'] for x in contact['samples'])
 assert identity['privileged'] is False and identity['renderer_depth_online'] is False
 summary['runs'][label]={'episode_id':episode,'termination':metrics['termination_reason'],'elapsed_s':metrics['elapsed_s'],'final_distance_m':metrics['final_distance_m'],'path_length_m':metrics['path_length_m'],'first_contact':next((x for x in contact['samples'] if x['contact']),None),'marker_captures':len(checks),'physics_contact_samples':sum(x['source']=='physics' for x in contact['samples']),'camera_clocks':clocks,'initial_render_invariants':invariants,'comparisons_to_original_clean':parity,'max_left_obstacle_pixels':max(x['obstacle_pixels'] for x in records),'trigger_time_s':next((i*.1 for i,x in enumerate(records) if x['detour_triggered']),None),'policy_model_sha256':identity['model_sha256']}
(H/'acquisition_audit.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
print(json.dumps({k:{a:b for a,b in v.items() if a!='comparisons_to_original_clean'} for k,v in summary['runs'].items()},indent=2))
