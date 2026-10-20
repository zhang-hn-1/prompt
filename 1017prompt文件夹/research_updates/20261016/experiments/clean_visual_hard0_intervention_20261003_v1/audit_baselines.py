from pathlib import Path
import json,numpy as np,hashlib
H=Path(__file__).parent;OLD=H.parent/'fixed_rig_visual_pilot_20261001/vision_detour_feasibility_v1'
summary={}
for episode in ('hard_0_e1_0003','hard_0_e1_0004'):
 root=H/'baseline'/episode;e=root/f'episode_{episode}';old=OLD/episode/f'episode_{episode}'
 metrics=json.loads((e/'metrics.json').read_text());env=json.loads((e/'environment.json').read_text());checks=json.loads((root/'marker_visibility_each_capture.json').read_text());contact=json.loads((e/'collision_audit.json').read_text());records=json.loads((root/'vision_records.json').read_text())
 comparisons={}
 for file in ('actions.npz','states.npz'):
  with np.load(e/file) as a,np.load(old/file) as b:
   comparisons[file]={k:bool(np.array_equal(a[k],b[k])) for k in a.files if k in b.files}
 capture_times=[];prev=None;duplicates=[]
 for i,path in enumerate(sorted((root/'online_inputs').glob('frame_*.npz'))):
  with np.load(path) as f,np.load(root/'offline_truth'/path.name) as truth:
   if prev is not None and np.array_equal(f['left_rgb'],prev[0]):duplicates.append({'index':i,'xy_motion_m':float(np.linalg.norm(f['position_world_xy']-prev[1])),'quaternion_change':float(np.linalg.norm(f['quaternion_world_wxyz_history'][-1]-prev[2])),'depth_duplicate':bool(np.array_equal(truth['left_depth_m'],prev[3]))})
   prev=(f['left_rgb'].copy(),f['position_world_xy'].copy(),f['quaternion_world_wxyz_history'][-1].copy(),truth['left_depth_m'].copy())
 samples=contact['samples'];first=next((x for x in samples if x['contact']),None)
 assert env['episode_budget_s']==35 and env['collision_mode']=='physical_contact_v2'
 assert all(len(x)==4 and all(y['computed_visibility']=='invisible' for y in x.values()) for x in checks)
 assert all(s['available'] for s in samples)
 summary[episode]={'outcome':metrics['termination_reason'],'elapsed_s':metrics['elapsed_s'],'goal_distance_m':metrics['final_distance_m'],'path_length_m':metrics['path_length_m'],'left_max_obstacle_pixels':max(r['obstacle_pixels'] for r in records),'trigger_s':next((i*.1 for i,r in enumerate(records) if r['detour_triggered']),None),'marker_captures':len(checks),'first_contact':first,'physics_contact_samples':sum(x['source']=='physics' for x in samples),'all_contacts_available':True,'comparisons_to_old_run':comparisons,'consecutive_duplicate_frames':duplicates}
(H/'baseline_audit.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps({k:{a:b for a,b in v.items() if a not in ('comparisons_to_old_run','consecutive_duplicate_frames')} for k,v in summary.items()},indent=2))
