from pathlib import Path
import json,numpy as np
h=Path(__file__).resolve().parent;src=h.parent/'oracle_terminal_independent_20260928_v1';m=json.loads((src/'manifest.json').read_text());out={}
for kind in ['LR','RL']:
 for mode in ['coast','recovery']:
  p=src/f'{kind}_{mode}';z=np.load(p/'physics.npz');xy=z['position_world_xy'];v=np.linalg.norm(z['linear_velocity_world_m_s'][:,:2],axis=1);d=np.linalg.norm(xy-m[kind]['goal'],axis=1);cr=json.loads((p/'condition.json').read_text());tr=json.loads((p/'terminal.json').read_text());start=next(r for r in cr if r['active']);release=next(r for r in cr if r['done']);a=start['block'];b=release['block'];hit=np.flatnonzero(d<=.2);goal=hit[0]/50 if len(hit) else None
  phases=[('reference_tracking',215,a),('test_braking',a,b),('after_release',b,350)];stats=[]
  for name,i,j in phases:
   x=xy[i*5:j*5+1];stats.append({'phase':name,'start_s':i/10,'end_s':j/10,'duration_s':(j-i)/10,'distance_enter_m':float(d[i*5]),'distance_exit_m':float(d[j*5]),'travel_m':float(np.linalg.norm(np.diff(x,axis=0),axis=1).sum()),'mean_speed_m_s':float(v[i*5:j*5+1].mean())})
  inds=np.arange(b*5,1751);moving=inds[v[inds]>.05];requests=z['raw_command_target'][inds,0];applied=z['applied_action'][inds,0];positive=inds[applied>.1];confirm=cr[[r['block'] for r in cr].index(b)-4:[r['block'] for r in cr].index(b)+1]
  out[kind+'_'+mode]={'phases':stats,'first_goal_s':goal,'release_s':b/10,'remaining_after_release_s':35-b/10,'time_release_to_goal_s':goal-b/10 if goal else None,'applied_main_above_0_1_delay_s':(positive[0]-b*5)/50 if len(positive) else None,'xy_speed_above_0_05_delay_s':(moving[0]-b*5)/50 if len(moving) else None,'low_confirm_samples':confirm if mode=='recovery' else None,'terminal_start_s':tr[0]['block']/10,'terminal_main_range':[min(r['target'][0] for r in tr),max(r['target'][0] for r in tr)],'all_raw_main_after_release_nonnegative':bool(np.all(requests>=0))}
(h/'timing.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
