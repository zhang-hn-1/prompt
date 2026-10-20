from pathlib import Path
import json,hashlib
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from clock_prefix import prefix_matches
from gate import clearance
h=Path(__file__).resolve().parent;old=h.parent/'oracle_terminal_aware_detour_20260928_v1';m=json.loads((h/'manifest.json').read_text());result={};fig,axes=plt.subplots(1,2,figsize=(12,5))
for kind,ax in zip(['LR','RL'],axes):
 result[kind]={};goal=np.array(m[kind]['goal']);center=np.array(m[kind]['obstacle']['center_xy']);ev=json.loads((h/f'{kind}_candidate/terminal.json').read_text());boundary=ev[0]['block']*5 if ev else 1750
 base=np.load(old/f'{kind}_candidate/physics.npz')
 for label,folder in [('previous',old),('candidate',h)]:
  p=folder/f'{kind}_candidate';z=np.load(p/'physics.npz');xy=z['position_world_xy'];v=np.linalg.norm(z['linear_velocity_world_m_s'][:,:2],axis=1);d=np.linalg.norm(xy-goal,axis=1);ct=np.load(p/'contact.npz')['contact'];hit=np.flatnonzero(d<=.2);last=np.arange(1700,1751,5)
  prefix=all(prefix_matches(z,{k:base[k][:boundary+1] for k in base.files},boundary+1).values())
  stable=bool(np.all(d[last]<=.2) and np.all(v[last]<.05));passed=bool(np.max((xy-center)@np.array(m[kind]['forward']))>=.55);valid=bool(len(ct)==len(xy) and np.all(ct[:,1:]));margin=clearance(xy,center,.55)
  result[kind][label]={'first_goal_s':float(z['simulation_time_s'][hit[0]]) if len(hit) else None,'min_distance_m':float(d.min()),'final_distance_m':float(d[-1]),'final_speed_m_s':float(v[-1]),'last1s_max_distance_m':float(d[last].max()),'last1s_max_speed_m_s':float(v[last].max()),'stable_goal':stable,'contacts':int(ct[:,0].sum()),'contact_valid':valid,'body_clearance_m':margin,'passed_obstacle':passed,'prefix_to_terminal_matches':prefix,'strict_success':bool(stable and passed and valid and not ct[:,0].any() and margin>0 and prefix),'terminal_start':ev[0] if label=='candidate' and ev else None}
  ax.plot(*xy[1400:].T,label=label)
 ax.add_patch(plt.Circle(goal,.2,color='green',alpha=.15));ax.plot(*goal,'*',color='red');ax.set_title(kind+' terminal approach');ax.axis('equal');ax.grid(alpha=.3);ax.legend()
identity=json.loads((h/'pre_compare_identity.json').read_text());assert all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==v for p,v in identity.items())
output={'results':result,'accepted':all(r['candidate']['strict_success'] for r in result.values()),'formal_benchmark':False};(h/'assessment.json').write_text(json.dumps(output,indent=2));fig.tight_layout();fig.savefig(h/'comparison.png',dpi=150);print(json.dumps(output,indent=2))
