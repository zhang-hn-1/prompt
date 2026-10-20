from pathlib import Path
import json,hashlib
import numpy as np
from clock_prefix import prefix_matches
from gate import clearance
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
h=Path(__file__).resolve().parent;old=h.parent/'oracle_terminal_feedback_20260928_v1';m=json.loads((h/'manifest.json').read_text());results={};fig,axes=plt.subplots(2,2,figsize=(11,9))
assert (h/'terminal.py').read_bytes()==(old/'terminal.py').read_bytes()
for row,k in enumerate(['LR','RL']):
 base=np.load(old/f'{k}_candidate/physics.npz');goal=np.array(m[k]['goal']);center=np.array(m[k]['obstacle']['center_xy'])
 for col,mode in enumerate(['coast','recovery']):
  p=h/f'{k}_{mode}';z=np.load(p/'physics.npz');xy=z['position_world_xy'];d=np.linalg.norm(xy-goal,axis=1);v=np.linalg.norm(z['linear_velocity_world_m_s'][:,:2],axis=1);ct=np.load(p/'contact.npz')['contact'];cr=json.loads((p/'condition.json').read_text());tr=json.loads((p/'terminal.json').read_text());start=next((r for r in cr if r['active']),None);release=next((r for r in cr if r['done']),None);idx=start['block']*5 if start else 1750
  prefix=all(prefix_matches(z,{key:base[key][:idx+1] for key in base.files},idx+1).values());last=np.arange(1700,1751,5);hit=np.flatnonzero(d<=.2);valid=bool(len(ct)==len(xy) and np.all(ct[:,1:]));margin=clearance(xy,center,.55);passed=bool(np.max((xy-center)@np.array(m[k]['forward']))>=.55);stable=bool(np.all(d[last]<=.2) and np.all(v[last]<.05));resume=next((r for r in tr if release and r['block']>=release['block'] and r['target'][0]>0),None)
  results[k+'_'+mode]={'condition_start':start,'condition_release':release,'first_positive_terminal_after_release':resume,'outside_goal_low_speed_release':bool(release and release['distance']>.2 and release['speed']<.05),'first_goal_s':float(z['simulation_time_s'][hit[0]]) if len(hit) else None,'final_distance_m':float(d[-1]),'min_distance_m':float(d.min()),'last1s_max_distance_m':float(d[last].max()),'last1s_max_speed_m_s':float(v[last].max()),'last1s_50hz_max_distance_m':float(d[1700:].max()),'last1s_50hz_max_speed_m_s':float(v[1700:].max()),'prefix_matches':prefix,'contacts':int(ct[:,0].sum()),'contact_valid':valid,'body_clearance_m':margin,'passed_obstacle':passed,'stable':stable,'accepted':bool(prefix and stable and valid and not ct[:,0].any() and margin>0 and passed)}
  ax=axes[row,col];ax.plot(*base['position_world_xy'][1400:].T,label='undisturbed');ax.plot(*xy[1400:].T,label=mode);ax.add_patch(plt.Circle(goal,.2,color='green',alpha=.15));ax.plot(*goal,'r*');ax.set_title(k+' '+mode);ax.axis('equal');ax.legend();ax.grid(alpha=.3)
identity=json.loads((h/'pre_compare_identity.json').read_text());assert all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==v for p,v in identity.items())
out={'results':results,'all_passed':all(r['accepted'] for r in results.values()),'passed_count':sum(r['accepted'] for r in results.values()),'formal_benchmark':False};(h/'assessment.json').write_text(json.dumps(out,indent=2));fig.tight_layout();fig.savefig(h/'comparison.png',dpi=150);print(json.dumps(out,indent=2))
