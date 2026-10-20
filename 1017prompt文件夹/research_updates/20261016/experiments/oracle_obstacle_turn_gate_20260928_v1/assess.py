import json,hashlib,math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from clock_prefix import prefix_matches
from gate import clearance
h=Path(__file__).resolve().parent;old=h.parent/'oracle_turn_envelope_clockfix_20260928_v1';scenes=json.loads((h/'scene_manifest.json').read_text());results={};fig,axes=plt.subplots(2,2,figsize=(12,10))
for row,kind in enumerate(['LR','RL']):
 for col,mode in enumerate(['safe','blocked']):
  name=kind+'_'+mode;p=h/name;z=np.load(p/'physics.npz');original=np.load(old/f'{kind}_validation/physics.npz');pred=np.load(p/'prediction_before_chain.npz');contact=np.load(p/'contact.npz')['contact'];decision=json.loads((p/'prediction_status.json').read_text());c=np.array(scenes[name]['center_xy'])
  prefix=prefix_matches(z,{k:original[k][:801] for k in original.files},801)
  xy=z['position_world_xy'];speed=np.linalg.norm(z['linear_velocity_world_m_s'][:,:2],axis=1);physical_clearance=clearance(xy,c,.55)
  err=np.linalg.norm(xy[800:]-pred['position_world_xy'],axis=1);heading=np.abs(np.arctan2(np.sin(z['yaw_world_rad'][800:]-pred['yaw_world_rad']),np.cos(z['yaw_world_rad'][800:]-pred['yaw_world_rad'])))
  contact_valid=bool(np.all(contact[:,1:])) and len(contact)==len(xy)
  stopped=bool(np.all(speed[np.arange(980,1001,5)]<.05))
  gatecorrect=bool(decision['selected_turn']==(mode=='safe'))
  okay=all(prefix.values()) and contact_valid and not np.any(contact[:,0]) and physical_clearance>0 and gatecorrect
  okay=okay and (bool(err.max()<=.1 and heading.max()<=math.radians(5)) if mode=='safe' else stopped)
  results[name]={'prefix_match':all(prefix.values()),'prefix_fields':prefix,'decision':decision,'contact_monitor_valid':contact_valid,'contact_samples':int(np.sum(contact[:,0])),'min_center_sphere_clearance_m':physical_clearance,'speed_final_m_s':float(speed[-1]),'final_five10hz_speed_m_s':speed[np.arange(980,1001,5)].tolist(),'five_stop_confirmations':stopped,'max_template_error_m':float(err.max()) if mode=='safe' else None,'max_template_heading_error_deg':float(np.degrees(heading.max())) if mode=='safe' else None,'post_decision_actual_distance_m':float(np.linalg.norm(np.diff(xy[800:],axis=0),axis=1).sum()),'local_gate_passed':bool(okay)}
  ax=axes[row,col];ax.plot(*xy[800:].T,label='actual');ax.plot(*pred['position_world_xy'].T,'--',label='turn prediction');ax.add_patch(plt.Circle(c,.15,color='black',label='physical cylinder'));ax.add_patch(plt.Circle(c,.65,fill=False,color='red',ls=':',label='body+error+obstacle'));ax.plot(*xy[800],'o',label='decision start');ax.axis('equal');ax.set_title(name);ax.grid(alpha=.3);ax.legend(fontsize=8)
manifest=json.loads((h/'pre_run_identity.json').read_text());assert all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==v for p,v in manifest.items())
out={'results':results,'accepted_local_gate':all(r['local_gate_passed'] for r in results.values()),'formal_navigation':False};(h/'assessment.json').write_text(json.dumps(out,indent=2)+'\n');fig.tight_layout();fig.savefig(h/'comparison.png',dpi=150);print(json.dumps(out,indent=2))
