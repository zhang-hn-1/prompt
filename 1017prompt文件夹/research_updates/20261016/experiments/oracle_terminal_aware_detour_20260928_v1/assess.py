import json,hashlib,numpy as np
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from gate import clearance
from clock_prefix import prefix_matches
h=Path(__file__).resolve().parent;m=json.loads((h/'manifest.json').read_text());out={};fig,axes=plt.subplots(1,2,figsize=(13,6))
for ax,kind in zip(axes,['LR','RL']):
 goal=np.array(m[kind]['goal']);c=np.array(m[kind]['obstacle']['center_xy']);old=np.load(h.parent/'oracle_local_detour_20260928_v1'/f'{kind}_hold/physics.npz');out[kind]={};pair=[]
 for mode in ['baseline','candidate']:
  p=(h if mode=='candidate' else h.parent/'oracle_local_detour_20260928_v1')/f'{kind}_{mode}';z=np.load(p/'physics.npz');ct=np.load(p/'contact.npz')['contact'];xy=z['position_world_xy'];t=z['simulation_time_s'];d=np.linalg.norm(xy-goal,axis=1);hits=np.flatnonzero(d<=.2);selection=json.loads((p/'selection.json').read_text());prefix=all(prefix_matches(z,{k:old[k][:801] for k in old.files},801).values());pair.append(z)
  margin=clearance(xy,c,.55);contactvalid=bool(np.all(ct[:,1:])) and len(ct)==len(xy)
  pred_error=None
  if selection['selected']!='brake':
   pred=np.load(p/f"{selection['selected']}_prediction_before_action.npz")['position_world_xy'];pred_error=float(np.linalg.norm(xy[800:800+len(pred)]-pred,axis=1).max())
  progress=(xy-np.array(m[kind]['start']))@np.array(m[kind]['forward']);obstacle_progress=float((c-np.array(m[kind]['start']))@np.array(m[kind]['forward']));passed=bool(np.max(progress)>=obstacle_progress+.55)
  safe=contactvalid and not ct[:,0].any() and margin>0
  success=prefix and safe and len(hits)>0 and passed
  if mode=='candidate':success=success and selection['eligible_exact_collected_start']
  r=json.loads((p/'tracking.json').read_text())
  out[kind][mode]={'hold_s':selection.get('chosen',{}).get('hold_s') if selection.get('chosen') else None,'navigation_gate':bool(prefix and safe and len(hits)>0 and passed),'model_gate':bool(pred_error is not None and pred_error<=.1),'selected':selection['selected'],'eligible_collected_state':selection['eligible_exact_collected_start'],'prefix_matches_collection':prefix,'goal_reached':bool(len(hits)),'first_goal_s':float(t[hits[0]]) if len(hits) else None,'minimum_goal_distance_m':float(d.min()),'final_goal_distance_m':float(d[-1]),'contact_valid':contactvalid,'contact_samples':int(ct[:,0].sum()),'minimum_actual_body_clearance_m':margin,'passed_obstacle_station':passed,'max_forward_progress_m':float(progress.max()),'compound_max_prediction_error_m':pred_error,'tracking_sources':{source:sum(x['source']==source for x in r) for source in set(x['source'] for x in r)},'local_goal_success':bool(success)}
  ax.plot(*xy[800:].T,label=mode)
  if (p/'rejoin.json').exists():
   ref=json.loads((p/'rejoin.json').read_text())
   if ref['clear']:ax.plot(*np.array(ref['path']).T,'--',label='rejoin reference')
 out[kind]['pair_first16s_identical']=all(prefix_matches(pair[0],{k:pair[1][k][:801] for k in pair[1].files},801).values())
 ax.plot(*goal,'*',ms=14,label='same frozen goal');ax.add_patch(plt.Circle(c,.15,color='black',label='physical obstacle'));ax.add_patch(plt.Circle(c,.55,fill=False,color='red',ls=':'));ax.axis('equal');ax.grid(alpha=.3);ax.legend(fontsize=8);ax.set_title(kind+' 35s local goal test')
identity=json.loads((h/'pre_compare_identity.json').read_text());assert all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==v for p,v in identity.items())
result={'results':out,'accepted_local_detour':all(x['candidate']['local_goal_success'] and x['pair_first16s_identical'] for x in out.values()),'all_compound_model_gates_pass':all(x['candidate']['model_gate'] for x in out.values()),'formal_benchmark':False}
(h/'assessment.json').write_text(json.dumps(result,indent=2)+'\n');fig.tight_layout();fig.savefig(h/'comparison.png',dpi=150);print(json.dumps(result,indent=2))
