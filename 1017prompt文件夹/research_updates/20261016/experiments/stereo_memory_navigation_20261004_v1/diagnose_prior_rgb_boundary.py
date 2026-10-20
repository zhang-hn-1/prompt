from pathlib import Path
import json,numpy as np
from reference_continuity_audit import reference_geometry_status
H=Path(__file__).resolve().parent;a=json.loads((H/'prior_rgb_navigation_assessment.json').read_text());i=a['comparison']['first_request_divergence_frame'];assert i==135
arms={};maps={}
for arm in ('current','history'):
 O=H/('prior_rgb_'+arm+'_run');r=json.loads((O/'policy_records.json').read_text());states=json.loads((O/'states.json').read_text());m=np.load(O/'maps'/f'frame_{i:03d}.npz');ref=np.load(O/'local_plans'/f'frame_{i-1:03d}.npz')['pose'][:,:2];xy=np.asarray(states[i]['position'][:2]);g=reference_geometry_status(xy,ref,m['labels'],m['origin'],float(m['resolution']));arms[arm]=dict(state=states[i],request=r[i]['local']['request'],recomputed_retention_screen=g,plan_status=r[i]['local']['plan']['status']);maps[arm]=m
np.testing.assert_equal(arms['current']['state'],arms['history']['state'])
# Independent resets can yield different RGB/render samples. Do not claim byte-identical sources.
source_comparison={}
for f in ['source_predictions/frame_135.npz','online_inputs/frame_135.npz']:
 x=np.load(H/'prior_rgb_current_run'/f);y=np.load(H/'prior_rgb_history_run'/f);parts={}
 for k in x.files:
  if x[k].dtype.kind in 'biuf':
   xx=x[k];yy=y[k];finite=np.isfinite(xx)&np.isfinite(yy);parts[k]=dict(exact_equal=bool(np.array_equal(xx,yy,equal_nan=True)),finite_mean_absolute_difference=float(np.mean(np.abs(xx[finite].astype(float)-yy[finite].astype(float)))) if finite.any() else None)
 source_comparison[f]=parts
# Same history-arm source, same existing pre-branch reference: reset only the offline map memory.
from prior_rgb_navigation import PriorRGBGrid
latest=dict(np.load(H/'prior_rgb_history_run/sensor_packets/source_135.npz'));latest['frame']=int(latest['frame']);latest['time_s']=float(latest['time_s']);mh=maps['history'];current_from_history=PriorRGBGrid(mh['origin'],shape=mh['labels'].shape,resolution=float(mh['resolution']),memory_mode='current');current_from_history.set_source(latest);current_from_history.update_points([],arms['history']['state']['position'],float(mh['time_s']))
ref=np.load(H/'prior_rgb_history_run/local_plans/frame_134.npz')['pose'][:,:2];same_source_guard=reference_geometry_status(np.asarray(arms['history']['state']['position'][:2]),ref,current_from_history.labels,mh['origin'],float(mh['resolution']))
# Also replay a shared current-arm complete source prefix in a history map.
mc=maps['current'];common_history=PriorRGBGrid(mc['origin'],shape=mc['labels'].shape,resolution=float(mc['resolution']),memory_mode='history')
rows=json.loads((H/'prior_rgb_current_run/policy_records.json').read_text());states=json.loads((H/'prior_rgb_current_run/states.json').read_text())
for j in range(i+1):
 pkt=dict(np.load(H/'prior_rgb_current_run/sensor_packets'/f"source_{rows[j]['local']['source_frame']:03d}.npz"));pkt['frame']=int(pkt['frame']);pkt['time_s']=float(pkt['time_s']);common_history.set_source(pkt);common_history.update_points([],states[j]['position'],rows[j]['local']['time_s'])
refc=np.load(H/'prior_rgb_current_run/local_plans/frame_134.npz')['pose'][:,:2];common_prefix_guard=reference_geometry_status(np.asarray(states[i]['position'][:2]),refc,common_history.labels,mc['origin'],float(mc['resolution']))
assert arms['current']['recomputed_retention_screen']['retention_necessary_screen_pass'];assert not arms['history']['recomputed_retention_screen']['retention_necessary_screen_pass']
extra=(maps['history']['labels']==1)&(maps['current']['labels']!=1);cells=np.argwhere(extra);origin=maps['history']['origin'];res=float(maps['history']['resolution']);world=origin+cells*res;path=np.asarray(arms['current']['recomputed_retention_screen']['suffix_xy']);t=13.5
result=dict(frame=i,time_s=t,exact_same_physical_state=True,source_comparison=source_comparison,same_history_source_current_only_screen=same_source_guard,shared_current_source_prefix_history_screen=common_prefix_guard,arms=arms,history_extra_hazard_cells=len(cells),extra_hazard_world_xy=world.tolist(),extra_hazard_original_times=maps['history']['hazard_time'][extra].tolist(),conclusion='actual first divergence is a hazard-screen/no-route event; independent RGB outputs differ. Offline paired-source screens below isolate memory effect only at this consumer, and cannot establish a different online trajectory or uniquely attribute the actual first stop to memory',posthoc_diagnosis_not_new_AB=True)
(H/'prior_rgb_navigation_boundary_diagnosis.json').write_text(json.dumps(result,indent=2)+'\n');print('branch',i,'extra hazard cells',len(cells),'causal guard difference verified')
