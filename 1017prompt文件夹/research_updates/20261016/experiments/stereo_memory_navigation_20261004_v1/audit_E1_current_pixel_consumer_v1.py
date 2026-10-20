"""Same old point set, original memory, only current pixel support comparison."""
from pathlib import Path
import sys,json,hashlib,copy
import numpy as np
H=Path(__file__).resolve().parent;P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm');sys.path.insert(0,str(P))
from tentative_point_memory import TentativePointMemory
from repeated_exposure_memory_v1 import RepeatedExposureMemoryGrid
D=H/'minimum_prefix_transition_navigation_run_v1';O=H/'E1_current_pixel_consumer_historical_v1';O.mkdir();states=json.loads((D/'states.json').read_text());m0=TentativePointMemory();m1=TentativePointMemory();origin=np.load(D/'maps/frame_000.npz')['origin'];g0=RepeatedExposureMemoryGrid(origin,10,static_scene_assumption=True);g1=RepeatedExposureMemoryGrid(origin,10,static_scene_assumption=True);epochs=[];rows=[];sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();inputs={};lastsource=None
for i in range(546):
 t=float(states[i]['time_s']);root=np.asarray(states[i]['position']);mp=D/'maps'/f'frame_{i:03d}.npz';z=np.load(mp);assert abs(float(z['time_s'])-t)<1e-8;inputs[str(mp)]=sha(mp)
 if i%5==0:
  files=[D/'partial_observations'/f'frame_{i:03d}.npz',D/'sensor_packets'/f'source_{i:03d}.npz',D/'source_inputs'/f'frame_{i:03d}.npz'];a=np.load(files[0]);raw=np.load(files[1]);rgb=np.load(files[2]);inputs.update({str(p):sha(p) for p in files});assert abs(float(a['acquisition_time_s'])-t)<1e-8;assert abs(float(raw['time_s'])-t)<1e-8;assert abs(float(rgb['time_s'])-t)<1e-8
  mapping={int(q):j for j,q in enumerate(raw['query_pixel_id'])};assert len(mapping)==len(raw['query_pixel_id']);ix=np.asarray([mapping[int(q)] for q in a['query_pixel_id']],int)
  for k in ['world','sigma_m','query_pixel_id','partner_pixel_id']:np.testing.assert_array_equal(a[k],raw[k][ix])
  q=a['query_pixel_id'];r=a['partner_pixel_id'];lb=np.all(rgb['left_rgb'][q//640,q%640]==0,axis=1);rb=np.all(rgb['right_rgb'][r//640,r%640]==0,axis=1);keep=~lb&~rb;record=dict(frame=i,time_s=t,world=a['world'],query_pixel_id=q,partner_pixel_id=r,input_sha256={str(p):sha(p) for p in files});filtered=copy.deepcopy(record)
  for k in ['world','query_pixel_id','partner_pixel_id']:filtered[k]=filtered[k][keep].copy()
  m0.update(record,t);m1.update(filtered,t);lastsource=i;epochs.append(dict(frame=i,time_s=t,original_points=len(q),removed_exactblack_points=int((~keep).sum()),left_black=int(lb.sum()),right_black=int(rb.sum()),original_CAD_source_membership_exact=True))
 for g,m in [(g0,m0),(g1,m1)]:
  g.set_source(dict(frame=lastsource,time_s=float(states[lastsource]['time_s']),records=m.at(t)['records']));g.update_points(np.empty((0,3)),root,t)
 for key,value in [('labels',g0.labels),('codes',g0.codes),('hazard_acquisition_time',g0.hazard_acquisition_time)]:np.testing.assert_array_equal(value,z[key])
 changed=g0.labels!=g1.labels;rows.append(dict(frame=i,changed_risk_cells=int(changed.sum()),original_risk_cells=int((g0.labels==1).sum()),candidate_risk_cells=int((g1.labels==1).sum())))
 if changed.any():np.savez_compressed(O/f'map_{i:03d}.npz',original=g0.labels,candidate=g1.labels,origin=origin,time_s=t,no_observed_free=True)
result=dict(scope='HISTORICAL_SOURCE_PIXEL_RULE_AND_REAL_MEMORY_CONSUMER_NOT_ACTOR',actual_prefix_maps_exact=546,source_epochs=epochs,map_rows=rows,removed_points=sum(x['removed_exactblack_points'] for x in epochs),maps_changed=sum(x['changed_risk_cells']>0 for x in rows),decision='REJECT_BEHAVIOR_EQUIVALENT_TESTED_DOMAIN' if not any(x['changed_risk_cells'] for x in rows) else 'MAP_CHANGE_NECESSARY_ONLY_NOT_ADMITTED',no_teacher_input=True,no_policy_recall_or_new_nav=True,code_sha256=sha(__file__),protocol_sha256=sha(H/'E1_current_pixel_consumer_audit_protocol_v1.json'))
(O/'assessment.json').write_text(json.dumps(result,indent=2)+'\n');(O/'seal.json').write_text(json.dumps(dict(inputs=inputs,outputs={str(p):sha(p) for p in O.glob('*') if p.is_file() and p.name!='seal.json'},dependencies={str(H/x):sha(H/x) for x in ['tentative_point_memory.py','repeated_exposure_memory_v1.py','partial_point_risk_grid.py','static_sparse_point_risk_grid_v1.py','nonbinding_side_hypothesis_v1.py']}),indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k not in ['source_epochs','map_rows']},indent=2))
