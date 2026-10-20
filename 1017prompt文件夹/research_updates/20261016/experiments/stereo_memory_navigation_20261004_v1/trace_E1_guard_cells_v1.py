"""Only two preselected first-fork guard cells; current/past <=331, seal before teacher."""
from pathlib import Path
import json,hashlib,numpy as np
H=Path(__file__).resolve().parent;O=H/'E1_guard_cell_trace_v1';O.mkdir();targets=np.asarray([[-.1999999940395334,1.000000005960466],[-.3999999940395327,.6000000059604673]]);datasets={'old':('minimum_prefix_transition_navigation_run_v1','partial_observations'),'new_raw':('E1_current_pixel_navigation_run_v1','baseline_partial_observations'),'new_filtered':('E1_current_pixel_navigation_run_v1','partial_observations')};sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest();inputs={};out={}
for name,(dirname,folder) in datasets.items():
 D=H/dirname;mp=D/'maps/frame_331.npz';m=np.load(mp);origin=m['origin'];res=float(m['resolution']);cells=np.rint((targets-origin)/res).astype(int);inputs[str(mp)]=sha(mp);records=[[] for _ in targets]
 for i in range(0,331,5):
  p=D/folder/f'frame_{i:03d}.npz';a=np.load(p);t=float(a['acquisition_time_s']);assert t<=33.1;inputs[str(p)]=sha(p);rgbfile=D/'source_inputs'/p.name;rgb=np.load(rgbfile);inputs[str(rgbfile)]=sha(rgbfile);pc=np.rint((a['world'][:,:2]-origin)/res).astype(int)
  for k,c in enumerate(cells):
   ix=np.flatnonzero((pc==c).all(axis=1))
   for j in ix:
    q=int(a['query_pixel_id'][j]);r=int(a['partner_pixel_id'][j]);records[k].append({'frame':i,'time_s':t,'world':a['world'][j].tolist(),'sigma_m':float(a['sigma_m'][j]),'query_pixel_id':q,'partner_pixel_id':r,'left_black':bool(np.all(rgb['left_rgb'][q//640,q%640]==0)),'right_black':bool(np.all(rgb['right_rgb'][r//640,r%640]==0)),'point_index':int(j),'cell_rounding_delta_m':(a['world'][j,:2]-targets[k]).tolist()})
 out[name]=[]
 for k,c in enumerate(cells):
  rr=records[k];times=sorted(set(r['time_s'] for r in rr));clock=float(m['hazard_acquisition_time'][tuple(c)])
  out[name].append({'target_xy':targets[k].tolist(),'cell':c.tolist(),'records':rr,'source_epochs':len(times),'points':len(rr),'last_source_time_s':max(times) if times else None,'current_past10s_points':sum(33.09999926015735-r['time_s']<=10 for r in rr),'saved_actual_candidate_label':int(m['labels'][tuple(c)]),'saved_actual_candidate_hazard_clock_s':clock if np.isfinite(clock) else None,'raw_name_uses_candidate_map_only':name=='new_raw'})
result={'scope':'TWO_PRESELECTED_GUARD_CELLS_HISTORICAL_TRACE_NOT_SOURCE_ADMISSION','end_frame':331,'future_exposures_not_read':True,'raw_map_not_independently_saved':True,'datasets':out};(O/'trace_pretruth.json').write_text(json.dumps(result,indent=2)+'\n');seal={'inputs':inputs,'output_sha256':sha(O/'trace_pretruth.json'),'code_sha256':sha(Path(__file__))};(O/'seal_pretruth.json').write_text(json.dumps(seal,indent=2)+'\n');print({k:[{j:r[j] for j in ['points','source_epochs','last_source_time_s','current_past10s_points','saved_actual_candidate_label','saved_actual_candidate_hazard_clock_s']} for r in v] for k,v in out.items()})
