"""Frozen first-fork map swap: necessary raster tube only, no policy/teacher/physics."""
from pathlib import Path
import sys,json,numpy as np
from scipy.ndimage import distance_transform_edt
H=Path(__file__).resolve().parent;sys.path.insert(0,'/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')
from repeated_exposure_memory_v1 import RepeatedExposureMemoryGrid
from retained_reference_tube_viability_v1 import tube_retention_screen
D=H/'E1_current_pixel_navigation_run_v1';B=H/'minimum_prefix_transition_navigation_run_v1';i=331;ns=json.loads((D/'states.json').read_text());bs=json.loads((B/'states.json').read_text());nxy=np.asarray([s['position'][:2] for s in ns]);bxy=np.asarray([s['position'][:2] for s in bs]);np.testing.assert_array_equal((nxy-nxy[0])[:i+1],(bxy-bxy[0])[:i+1]);n=json.loads((D/'policy_records.json').read_text())[i];old=json.loads((B/'policy_records.json').read_text())[i];ng=n['reference_decision']['guard'];og=old['reference_decision']['guard']
for key in ['bridge_xy','suffix_xy']:np.testing.assert_array_equal(ng[key],og[key])
new=np.load(D/'maps/frame_331.npz');b=np.load(B/'maps/frame_331.npz');raw=np.load(H/'E1_current_pixel_same_stream_map_audit_v1/map_331.npz')['original'];pieces=[]
for key in ['bridge_xy','suffix_xy']:
 q=np.asarray(ng[key]);
 if len(q)==1:pieces.append(q)
 for a,z in zip(q[:-1],q[1:]):pieces.append(np.linspace(a,z,max(2,int(np.ceil(np.linalg.norm(z-a)/.05))+1)))
points=np.vstack(pieces);results={}
for name,labels in [('historical_B0',b['labels']),('new_filtered',new['labels']),('new_same_stream_unfiltered',raw)]:
 g=RepeatedExposureMemoryGrid(new['origin'],10,static_scene_assumption=True);g.labels=labels.copy();screen=tube_retention_screen(g,ng);cells=g.cells(points);dist,indices=distance_transform_edt(np.pad(labels!=1,1),return_indices=True);values=dist[1:-1,1:-1][tuple(cells.T)]*.1;k=int(np.argmin(values));c=cells[k]+1;nearest=indices[:,c[0],c[1]]-1;results[name]={'screen':screen,'minimum_at_sample':points[k].tolist(),'sample_cell':cells[k].tolist(),'nearest_risk_or_padded_boundary_cell':nearest.tolist(),'risk_cell_world':(g.origin+nearest*.1).tolist(),'hazard_cells':int((labels==1).sum())}
assert results['historical_B0']['screen']==old['reference_decision']['tube_viability_screen'];assert results['new_filtered']['screen']==n['reference_decision']['tube_viability_screen'];assert results['new_same_stream_unfiltered']['screen']['pass']==results['new_filtered']['screen']['pass']
result={'scope':'FROZEN_CURRENT_HISTORICAL_GUARD_MAP_SWAP_NECESSARY_SCREEN_NOT_CONTROL_AB','frame':331,'knownpose_physical_prefix_relative_XY_exact':True,'common_predecision_reference_bridge_suffix_exact':True,'only_swapped_labels':True,'results':results,'historical_vs_new_raw_label_difference':int((b['labels']!=raw).sum()),'map_change_crosses_existing_threshold':True,'threshold_not_changed':True,'no_teacher_or_future_or_policy_recall':True,'filter_not_cause_of_historical_rebuild_difference':True,'no_safety_or_complete_volume_certificate':True};(D/'first_fork_guard_map_cause_v1.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
