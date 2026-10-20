"""Bounded terminal-cell provenance scoring, not a new controller or clearance gate."""
from pathlib import Path
import json,sys,numpy as np
from scipy.ndimage import distance_transform_edt
D=Path(__file__).resolve().parent;W=D.parents[1];H=W/'experiments/stereo_memory_navigation_20261004_v1';sys.path[:0]=[str(H),'/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm']
from repeated_exposure_memory_v1 import RepeatedExposureMemoryGrid
from execution_domain_four_point_consumer_v2 import planning_clearance
R=D/'candidate_e2_run';maps=sorted((R/'maps').glob('*.npz'));last=np.load(maps[-1]);t=float(last['time_s']);native=json.loads((R/'native_predictions.json').read_text());n=min((x for x in native if x.get('available')),key=lambda x:abs(float(x['time_s'])-t));assert abs(float(n['time_s'])-t)<1e-7;root=np.array(n['position_world'])
g=RepeatedExposureMemoryGrid(last['origin'],10,static_scene_assumption=True);g.labels=last['labels'];cell=g.cells(root[:2][None])[0];hazards=np.argwhere(g.labels==1);dist=np.linalg.norm((hazards-cell)*g.resolution,axis=1);selected=hazards[dist<planning_clearance(g)+1e-8];sel=set(map(tuple,selected));counts=dict(point_instances=0,truth_valid=0,truth_floor=0,truth_obstacle=0,frame_count=0);examples=[]
for p in sorted((R/'partial_observations').glob('*.npz')):
 a=np.load(p)
 if float(a['acquisition_time_s'])>t:continue
 cells=g.cells(a['world'][:,:2]);k=np.array([tuple(x) in sel for x in cells],bool)
 if not k.any():continue
 f=np.load(R/'frames'/p.name);q=a['query_pixel_id'][k];xx=q%640;yy=q//640;z=f['left_depth_m'][yy,xx];valid=np.isfinite(z)&(z>0);rays=np.column_stack([(xx-319.5)/584.8888888888889,(yy-239.5)/584.8888888888889,np.ones(len(q))]);T=f['left_camera_usd_world'];truth=T[:3,3]+(rays*np.where(valid,z,0)[:,None])@(T[:3,:3]@np.diag([1,-1,-1])).T
 floor=valid&(abs(truth[:,2])<=.03);obstacle=valid&(truth[:,2]>.05)&(truth[:,2]<=1.5)
 for key,value in [('point_instances',len(q)),('truth_valid',valid.sum()),('truth_floor',floor.sum()),('truth_obstacle',obstacle.sum()),('frame_count',1)]:counts[key]+=int(value)
 examples.append(dict(frame=int(a['frame']),points=len(q),truth_floor=int(floor.sum()),truth_obstacle=int(obstacle.sum()),median_predicted_height=float(np.median(a['world'][k,2])),median_truth_height=float(np.median(truth[valid,2])) if valid.any() else None))
x=dict(map_frame=maps[-1].stem,time_s=t,native_root=root.tolist(),nearest_hazard_cell_clearance_m=float(dist.min()) if len(dist) else None,planning_clearance_m=planning_clearance(g),cells_inside_planning_margin=selected.tolist(),saved_point_provenance=counts,source_frames=examples,scope='All saved admitted source point instances in current blocking cells, not assertion that every point still has active individual lifetime. Same-frame renderer only offline; invalid pixels excluded; no altered labels or clearance and no alternate physical outcome.')
(D/'terminal_blocking_points.json').write_text(json.dumps(x,indent=2)+'\n');print(json.dumps({k:v for k,v in x.items() if k!='source_frames'}))
