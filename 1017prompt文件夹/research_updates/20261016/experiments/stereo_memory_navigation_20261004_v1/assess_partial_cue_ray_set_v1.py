"""Post-seal GT-only necessary XY containment / route feasibility, not admission."""
from pathlib import Path
import json,hashlib,datetime
import numpy as np
from scipy.spatial import cKDTree
from risk_grid import ExplorationGrid
from route_geometry_audit import hazard_clearance
H=Path(__file__).resolve().parent;D=H/'controlled_heading_probe_run_v1';O=H/'partial_cue_ray_set_v1';sha=lambda f:hashlib.sha256(Path(f).read_bytes()).hexdigest();s=json.loads((O/'prediction_seal_v1.json').read_text());assert all(sha(f)==v for f,v in s['input_sha256'].items());assert all(sha(f)==v for f,v in s['output_sha256'].items());p=np.load(O/'possible_set_v1.npz');w=p['world_voxel_centres'];pitch=float(p['pitch_m']);xy=np.unique(w[:,:2],axis=0);area=len(xy)*pitch**2;gt=np.load(D/'offline_occupancy_label.npz');occupied=np.argwhere(gt['labels']>0);truthxy=gt['origin']+occupied*float(gt['resolution']);dist=cKDTree(xy).query(truthxy)[0] if len(xy) else np.full(len(truthxy),np.inf);covered=dist<=np.sqrt(2)*pitch/2+1e-9;contain=float(covered.mean())
grid=ExplorationGrid(p['current_root_xy']-20,shape=(400,400),resolution=.1);cells=grid.cells(xy);cells=cells[grid.inside(cells)];grid.labels[tuple(cells.T)]=1;route,why=grid.plan(p['current_root_xy'],p['goal_xy']);length=None;clear=None
if route is not None:
 length=float(np.linalg.norm(np.diff(route,axis=0),axis=1).sum());clear=hazard_clearance(route,np.array([[0.,0.]]))-.75
checks=dict(area_le6=bool(area<=6),physical_XY_containment_ge95=bool(contain>=.95),geometric_route_exists=route is not None,route_length_le7=bool(length is not None and length<=7),continuous_true_body_clearance_ge0=bool(clear is not None and clear>=0))
np.savez_compressed(O/'diagnostic_route_v1.npz',xy=route if route is not None else np.empty((0,2)),not_executed=True,GT_not_input_to_planner=True)
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig,ax=plt.subplots(figsize=(9,6));ax.scatter(xy[:,0],xy[:,1],s=20,label='possible XY hypothesis (not observed)');ax.scatter(truthxy[:,0],truthxy[:,1],s=10,color='black',label='posthoc physical occupancy');theta=np.linspace(0,2*np.pi,200);ax.plot(.75*np.cos(theta),.75*np.sin(theta),'k--',label='root collision boundary')
if route is not None:ax.plot(route[:,0],route[:,1],label='unexecuted unchanged-grid route')
ax.scatter(*p['current_root_xy'],marker='*',s=80,color='green');ax.scatter(*p['goal_xy'],marker='*',s=80,color='red');ax.axis('equal');ax.grid();ax.legend();ax.set_xlabel('world X m');ax.set_ylabel('world Y m');ax.set_title('Historical partial-cue set feasibility / no control or model admission');fig.savefig(H/'partial_cue_ray_set_applicability_v1.png',dpi=160);plt.close(fig)
r=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),decision='PASS_NECESSARY_XY_APPLICABILITY_ONLY' if all(checks.values()) else 'REJECT_PARTIAL_SET_APPLICABILITY',checks=checks,retained_voxels=len(w),XY_cells=len(xy),XY_area_m2=area,physical_occupied_cells=len(truthxy),physical_XY_cells_covered=int(covered.sum()),physical_XY_containment=contain,physical_missing_XY=truthxy[~covered].tolist(),route_reason=why,unexecuted_route_length_m=length,unexecuted_continuous_body_clearance_m=clear,bbox_xy=None if not len(xy) else [xy.min(0).tolist(),xy.max(0).tolist()],source_kind='possible hypothesis from partial positive appearance support',old_depth_and_class_completeness_and_metric_volume_REJECT_preserved=True,source_not_globally_admitted=True,not_navigation_success=True,no_control_or_newphysics=True,historical_development_not_fresh=True,GT_only_after_seal=True,GT_not_used_by_planner=True,whole_research_complete=False,prediction_seal_sha256=sha(O/'prediction_seal_v1.json'),script_sha256=sha(__file__))
(O/'assessment_v1.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps({k:v for k,v in r.items() if k!='physical_missing_XY'},indent=2))
