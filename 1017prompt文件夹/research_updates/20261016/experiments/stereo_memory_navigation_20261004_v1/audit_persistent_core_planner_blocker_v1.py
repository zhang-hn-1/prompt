from pathlib import Path
import sys,json,hashlib,datetime,numpy as np
from scipy.spatial import cKDTree
from scipy.ndimage import distance_transform_edt
H=Path(__file__).resolve().parent
for name in ['oracle_feasible_route_20260929_v9/hybrid','bilateral_temporal_geometry_navigation_20261003_v1','clean_visual_hard0_intervention_20261003_v1','visible_surface_connectivity_20261004_v1']:sys.path.insert(0,str(H.parent/name))
from persistent_core_grid import PersistentCoreGrid
from partial_cue_source_v2 import inflate_xy
from tracking_tube_planner import planning_clearance
D=H/'bank_rock_observation_run_v1';O=H/'persistent_core_blocker_audit_v1';O.mkdir(exist_ok=False);S=H/'bilateral_core_memory_preflight_v2';sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
seal=json.loads((S/'outputs_pretruth_seal.json').read_text());assert all(sha(f)==s for f,s in seal.items());pool=dict(np.load(S/'pool.npz'));prov=json.loads((S/'core_provenance.json').read_text());assert len(prov['cores'])==len(pool['raw_world_voxel_centres'])
pose=np.load(D/'estimated_poses/frame_140.npz');init=np.load(D/'estimated_poses/frame_000.npz');origin=init['root_position_world_m'][:2]-20;t=float(pose['time_s']);root=pose['root_position_world_m'][:2]
current=dict(np.load(D/'possible_sets/frame_140.npz'));arms={'pool':pool,'last_current':dict(current,static_persistence_prior=True)};rows={};bad=None
for name,q in arms.items():
 g=PersistentCoreGrid(origin,60);g.seed(q,t);g.set_source(dict(time_s=t));g.update_points(np.empty((0,3)),pose['root_position_world_m'],t);cell=g.cells(root[None])[0];haz=g.origin+np.argwhere(g.labels==1)*g.resolution;dist,idx=cKDTree(haz).query(g.origin+cell*g.resolution);clear=distance_transform_edt(np.pad(g.labels!=1,1))[1:-1,1:-1]*g.resolution;blocked=haz[np.linalg.norm(haz-(g.origin+cell*g.resolution),axis=1)<planning_clearance(g)-1e-8]
 rows[name]=dict(start_cell=cell.tolist(),estimated_root_xy=root.tolist(),raster_root_xy=(g.origin+cell*g.resolution).tolist(),nearest_hypothesis_raster_xy=haz[idx].tolist(),nearest_distance_m=float(dist),EDT_start_m=float(clear[tuple(cell)]),required_m=planning_clearance(g),blocking_cells=len(blocked),hypothesis_cells=len(haz))
 if name=='pool':bad=blocked
# Independent offline GT only now; no change of frozen pool or any actor.
truth=np.load(D/'offline_occupancy_label.npz');xy=truth['origin']+np.argwhere(truth['labels']>0)*float(truth['resolution']);gtree=cKDTree(xy);core_rows=[]
for p,r in zip(pool['raw_world_voxel_centres'],prov['cores']):
 np.testing.assert_allclose(p,r['xyz'],atol=1e-9,rtol=0)
 expanded=inflate_xy(p[None,:2],.2);cells=np.rint((expanded-origin)/.1).astype(int);raster=origin+cells*.1
 if len(bad) and (cKDTree(bad).query(raster)[0]<1e-8).any():core_rows.append(dict(**r,offline_distance_to_true_footprint_m=float(gtree.query(p[:2])[0]),distance_to_current_estimated_root_m=float(np.linalg.norm(p[:2]-root))))
out=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),arms=rows,blocking_raw_cores=len(core_rows),blocking_core_details=core_rows,minimum_blocking_core_GT_distance_m=min(r['offline_distance_to_true_footprint_m'] for r in core_rows),maximum_blocking_core_GT_distance_m=max(r['offline_distance_to_true_footprint_m'] for r in core_rows),all_diagnostic_after_rejected_candidate=True,no_actor_or_hypothesis_changed=True,whole_research_complete=False,inputs_sha256=seal,code_sha256=sha(__file__))
(O/'assessment.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps({k:v for k,v in out.items() if k not in ['blocking_core_details','inputs_sha256']},indent=2))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig,ax=plt.subplots(figsize=(8,7));ax.scatter(pool['possible_xy'][:,0],pool['possible_xy'][:,1],s=24,label='pooled MODEL possible, not observation');ax.scatter(xy[:,0],xy[:,1],s=12,c='black',label='offline footprint label');ax.scatter(bad[:,0],bad[:,1],s=45,c='red',label='raster cells blocking frozen start');ax.scatter(root[0],root[1],marker='*',s=150,c='green',label='current estimated root');ax.add_patch(plt.Circle(root,.8707106781186548,fill=False,color='green',linestyle='--'));ax.set_aspect('equal');ax.set_xlabel('world X (m)');ax.set_ylabel('world Y (m)');ax.set_title('Rejected persistent core / offline diagnostic only');ax.legend();fig.tight_layout();fig.savefig(O/'actual_blocker.png',dpi=160)
