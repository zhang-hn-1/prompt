from pathlib import Path
import json,hashlib,datetime,numpy as np
from scipy.ndimage import distance_transform_edt
from scipy.spatial import cKDTree
from persistent_core_grid import PersistentCoreGrid
from negative_memory_validation import true_cylinder_clearance
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
H=Path(__file__).resolve().parent;O=H/'negative_ray_memory_stop_cause_v1';O.mkdir(exist_ok=False)
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
D=H/'negative_ray_memory_R1_run_v2';rows=json.loads((D/'policy_records.json').read_text());states=json.loads((D/'states.json').read_text());ref=np.load(D/'local_plans/frame_140.npz')['pose'][:,:2];delta=np.diff(ref,axis=0);length=np.linalg.norm(delta,axis=1);arc=np.r_[0,np.cumsum(length)]
def project(xy):
 u=np.clip(((xy-ref[:-1])*delta).sum(1)/length**2,0,1);points=ref[:-1]+u[:,None]*delta;dist=np.linalg.norm(points-xy,axis=1);j=int(dist.argmin());return float(arc[j]+u[j]*length[j]),float(dist[j]),points[j]
xy=np.asarray([s['position'][:2] for s in states]);course=[]
for i in range(140,266):
 est=np.load(D/'estimated_poses'/f'frame_{i:03d}.npz')['root_position_world_m'][:2];p,e,_=project(xy[i]);pe,ee,_=project(est);course.append(dict(frame=i,time_s=rows[i]['time_s'],true_progress_m=p,true_lateral_error_m=e,estimated_progress_m=pe,estimated_lateral_error_m=ee,request=rows[i]['request'][:2],actual_speed_m_s=float(np.linalg.norm(states[i]['velocity'][:2]))))
m=np.load(D/'maps/frame_265.npz');seed=dict(np.load(D/'frozen_memory_seed.npz'));init=np.load(D/'estimated_poses/frame_000.npz');pose=np.load(D/'estimated_poses/frame_265.npz');est=pose['root_position_world_m'][:2];point=np.asarray(rows[265]['local']['reference_decision']['guard']['bridge_xy'][1]);field=distance_transform_edt(np.pad(m['labels']!=1,1))[1:-1,1:-1]*float(m['resolution']);g=PersistentCoreGrid(m['origin'],60);g.set_source(dict(time_s=float(init['time_s'])));g.update_points(np.empty((0,3)),init['root_position_world_m'],float(init['time_s']));g.seed(seed,float(seed['time_s']));g.set_source(dict(time_s=float(pose['time_s'])));g.update_points(np.empty((0,3)),pose['root_position_world_m'],float(pose['time_s']));np.testing.assert_equal(g.labels,m['labels'])
# All actor-derived inputs presealed; only two fixed original grid queries.
inputs=[D/'local_plans/frame_140.npz',D/'maps/frame_265.npz',D/'estimated_poses/frame_265.npz',D/'frozen_memory_seed.npz',D/'policy_records.json',D/'states.json']
(O/'inputs_and_code_seal.json').write_text(json.dumps(dict(code_sha256=sha(__file__),input_sha256={str(p):sha(p) for p in inputs},queries=['actual_estimated_root','original_guard_projection'],no_GT_actor=True),indent=2)+'\n')
queries=[]
for label,q in [('actual_estimated_root',est),('original_guard_projection',point)]:
 cell=g.cells(q[None])[0];route,status=g.plan(q,np.array([2.2,.15]));queries.append(dict(label=label,xy=q.tolist(),cell=cell.tolist(),original_raster_clearance_m=float(field[tuple(cell)]),status=status,route_exists=route is not None))
haz=m['origin']+np.argwhere(m['labels']==1)*float(m['resolution']);hd,hi=cKDTree(haz).query(est);nearest=haz[hi];raw=seed['raw_world_voxel_centres'];cd,ci=cKDTree(raw[:,:2]).query(nearest)
zero=next(i for i in range(140,len(rows)) if all(np.all(np.asarray(r['request'])==0) for r in rows[i:]));a=np.asarray([c['true_lateral_error_m'] for c in course]);stopped=xy[265:];dist=np.linalg.norm(xy-[2.2,.15],axis=1)
r=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),decision='REJECT_CONDITIONAL_MEMORY_NAVIGATION_PRESERVED',queries=queries,first_persistent_zero_frame=zero,persistent_zero_duration_s=(len(rows)-zero)*.1,first_stop_guard=rows[265]['local']['reference_decision']['guard'],true_initial_reference_lateral_RMS_m=float(np.sqrt(np.mean(a*a))),true_initial_reference_lateral_peak_m=float(a.max()),true_initial_reference_progress_at265_m=course[-1]['true_progress_m'],course=course,initial_reference_length_m=float(arc[-1]),true_stopped_phase_displacement_m=float(np.linalg.norm(stopped[-1]-stopped[0])),minimum_goal_XY_m=float(dist.min()),goal_never_reached=bool(dist.min()>.2),nearest_MODEL_hazard=nearest.tolist(),nearest_MODEL_distance_m=float(hd),nearest_retained_core_xyz=raw[ci].tolist(),nearest_core_to_hazard_distance_m=float(cd),true_distance_current_root_to_cylinder_surface_m=float(np.linalg.norm(xy[265])-.35),true_initial_nominal_clearance_m=true_cylinder_clearance(ref),not_online=True,not_fresh=True,no_parameter_or_control_change=True,whole_research_complete=False)
(O/'audit.json').write_text(json.dumps(r,indent=2)+'\n')
fig,axs=plt.subplots(1,2,figsize=(12,5));axs[0].scatter(haz[:,0],haz[:,1],s=6,label='MODEL raster');axs[0].plot(ref[:,0],ref[:,1],label='actual installed initial reference');axs[0].plot(xy[:,0],xy[:,1],label='actual trajectory');axs[0].plot([est[0],point[0]],[est[1],point[1]],'r--',label='265 guard bridge');axs[0].set_aspect('equal');axs[0].legend(fontsize=8);axs[0].grid();axs[1].plot([c['time_s'] for c in course],a,label='true lateral error');axs[1].axhline(.2,color='red',linestyle='--',label='empirical tube design .2m');axs[1].set_xlabel('simulation seconds');axs[1].set_ylabel('metres');axs[1].legend();axs[1].grid();fig.tight_layout();fig.savefig(O/'stop_cause.png',dpi=160)
print(json.dumps({k:v for k,v in r.items() if k not in ['course','first_stop_guard']},indent=2))
