import json,math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree
from source_map import SourceMapReplanner,ROOT
from benchmark.drive_model import DriveModel
from probe import write
HERE=Path(__file__).resolve().parent
MODEL=DriveModel.from_json(ROOT/'research/ball_wm/configs/benchmark_v1/drive_dynamics_v1.json')
results={};fig,axes=plt.subplots(2,2,figsize=(12,9))
for row,case in enumerate(['0002','0004']):
    adapter=SourceMapReplanner(case,MODEL);results[case]={}
    entry=[]
    for group in ['real','reserve']:
        p=HERE/f'{case}_{group}';z=np.load(p/'trace.npz');ph=np.load(p/'physics.npz');config=json.loads((p/'config.json').read_text());goal=np.asarray(config['common_goal']);entry.append(json.loads((p/'entry.json').read_text())['before'])
        xy=ph['position_world_xy'];times=ph['simulation_time_s'];times=times-times[0];distance=np.linalg.norm(xy-goal,axis=1);hits=np.flatnonzero(distance<=.2);end=int(hits[0]) if len(hits) else len(xy)-1
        source_xy=xy@adapter.rotation.T+adapter.origin
        cells=np.floor((source_xy-np.asarray(adapter.map.global_origin_xy))/adapter.map.resolution_m+.5).astype(int)
        inside=(cells[:,0]>=0)&(cells[:,1]>=0)&(cells[:,0]<adapter.grid.shape[0])&(cells[:,1]<adapter.grid.shape[1]);unsafe=~inside
        unsafe[inside]=adapter.grid[cells[inside,0],cells[inside,1]]==1
        collision_idx=np.flatnonzero(unsafe[:end+1])
        # Dense polyline diagnostic distance; never used for goal success.
        ref=z['reference_xy'];dense=np.concatenate([np.linspace(a,b,max(2,int(np.ceil(np.linalg.norm(b-a)/.01))+1)) for a,b in zip(ref[:-1],ref[1:])]);err=cKDTree(dense).query(xy)[0]
        records=json.loads((p/'tracking.json').read_text())
        results[case][group]=dict(goal=goal.tolist(),goal_reached_by35=bool(len(hits)),first_goal_time_s=float(times[end]) if len(hits) else None,min_goal_distance_m=float(distance.min()),final_goal_distance_m=float(distance[-1]),actual_distance_m=float(np.linalg.norm(np.diff(xy,axis=0),axis=1).sum()),reference_length_m=float(np.linalg.norm(np.diff(ref,axis=0),axis=1).sum()),own_reference_rms_m=float(np.sqrt(np.mean(err**2))),own_reference_peak_m=float(err.max()),first_inflated_map_violation_s=float(times[collision_idx[0]]) if len(collision_idx) else None,original_map_clear_until_goal_or_end=not bool(len(collision_idx)),braking_request_s=.1*sum(r['source']=='direction_change_braking' for r in records),tracking_failure_blocks=sum(r['source']=='tracking_infeasible' for r in records))
        axes[row,0].plot(ref[:,0],ref[:,1],'--',alpha=.6,label=group+' reference');axes[row,0].plot(xy[:,0],xy[:,1],label=group+' actual');axes[row,1].plot(times,distance,label=group)
    # Reference-dependent policy fields are intentionally different; physics must match.
    results[case]['initial_physics_identical']=all(entry[0][k]==entry[1][k] for k in entry[0] if k!='policy')
    results[case]['common_goal_identical']=results[case]['real']['goal']==results[case]['reserve']['goal']
    c=results[case]['reserve'];results[case]['accepted']=results[case]['initial_physics_identical'] and results[case]['common_goal_identical'] and c['goal_reached_by35'] and c['original_map_clear_until_goal_or_end']
    axes[row,0].axis('equal');axes[row,0].set_title(case+' references and actual paths');axes[row,1].set_title(case+' distance to SAME goal');axes[row,1].set_xlabel('simulation time s');axes[row,1].set_ylabel('m')
    for ax in axes[row]:ax.grid(alpha=.3);ax.legend()
identity=json.loads((HERE/'pre_run_identity.json').read_text())
import hashlib
changed_identity_paths=[p for p,v in identity.items() if hashlib.sha256(Path(p).read_bytes()).hexdigest()!=v]
identity_unchanged=not changed_identity_paths
runtime_identity_unchanged=all(Path(p).name.startswith('test_') for p in changed_identity_paths)
write(HERE/'assessment.json',dict(results=results,pre_run_source_identity_unchanged=identity_unchanged,changed_identity_paths=changed_identity_paths,runtime_identity_unchanged=runtime_identity_unchanged,accepted=runtime_identity_unchanged and all(v['accepted'] for v in results.values()),formal_benchmark=False,clearance_scope='offline original inflated map check; not physical collision validation'))
fig.tight_layout();fig.savefig(HERE/'comparison.png',dpi=170)
print(json.dumps(results,indent=2))
