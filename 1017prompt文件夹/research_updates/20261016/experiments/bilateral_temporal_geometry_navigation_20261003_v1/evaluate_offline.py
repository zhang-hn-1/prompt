"""Whole historical sequences; renderer depth is strictly scoring-only."""
import json
from pathlib import Path
import numpy as np
from geometry import MultiViewGeometry
from control import observed_obstacles,front_blocked

HERE=Path(__file__).resolve().parent
PILOT=HERE.parent/'fixed_rig_visual_pilot_20261001'
PROJECT=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')


def evaluate(label,folder,goal):
    model=MultiViewGeometry();rows=[];errors=[];scene_predictions=0;cache=[];by_eye={eye:0 for eye in ('left','right')}
    paths=sorted(folder.glob('frame_*.npz'))
    if not paths:raise FileNotFoundError(f'no complete sequence: {folder}')
    for path in paths:
        with np.load(path) as f:
            xy=f['root_position_world_m'][:2];q=f['root_quaternion_world_wxyz'];t=float(f['simulation_time_s'])
            # Predictor receives no file handle, depth, camera truth pose or label.
            pred=model.update(f['left_rgb'].copy(),f['right_rgb'].copy(),xy.copy(),q.copy(),t)
            fresh=[]
            for eye in ('left','right'):
                p=pred[eye];by_eye[eye]+=len(p['depth_m']);uv=np.rint(p['uv']).astype(int)
                truth=f[f'{eye}_depth_m'][uv[:,1],uv[:,0]]
                valid=np.isfinite(truth)&(truth>=.45)&(truth<=10)
                transform=f[f'{eye}_camera_usd_world'];rotation=np.column_stack((transform[:3,0],-transform[:3,1],-transform[:3,2]))
                rays=np.column_stack(((p['uv'][:,0]-319.5)/(16.45/18*640),
                                      (p['uv'][:,1]-239.5)/(16.45/18*640),np.ones(len(uv))))
                world=transform[:3,3]+(rays*np.where(valid,truth,0)[:,None])@rotation.T
                scene=valid&(np.linalg.norm(world-f['root_position_world_m'],axis=1)>.55)
                scene_predictions+=int(scene.sum())
                errors.extend(np.where(np.isfinite(truth),abs(p['depth_m']-truth),np.inf).tolist())
                fresh.append(observed_obstacles(p['points_world'],xy))
            cache=[(old,points) for old,points in cache if old>=t-3]
            cache.append((t,np.vstack(fresh)))
            observed=np.unique(np.vstack([p for _,p in cache]),axis=0)
            blocked=front_blocked(observed,xy,goal)
            rows.append(dict(frame=path.name,time_s=t,left_predictions=len(pred['left']['depth_m']),
                             right_predictions=len(pred['right']['depth_m']),observed_obstacle_cells=len(observed),central_blocked=blocked))
    array=np.asarray(errors);n=len(array)
    fraction=float(np.mean(array<=.5)) if n else None
    p90=float(np.percentile(array,90)) if n and np.isfinite(np.percentile(array,90)) else None
    event=next((rows[i]['time_s'] for i in range(1,len(rows)) if rows[i-1]['central_blocked'] and rows[i]['central_blocked']),None)
    output=dict(sequence=label,frames=len(rows),depth_predictions=n,scene_predictions=scene_predictions,
                predictions_by_eye=by_eye,fraction_error_le_0_5_m=fraction,p90_error_m=p90,
                first_two_frame_central_event_s=event,central_blocked_frame_fraction=float(np.mean([r['central_blocked'] for r in rows])),rows=rows)
    print(label,{k:v for k,v in output.items() if k!='rows'},flush=True)
    return output


def main():
    episodes=json.loads((PROJECT/'configs/benchmark_v1/golden_v1_manifest.json').read_text())['episodes']
    results={}
    for difficulty in ('e0','e1','e2'):
        spec=next(e for e in episodes if e['episode_id']==f'golden_{difficulty}_0001')
        folder='integrated_fill_e0b' if difficulty=='e0' else f'integrated_fill_{difficulty}'
        results[difficulty]=evaluate(difficulty,PILOT/f'{folder}/depth_truth',
                                     np.asarray(spec['goal_position_world_m'][:2]))
    gate=dict(depth_support=all(r['scene_predictions']>=30 for r in results.values()),
              depth_accuracy=all(r['fraction_error_le_0_5_m'] is not None and r['fraction_error_le_0_5_m']>=.8
                                 and r['p90_error_m'] is not None and r['p90_error_m']<=1 for r in results.values()),
              e1_early_detection=results['e1']['first_two_frame_central_event_s'] is not None
                                 and results['e1']['first_two_frame_central_event_s']<4,
              e2_false_blocking=results['e2']['central_blocked_frame_fraction']<=.1)
    (HERE/'offline_assessment.json').write_text(json.dumps(dict(sequences=results,gate=gate,accepted=all(gate.values()),
                         historical_regression_not_fresh_heldout=True),indent=2)+'\n')
    print('Frozen perception gate',gate,'accepted',all(gate.values()),flush=True)


if __name__=='__main__':main()
