"""Offline origin of actual trigger cells and frozen camera/time consistency."""
import json
import os
from pathlib import Path
import numpy as np
from geometry import camera_pose,FOCAL,CX,CY
from control import frame_coordinates

HERE=Path(os.environ.get('BILATERAL_ASSESS_ROOT',Path(__file__).resolve().parent))


def main():
    results={}
    for label in ('e2','e1'):
        root=HERE/f'candidate_{label}';records=json.loads((root/'vision_records.json').read_text())
        event=next(r for r in records if r['reference_created']);index=event['block'];sources={}
        origin_error=[];rotation_error=[];time_error=[];raw_clock=[]
        for i,path in enumerate(sorted((root/'online_inputs').glob('frame_*.npz'))):
            with np.load(path) as inp,np.load(root/'offline_truth'/path.name) as truth:
                pos=inp['position_world_xy'];q=inp['quaternion_world_wxyz_history'][-1]
                time_error.append(abs(float(truth['simulation_time_s'])-i/10))
                for eye in ('left','right'):
                    center,R=camera_pose(pos,q,eye);matrix=truth[f'{eye}_camera_usd_world']
                    actualR=np.column_stack((matrix[:3,0],-matrix[:3,1],-matrix[:3,2]))
                    origin_error.append(float(np.linalg.norm(center-matrix[:3,3])))
                    rotation_error.append(float(abs(R-actualR).max()))
                    if i>index or i<index-30:continue
                    points=inp[f'{eye}_points_world'];uv=inp[f'{eye}_uv'];coords=np.rint(uv).astype(int)
                    depth=truth[f'{eye}_depth_m'][coords[:,1],coords[:,0]]
                    valid=np.isfinite(depth)&(depth>0)&(depth<=10)
                    rays=np.column_stack(((uv[:,0]-CX)/FOCAL,(uv[:,1]-CY)/FOCAL,np.ones(len(uv))))
                    world=matrix[:3,3]+(rays*np.where(valid,depth,0)[:,None])@actualR.T
                    own=valid&(np.linalg.norm(world-truth['root_position_world_m'],axis=1)<=.55)
                    floor=valid&~own&(abs(world[:,2])<=.12)
                    obstacle=valid&~own&(world[:,2]>=.25)&(world[:,2]<=1.5)
                    picked=(points[:,2]>=.25)&(points[:,2]<=1.5)&(np.linalg.norm(points-np.r_[pos,.4],axis=1)>.55)
                    for j in np.flatnonzero(picked):
                        key=tuple(np.rint(points[j,:2]/.1).astype(int))
                        category='self' if own[j] else 'floor' if floor[j] else 'body_height' if obstacle[j] else 'other_unknown'
                        sources.setdefault(key,[]).append(dict(eye=eye,block=i,truth_surface=category,
                                          source_uv=uv[j].tolist(),true_world_xy=world[j,:2].tolist() if valid[j] else None,
                                          predicted_depth_m=float(inp[f'{eye}_depth_m'][j]),
                                          renderer_depth_m=float(depth[j]) if np.isfinite(depth[j]) else None))
        position=np.asarray(event['position_world_xy']);goal=np.asarray(event['goal_base_xy']);
        with np.load(root/'online_inputs'/f'frame_{index:03d}.npz') as f:
            yaw=float(f['yaw_world_rad']);c,s=np.cos(yaw),np.sin(yaw)
            goal_world=position+np.array([[c,-s],[s,c]])@goal
            cells=f['observed_obstacle_xy']
        local=frame_coordinates(cells,position,goal_world)
        central=(local[:,0]>=.55)&(local[:,0]<=3.5)&(abs(local[:,1])<=.6)
        labels={name:0 for name in ('self','floor','body_height','other_unknown')};cell_records=[]
        wall_lateral=[]
        for cell in cells[central]:
            src=sources.get(tuple(np.rint(cell/.1).astype(int)),[])
            categories=set(x['truth_surface'] for x in src)
            for name in categories:labels[name]+=1
            for source in src:
                if source['truth_surface']=='body_height':
                    local_true=frame_coordinates(np.asarray([source['true_world_xy']]),position,goal_world)
                    wall_lateral.append(float(local_true[0,1]))
            cell_records.append(dict(predicted_world_xy=cell.tolist(),source_truth_categories=sorted(categories),sources=src))
        results[label]=dict(trigger_block=index,trigger_time_s=event['time_s'],central_cells=int(central.sum()),
                    central_cells_by_true_source=labels,source_categories_may_overlap=True,
                    true_body_height_source_lateral_range_m=[min(wall_lateral),max(wall_lateral)] if wall_lateral else None,
                    trigger_side=event['side'],candidate_observed_clearances_m=event['side_observed_clearance'],
                    camera_origin_max_error_from_0_4m_height_m=max(origin_error),
                    camera_rotation_max_element_error=max(rotation_error),
                    observation_relative_clock_max_error_s=max(time_error),
                    central_cell_sources=cell_records)
        print(label,{k:v for k,v in results[label].items() if k!='central_cell_sources'},flush=True)
    (HERE/'online_geometry_audit.json').write_text(json.dumps(results,indent=2)+'\n')


if __name__=='__main__':main()
