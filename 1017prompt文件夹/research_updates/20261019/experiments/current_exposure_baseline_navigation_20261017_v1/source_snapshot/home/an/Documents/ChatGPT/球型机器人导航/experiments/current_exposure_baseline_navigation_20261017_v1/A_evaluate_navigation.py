"""Sealed E1 post-run teacher evaluation. No GT feeds back to navigation."""
from paths import *
from source import NAMES,RAYS,AX
import json,numpy as np,hashlib
from scipy.spatial import cKDTree
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def quant(v):return np.quantile(v,[.5,.9,.95]).tolist() if len(v) else None
def main():
    out=D/'e1_run';status=json.loads((D/'navigation_status.json').read_text());assert status['status']=='finished' and status['child_returncode']==0
    seal=json.loads((out/'source_seal.json').read_text());cap=json.loads((out/'capture_seal.json').read_text());assert seal['complete'] and cap['complete']
    cfg=json.loads((D/'protocol.json').read_text());ep=json.loads((out/'episode.json').read_text());physics=np.load(out/'physics.npz')
    occupancy=np.load(cfg['occupancy']); origin=occupancy['global_origin_xy']; resolution=float(occupancy['resolution_m']);assert abs(resolution-.1)<1e-7
    actual=np.argwhere(occupancy['grid']>0)*resolution+origin;truth=cKDTree(actual)
    traces=physics['root_position_world_m'];start=traces[0,:2];goal=np.asarray(ep['goal_position_world_m'])[:2]
    policy=json.loads((out/'policy_records.json').read_text());req=np.load(out/'requests.npz')['requests']
    pp=[];source=[];targetall=0;validall=0;coveredall=0;predall=0;correctall=0;past_targets=set();blindden=0;blindmiss=0;falsemapintegral=0.;maxfalse=0;err=[]
    latest_target=set();frame_idx=0;source_by_time={round(float(r['time_s']),5):r for r in seal['rows']}
    for r,c in zip(seal['rows'],cap['rows']):
        assert r['frame']==len(source) and r['input_sha256']==c['sha256'];f=c['frame'];ip=out/'frames'/f;pf=out/'predictions'/f
        assert sha(ip)==c['sha256'] and sha(pf)==r['prediction_sha256']
        target=[];world=[];totalpix=0;v=0;count={}
        with np.load(ip) as a,np.load(pf) as p:
            for n in NAMES:
                z=a[n+'_depth_label'];T=a[n+'_camera_usd_world_label'];w=(RAYS*z[...,None])@(T[:3,:3]@AX).T+T[:3,3]
                k=np.isfinite(z)&(z>=.5)&(z<=3)&~p[n+'_CAD']&(np.linalg.norm(w-a['root_position_physics_label'],axis=2)>=.55)&(w[...,2]>.05)&(w[...,2]<=1.5)
                valid=p[n+'_valid']&np.isfinite(p[n+'_depth'])&(p[n+'_depth']>=.5)&(p[n+'_depth']<=3)
                e=abs(z[k&valid]-p[n+'_depth'][k&valid]);err.extend(e.tolist());totalpix+=int(k.sum());v+=int((k&valid).sum())
                target.append(np.floor((w[k,:2]-origin)/.1+.5).astype(np.int64));world.append(p[n+'_world']);count[n]=dict(target_pixels=int(k.sum()),valid_target_pixels=int((k&valid).sum()),risk_points=len(p[n+'_world']))
            target=np.unique(np.concatenate(target),axis=0);world=np.concatenate(world);pred=np.unique(np.floor((world[:,:2]-origin)/.1+.5).astype(np.int64),axis=0)
            d=cKDTree(target*.1).query(pred*.1)[0] if len(target) and len(pred) else np.full(len(pred),np.inf)
            td=cKDTree(pred*.1).query(target*.1)[0] if len(target) and len(pred) else np.full(len(target),np.inf)
            targetall+=len(target);coveredall+=int((td<=.15).sum());predall+=len(pred);correctall+=int((d<=.15).sum());validall+=v
            source.append(dict(frame=r['frame'],time_s=r['time_s'],target_cells=len(target),covered_cells=int((td<=.15).sum()),pred_cells=len(pred),correct_cells=int((d<=.15).sum()),target_pixels=totalpix,valid_target_pixels=v,per_eye=count,generation_s=r['generation_s'],target_cells_array=target.tolist()))
    latest_target=set();past_targets=set();idx=0;falsepath=[]
    for i,row in enumerate(policy):
        t=float(row['time_s'])
        while idx<len(source) and source[idx]['time_s']<=t+1e-8:
            latest_target=set(map(tuple,source[idx]['target_cells_array']));past_targets|=latest_target;idx+=1
        with np.load(out/'maps'/f'frame_{i:03d}.npz') as m:
            cells=np.argwhere(m['labels']==1);xy=cells*float(m['resolution'])+m['origin'];false=int((truth.query(xy)[0]>.15).sum()) if len(xy) else 0
            falsemapintegral+=false*.1;maxfalse=max(maxfalse,false);falsepath.append(false)
            blind=np.array(sorted(past_targets-latest_target),int).reshape(-1,2)
            if len(blind):
                bt=blind*.1+origin;distance=cKDTree(xy).query(bt)[0] if len(xy) else np.full(len(bt),np.inf)
                blindden+=len(blind);blindmiss+=int((distance>.15).sum())
    term=json.loads((out/'terminal.json').read_text());totalrisk=sum(sum(r['per_eye_points'].values()) for r in seal['rows']);nonempty=sum(any(r['per_eye_points'].values()) for r in seal['rows'])
    result=dict(complete=True,termination=term['termination'],sim_time_s=term['time_s'],actions=len(req),frames=len(source),nonempty_source_frames=nonempty,total_risk_points=totalrisk,minimum_observation_support=nonempty>=11 and totalrisk>=100,goal_distance_initial_m=float(np.linalg.norm(start-goal)),goal_distance_final_m=float(np.linalg.norm(traces[-1,:2]-goal)),net_goal_progress_m=float(np.linalg.norm(start-goal)-np.linalg.norm(traces[-1,:2]-goal)),path_length_m=float(np.linalg.norm(np.diff(traces[:,:2],axis=0),axis=1).sum()),request_reverse_time_s=float((req[:,::2].mean(axis=1)<0).sum()*.1),request_direction_switches=int((np.diff(np.sign(req[:,::2].mean(axis=1)))!=0).sum()),source=dict(target_cells=targetall,covered_cells=coveredall,predicted_cells=predall,correct_cells=correctall,precision=correctall/predall if predall else None,recall=coveredall/targetall if targetall else None,valid_target_pixels=validall,conditional_depth_P50_P90_P95_m=quant(err),generation_P50_P90_P95_s=quant([r['generation_s'] for r in source])),memory=dict(false_map_cell_seconds=falsemapintegral,max_false_map_cells=maxfalse,blind_historical_target_cell_queries=blindden,blind_missing_queries=blindmiss,blind_historical_missing_fraction=blindmiss/blindden if blindden else None,scope='Historical visible GT target cells currently out of target union; not all physical blind volume. Memory absence is unknown. False map vs static entire bodyband occupancy within.15m; not a 3D certificate.'),end_to_end_policy_execute_render_perception_P50_P90_P95_s=quant([r['action_render_perception_wall_s'] for r in policy]),source_rows=source,false_map_cells_by_policy=falsepath,comparison='prospective A current-exposure development baseline; known B reference, no full3arm or independent generalization claim',old_strict_gate_pass=False,GT_scope='postseal scoring only except disclosed initial gauge/height/terminal',pose_and_semantics_limits='Native gyro encoder no slip assumption, initial gauge privileged, flatheight, ideal bank feedback. Source uses world-height risk proxy not real RGB semantics.')
    (D/'e1_result.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n');print('NAV_RESULT',json.dumps({k:result[k] for k in ('termination','sim_time_s','actions','frames','net_goal_progress_m','path_length_m','minimum_observation_support','end_to_end_policy_execute_render_perception_P50_P90_P95_s')}),flush=True)
if __name__=='__main__':main()
