"""Replay frozen feature functions only; GT annotates stages after predictions."""
from pathlib import Path
import hashlib,json
import numpy as np
from feature_tracks import extract,match_descriptors,triangulate
from timed_static_base_self_mask import TimedStaticBaseSelfMask
from dynamic_self_mask import joints_from_proprio
H=Path(__file__).resolve().parent;D=H/'minimum_prefix_transition_navigation_run_v1';O=H/'left_temporal_missing_divider_development_v1'
predpath=O/'predictions.json';seal=json.loads((O/'prediction_seal.json').read_text())['sha256'];assert hashlib.sha256(predpath.read_bytes()).hexdigest()==seal
pred=json.loads(predpath.read_text());K=np.array([[584.8888888888889,0,319.5],[0,584.8888888888889,239.5],[0,0,1.]])
cad=TimedStaticBaseSelfMask();cache={};rows=[]
for row in pred:
    f=row['frame']
    for i in range(f-20,f+1,5):
        if i in cache:continue
        rgb=np.load(D/'source_inputs'/f'frame_{i:03d}.npz')['left_rgb'];pose=np.load(D/'known_poses'/f'frame_{i:03d}.npz')['left_camera_usd_world'];history=np.load(D/'history'/f'frame_{i:03d}.npz')['proprio_history'].copy();history[:,:9]=0
        xy,desc,size=extract(rgb,cad.project(joints_from_proprio(history))['left']);cache[i]=dict(xy=xy,desc=desc,pose=pose)
    q=cache[f];assert np.array_equal(q['xy'],np.array(row['prediction']['query_xy']))
    stages=[]
    for past in range(f-20,f,5):
        s=cache[past];pairs=match_descriptors(q['xy'],q['desc'],s['xy'],s['desc'],q['pose'],s['pose'],K)
        tested=[]
        for a,b in pairs:
            r=triangulate(q['xy'][a],s['xy'][b],q['pose'],s['pose'],K)
            tested.append(dict(query_index=a,source_index=b,geometry_valid=r is not None,sigma_pass=r is not None and r['sigma_m']<=.1))
        stages.append(dict(past=past,source_features=len(s['xy']),matches=tested))
    rows.append(dict(frame=f,query_features=len(q['xy']),stages=stages,accepted_indices=[p['query'][2] for p in row['prediction']['points']],conflicting_queries=row['prediction']['conflicting_queries']))
# Seal reproduced intermediate decisions before introducing scene/GT annotations.
intermediate=json.dumps(rows,sort_keys=True);intermediate_sha=hashlib.sha256(intermediate.encode()).hexdigest()
obj=next(x for x in json.loads((H/'minimum_prefix_transition_sparse_membership_v2/scene_collision_shapes.json').read_text()) if x['path']=='/E1Detour/Divider/divider_06');c=np.array(obj['position']);labels={}
for i,q in cache.items():
    depth=np.load(D/'frames'/f'frame_{i:03d}.npz')['left_depth_m'];xy=q['xy'];u,v=np.rint(xy).astype(int).T;z=depth[v,u];T=q['pose'];R=T[:3,:3]@np.diag([1,-1,-1])
    world=T[:3,3]+np.c_[(xy[:,0]-319.5)/K[0,0]*z,(xy[:,1]-239.5)/K[1,1]*z,z]@R.T
    labels[i]=np.isfinite(world).all(1)&(np.linalg.norm(world[:,:2]-c[:2],axis=1)<=obj['radius']+.001)&(np.abs(world[:,2]-c[2])<=obj['height']/2+.001)
for row in rows:
    lab=labels[row['frame']];row['query_target_features']=int(lab.sum());row['accepted_target_queries']=int(lab[row['accepted_indices']].sum())
    for s in row['stages']:
        target=[m for m in s['matches'] if lab[m['query_index']]];s['source_target_features']=int(labels[s['past']].sum());s['target_descriptor_matches']=len(target);s['target_geometry_valid']=sum(m['geometry_valid'] for m in target);s['target_sigma_pass']=sum(m['sigma_pass'] for m in target)
        s['all_descriptor_matches']=len(s['matches']);del s['matches']
result=dict(rows=rows,prediction_seal_sha256=seal,intermediate_decision_sha256=intermediate_sha,exact_query_features_verified=True,no_producer_change=True,no_truth_to_match_or_triangulation=True,no_actual_navigation_or_new_network=True,source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
(O/'feature_stage_coverage_audit_v1.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
