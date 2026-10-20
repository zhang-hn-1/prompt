"""Actual sealed SAME-time reference snapshots, read-only diagnostic consumption."""
from pathlib import Path
import json,hashlib,copy
import numpy as np
from bearing_observability_v1 import build_context
from uncertain_bearing_memory_v1 import UncertainBearingMemory
from reference_bearing_ambiguity_v1 import assess_reference
H=Path(__file__).resolve().parent;D=H/'minimum_prefix_transition_navigation_run_v1';O=H/'reference_bearing_ambiguity_development_v1';O.mkdir(exist_ok=False)
proto=H/'reference_bearing_ambiguity_protocol_v1.json';protocol=json.loads(proto.read_text());p=H/'left_temporal_missing_divider_development_v1/predictions.json';rp=D/'policy_records.json';pred=json.loads(p.read_text());policies=json.loads(rp.read_text());inputs={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in [proto,p,rp]};packets=[];references=[];K=np.array([[584.8888888888889,0,319.5],[0,584.8888888889,239.5],[0,0,1.]])
for f in protocol['queries']:
    file=D/'known_poses'/f'frame_{f:03d}.npz';inputs[str(file)]=hashlib.sha256(file.read_bytes()).hexdigest();a=np.load(file);xy=next(r['prediction']['query_xy'] for r in pred if r['frame']==f);packets.append(build_context(xy,a['left_camera_usd_world'],K,float(a['time_s']),f));r=policies[f];assert r['frame']==f and abs(r['time_s']-float(a['time_s']))<1e-8
    ref=np.asarray(r['installed_reference_xy_after_act'],float);assert ref.ndim==2 and ref.shape[1]==2 and len(ref)
    references.append(np.c_[ref,np.full(len(ref),float(a['root_position_world_m'][2]))])
def replay(seq,refs):
    memory=UncertainBearingMemory();rows=[]
    for context,reference in zip(seq,refs):
        now=context['time_s'];memory.update(context,now,0.);snapshot=memory.snapshot(now);assessment=assess_reference(snapshot,reference,now)
        rows.append(dict(frame=context['frame'],time_s=now,reference_points=len(reference),reference_snapshot_same_frame=True,assessment=assessment))
    return rows
rows=replay(packets,references);short=replay(packets[:3],references[:3]);assert rows[:3]==short
poison=copy.deepcopy(references);poison[3][:]=np.nan;assert replay(packets[:3],poison[:3])==short
result=dict(rows=rows,truncated_prefix_exact=True,poisoned_future_reference_prefix_exact=True,no_policy_act_called=True,no_reference_or_request_changed=True,knownGT_pose_diagnostic=True,replay_not_online=True,no_true_depth_or_instances_read=True,no_semantic_obstacle_prediction=True,whole_research_complete=False)
(O/'assessment.json').write_text(json.dumps(result,indent=2)+'\n');files=[f for f in O.iterdir() if f.is_file()];(O/'seal.json').write_text(json.dumps(dict(input_sha256=inputs,source_sha256={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in [Path(__file__),H/'reference_bearing_ambiguity_v1.py',H/'uncertain_bearing_memory_v1.py',H/'bearing_observability_v1.py',H/'feature_tracks.py']},output_sha256={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in files},no_truth_labels_read=True),indent=2)+'\n');print([(r['frame'],r['assessment']['ambiguous_bearings'],r['assessment']['tested_bearings']) for r in rows])
