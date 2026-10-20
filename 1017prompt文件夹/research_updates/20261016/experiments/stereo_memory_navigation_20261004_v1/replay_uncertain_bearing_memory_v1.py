"""Frozen six-packet causal memory replay, not online sensor/actor validation."""
from pathlib import Path
import json,hashlib,copy
import numpy as np
from bearing_observability_v1 import build_context
from uncertain_bearing_memory_v1 import UncertainBearingMemory
H=Path(__file__).resolve().parent;D=H/'minimum_prefix_transition_navigation_run_v1';O=H/'uncertain_bearing_memory_development_v1';O.mkdir(exist_ok=False)
proto=H/'uncertain_bearing_memory_protocol_v1.json';protocol=json.loads(proto.read_text());p=H/'left_temporal_missing_divider_development_v1/predictions.json';data=json.loads(p.read_text());inputs={str(p):hashlib.sha256(p.read_bytes()).hexdigest(),str(proto):hashlib.sha256(proto.read_bytes()).hexdigest()};packets=[];K=np.array([[584.8888888888889,0,319.5],[0,584.8888888888889,239.5],[0,0,1.]])
for f in protocol['queries']:
    file=D/'known_poses'/f'frame_{f:03d}.npz';inputs[str(file)]=hashlib.sha256(file.read_bytes()).hexdigest();a=np.load(file);xy=next(r['prediction']['query_xy'] for r in data if r['frame']==f);packets.append(build_context(xy,a['left_camera_usd_world'],K,float(a['time_s']),f))
def replay(seq):
    m=UncertainBearingMemory();rows=[]
    for context in seq:
        m.update(context,context['time_s'],0.);s=m.snapshot(context['time_s']);rows.append(dict(frame=context['frame'],snapshot=s,epochs=len(s),bearings=sum(len(r['query_xy']) for r in s)))
    return rows
rows=replay(packets);truncated=replay(packets[:3]);assert rows[:3]==truncated
# A valid later packet with no corrupted fields must still be refused at the earlier clock.
m=UncertainBearingMemory();m.update(packets[0],packets[0]['time_s'],0.);before=m.snapshot(packets[0]['time_s']);rejected=False
try:m.update(packets[-1],packets[0]['time_s'],0.)
except ValueError:rejected=True
assert rejected and before==m.snapshot(packets[0]['time_s'])
poison=copy.deepcopy(packets);poison[3]['xy'][:]=np.nan;assert replay(poison[:3])==truncated
assert all(not r['observed_depth'] and not r['certified_free'] for row in rows for r in row['snapshot'])
result=dict(rows=rows,truncated_prefix_exact=True,poisoned_future_prefix_exact=True,future_valid_capture_rejected_no_mutation=True,knownGT_pose_diagnostic_only=True,no_observed_depth_or_free=True,no_actor_change=True,replay_not_online=True,whole_research_complete=False)
(O/'assessment.json').write_text(json.dumps(result,indent=2)+'\n');files=[f for f in O.iterdir() if f.is_file()];(O/'seal.json').write_text(json.dumps(dict(input_sha256=inputs,source_sha256={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in [Path(__file__),H/'uncertain_bearing_memory_v1.py',H/'bearing_observability_v1.py']},output_sha256={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in files},no_truth_labels_read=True),indent=2)+'\n');print([(r['frame'],r['epochs'],r['bearings']) for r in rows])
