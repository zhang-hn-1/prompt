"""Two preregistered ideal translations on current-only frozen pixel bearings."""
from pathlib import Path
import json,hashlib
import numpy as np
from feature_tracks import rotation
from bearing_observability_v1 import build_context,assess_view
H=Path(__file__).resolve().parent;D=H/'minimum_prefix_transition_navigation_run_v1';O=H/'bearing_observability_development_v1';O.mkdir(exist_ok=False)
p=H/'bearing_observability_protocol_v1.json';protocol=json.loads(p.read_text());source=H/'left_temporal_missing_divider_development_v1/predictions.json';pred=json.loads(source.read_text());K=np.array([[584.8888888888889,0,319.5],[0,584.8888888888889,239.5],[0,0,1.]])
inputs={str(p):hashlib.sha256(p.read_bytes()).hexdigest(),str(source):hashlib.sha256(source.read_bytes()).hexdigest()};rows=[]
for current in protocol['queries']:
    raw=next(r for r in pred if r['frame']==current);q=raw['prediction']['query_xy'];path=D/'known_poses'/f'frame_{current:03d}.npz';inputs[str(path)]=hashlib.sha256(path.read_bytes()).hexdigest();z=np.load(path);a=z['left_camera_usd_world'];c=build_context(q,a,K,float(z['time_s']),current);groups=[]
    for name,shift in [('optical_forward',[0.,0.,.3]),('optical_transverse',[.3,0.,0.])]:
        b=a.copy();b[:3,3]+=rotation(a)@np.array(shift);r=assess_view(c,b,protocol['range_hypotheses_optical_depth_m']);by_depth=[]
        for depth in protocol['range_hypotheses_optical_depth_m']:
            v=[x for x in r['rows'] if x['optical_depth_hypothesis_m']==depth];by_depth.append(dict(depth_hypothesis_m=depth,query_count=len(v),visible=sum(x['in_image'] for x in v),geometry_valid=sum(x['geometry_valid'] for x in v),sigma_pass=sum(x['sigma_pass'] for x in v)))
        namefile=f'frame_{current:03d}_{name}.json';(O/namefile).write_text(json.dumps(r,indent=2)+'\n');groups.append(dict(ideal_translation=name,by_depth=by_depth,result_file=namefile))
    rows.append(dict(frame=current,query_features=len(q),groups=groups))
(O/'assessment.json').write_text(json.dumps(dict(rows=rows,conditional_hypotheses_only=True,no_model_or_source_admission=True,no_physical_motion_prediction=True,no_actual_run=True,no_GT_depth_instances_read=True,no_future_feedback=True,whole_research_complete=False),indent=2)+'\n')
files=[f for f in O.iterdir() if f.is_file()];(O/'seal.json').write_text(json.dumps(dict(input_sha256=inputs,source_sha256={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in [Path(__file__),H/'bearing_observability_v1.py',H/'feature_tracks.py']},output_sha256={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in files},no_truth_labels_read=True),indent=2)+'\n');print(json.dumps(rows,indent=2))
