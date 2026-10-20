"""One fixed curved-command nominal view contrast, not a physical predictor."""
from pathlib import Path
import json,hashlib,math
import numpy as np
from bearing_observability_v1 import build_context,assess_view
from execution_domain_four_point_source_v2 import supported_curvature,source_identity
from remaining_reference_v1 import project_remaining
H=Path(__file__).resolve().parent;D=H/'minimum_prefix_transition_navigation_run_v1';O=H/'retreat_observation_development_v2_history_adapter';O.mkdir(exist_ok=False)
p=H/'retreat_observation_protocol_v1.json';proto=json.loads(p.read_text());s=H/'left_temporal_missing_divider_development_v1/predictions.json';pred=json.loads(s.read_text());inputs={str(p):hashlib.sha256(p.read_bytes()).hexdigest(),str(s):hashlib.sha256(s.read_bytes()).hexdigest()};rows=[]
K=np.array([[584.8888888888889,0,319.5],[0,584.8888888888889,239.5],[0,0,1.]])
for f in [proto['frame']]:
    pp=D/'known_poses'/f'frame_{f:03d}.npz';hp=D/'history'/f'frame_{f:03d}.npz'
    for file in [pp,hp]:inputs[str(file)]=hashlib.sha256(file.read_bytes()).hexdigest()
    pose=np.load(pp);hist=np.load(hp);q=hist['quaternion_world_wxyz_history'][-1];w,x,y,z=q;yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z));root=pose['root_position_world_m'];a=pose['left_camera_usd_world'];xy=next(r['prediction']['query_xy'] for r in pred if r['frame']==f);context=build_context(xy,a,K,float(pose['time_s']),f);groups=[]
    for name in ['candidate']:
        k=supported_curvature(proto['planned_carrier_main'],proto['raw_aux_rad']);length=proto['signed_arc_m'];theta=k*length
        dx=length*math.cos(yaw) if abs(k)<1e-12 else (math.sin(yaw+theta)-math.sin(yaw))/k
        dy=length*math.sin(yaw) if abs(k)<1e-12 else (-math.cos(yaw+theta)+math.cos(yaw))/k
        R=np.array([[math.cos(theta),-math.sin(theta),0],[math.sin(theta),math.cos(theta),0],[0,0,1.]])
        b=a.copy();b[:3,:3]=R@a[:3,:3];b[:3,3]=root+np.array([dx,dy,0.])+R@(a[:3,3]-root)
        result=assess_view(context,b,proto['range_hypotheses_m']);file=O/f'frame_{f:03d}_{name}.json';file.write_text(json.dumps(result,indent=2)+'\n');summaries=[]
        for depth in proto['range_hypotheses_m']:
            subset=[r for r in result['rows'] if r['optical_depth_hypothesis_m']==depth];summaries.append(dict(depth_hypothesis_m=depth,features=len(subset),in_image=sum(r['in_image'] for r in subset),sigma_pass=sum(r['sigma_pass'] for r in subset)))
        groups.append(dict(arm=name,curvature_1_m=k,nominal_camera_displacement_m=float(np.linalg.norm(b[:3,3]-a[:3,3])),by_depth=summaries))
    statesfile=D/'states.json';inputs[str(statesfile)]=hashlib.sha256(statesfile.read_bytes()).hexdigest();states=json.loads(statesfile.read_text());prefix=states[:f+1]
    assert len(prefix)==f+1 and abs(prefix[-1]['time_s']-float(pose['time_s']))<1e-8
    assert all(r['time_s']<=float(pose['time_s'])+1e-8 for r in prefix)
    history=[r['position'][:2] for r in prefix]
    samples=np.linspace(0.,length,13);distances=[]
    for arc in samples:
        t=k*arc
        dx=arc*math.cos(yaw) if abs(k)<1e-12 else (math.sin(yaw+t)-math.sin(yaw))/k
        dy=arc*math.sin(yaw) if abs(k)<1e-12 else (-math.cos(yaw+t)+math.cos(yaw))/k
        distances.append(project_remaining(history,root[:2]+[dx,dy])['distance_m'])
    bound=max(distances)+.025
    groups[0]['view_gate']=all(x['sigma_pass']/x['features']>=.95 for x in groups[0]['by_depth'])
    rows.append(dict(frame=f,groups=groups,root_history_prior_max_center_distance_m=max(distances),conservative_sample_gap_bound_m=bound,historical_prior_necessary_gate=bound<=.2,history_is_prior_not_observed_free=True,actual_safe_transition_established=False))
(O/'assessment.json').write_text(json.dumps(dict(rows=rows,nominal_model_source=source_identity(),physical_execution_validated=False,source_coverage_validated=False,control_candidate_admitted=False,no_true_depth_or_future_feedback=True,whole_research_complete=False),indent=2)+'\n')
files=[file for file in O.iterdir() if file.is_file()];(O/'seal.json').write_text(json.dumps(dict(input_sha256=inputs,output_sha256={str(file):hashlib.sha256(file.read_bytes()).hexdigest() for file in files},source_sha256={str(file):hashlib.sha256(file.read_bytes()).hexdigest() for file in [Path(__file__),H/'bearing_observability_v1.py',H/'execution_domain_four_point_source_v2.py',H/'supported_forward_primitive_source.py',H/'remaining_reference_v1.py']},no_truth_labels_read=True),indent=2)+'\n');print(json.dumps(rows,indent=2))
