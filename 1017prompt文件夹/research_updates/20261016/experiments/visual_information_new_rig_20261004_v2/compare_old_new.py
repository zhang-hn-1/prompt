"""Compare new measurements to preserved previous-stage measurements."""
import json
from pathlib import Path
import numpy as np
HERE=Path(__file__).resolve().parent;OLD=HERE.parent/'visual_information_diagnostic_20261003_v1'
new_depth=json.loads((HERE/'depth_assessment.json').read_text());old_depth=json.loads((OLD/'depth_assessment.json').read_text())
new_runs=json.loads((HERE/'run_assessment.json').read_text());old_runs=json.loads((OLD/'run_assessment.json').read_text())
comparison={'scope':'matched familiar development cases; new physical captures, no fresh held-out claim','depth':{},'navigation':{},'tube_coverage':{},'replay_state_alignment':{}}
for model in ('da3_metric','unidepth_v2'):
    a=old_depth[model];b=new_depth[model]
    comparison['depth'][model]={'old_accepted':a['accepted'],'new_accepted':b['accepted'],'old_latency':a['latency'],'new_latency':b['latency'],'sequences':{}}
    for seq in b['sequences']:
        comparison['depth'][model]['sequences'][seq]={group:dict(old=a['sequences'][seq][group],new=b['sequences'][seq][group]) for group in ('scene','obstacle','near_obstacle','far_obstacle')}
for label in ('e0','e2','h3','h4'):
    comparison['tube_coverage'][label]=dict(old=old_runs['initial_nominal_tube_coverage'][label],new=new_runs['initial_nominal_tube_coverage'][label])
    for mode in ('map','visible'):
        key=mode+'_'+label
        roots={version:base/'runs'/mode/label for version,base in [('old',OLD),('new',HERE)]}
        fields=('position_world_xy','yaw_world_rad','quaternion_world_wxyz_history','proprio_history','action_history','pointgoal_base_xy')
        a=np.load(roots['old']/'online_inputs/frame_000.npz');b=np.load(roots['new']/'online_inputs/frame_000.npz')
        differences={field:float(np.max(np.abs(a[field]-b[field]))) for field in fields}
        assert all(v==0 for v in differences.values())
        comparison['navigation'][key]=dict(old=old_runs['runs'][key],new=new_runs['runs'][key],old_new_initial_state_max_abs_difference=differences)
for p in sorted((HERE/'capture').glob('*/capture_summary.json')):
    row=json.loads(p.read_text());comparison['replay_state_alignment'][row['sequence']]=row['old_new_state_max_abs_differences']
(HERE/'old_new_comparison.json').write_text(json.dumps(comparison,indent=2)+'\n')
print(json.dumps({'models':{m:d['new_accepted'] for m,d in comparison['depth'].items()},'navigation':{k:dict(old=v['old']['termination'],new=v['new']['termination'],new_nonzero_requests=v['new']['nonzero_request_blocks']) for k,v in comparison['navigation'].items()},'tube':comparison['tube_coverage']},indent=2))
