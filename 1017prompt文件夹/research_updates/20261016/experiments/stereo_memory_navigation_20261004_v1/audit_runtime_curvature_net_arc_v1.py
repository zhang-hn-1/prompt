from pathlib import Path
import hashlib,json
import numpy as np
from runtime_geometric_curvature_v1 import measure
from runtime_curvature_net_arc_v1 import net_course_arc
H=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
source=H/'runtime_geometric_curvature_assessment_v1.json';old=json.loads(source.read_text());results={}
for arm,assessment in old['arms'].items():
 p=H/f'bounded_prefix_e1_{arm}_run_v1/physics.npz';assert sha(p)==old['input_sha256'][str(p)]
 raw=np.load(p);rows=[]
 for w in assessment['windows']:
  sl=slice(5*w['start_frame'],5*w['end_frame_exclusive']+1)
  position=raw['position_world_xy'][sl]
  m=measure(raw['simulation_time_s'][sl],position,raw['linear_velocity_world_m_s'][sl,:2],raw['root_quaternion_world_wxyz'][sl],raw['angular_velocity_world_rad_s'][sl])
  r=net_course_arc(position,m['course_rad'],m['planar_speed_m_s'])
  rows.append(dict(start_frame=w['start_frame'],end_frame_exclusive=w['end_frame_exclusive'],geometry_identifiable=w['geometry_identifiable'],oscillatory=w['oscillatory'],nominal_course_curvature_1_m=w['nominal_course_curvature_1_m'],instantaneous_threepoint_median_1_m=w['measured_threepoint_curvature']['median'],net_course_arc=r))
 results[arm]=rows
out=H/'runtime_curvature_net_arc_assessment_v1.json';assert not out.exists()
out.write_text(json.dumps(dict(arms=results,input_sha256={str(source):sha(source)},source_sha256={str(p):sha(p) for p in [Path(__file__).resolve(),H/'runtime_curvature_net_arc_v1.py',H/'runtime_curvature_net_arc_scope_v1.json']},retrospective_only=True,no_model_or_control_admission=True),indent=2)+'\n')
print(json.dumps(results,indent=2))
