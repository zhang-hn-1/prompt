"""Three fixed measured states; establish cacheability before optimization."""
from pathlib import Path
import json,time,hashlib,numpy as np
from dynamic_self_mask import RobotSelfMask,joints_from_proprio
H=Path(__file__).resolve().parent;D=H/'live_paired_texture_navigation_run_v1';O=H/'CAD_static_part_workload_v1';O.mkdir(exist_ok=False);cad=RobotSelfMask();rows=[];prior=None
for i in [0,150,295]:
 q=joints_from_proprio(np.load(D/'history'/f'frame_{i:03d}.npz')['proprio_history']);m={};timing={}
 for name,parts in [('base',('base_link',)),('dynamic',('link1','link2','link3')),('whole',('base_link','link1','link2','link3'))]:
  begin=time.monotonic();m[name]=cad.project(q,parts=parts,include_frozen=False);timing[name]=time.monotonic()-begin
 for eye in ('left','right'):
  np.testing.assert_array_equal(m['base'][eye]|m['dynamic'][eye],m['whole'][eye])
  if prior is not None:np.testing.assert_array_equal(m['base'][eye],prior[eye])
 prior=m['base'];rows.append(dict(frame=i,current_measured_joints=q.tolist(),wall_s=timing,base_joint_independent_on_three_states=True,OR_composition_exact=True))
result=dict(triangles_by_part={k:len(v) for k,v in cad.triangles.items()},rows=rows,static_base_transform_identity_by_actual_code=True,scope='CPU known CAD mask only; no NN, action, source or new safety admission',no_original_code_changed=True,not_realtime_claim=True,whole_complete=False);(O/'assessment.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
