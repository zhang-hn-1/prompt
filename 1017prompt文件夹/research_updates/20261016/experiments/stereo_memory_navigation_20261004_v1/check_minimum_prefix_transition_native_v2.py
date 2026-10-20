from pathlib import Path
import json,hashlib,ast
from prior_rgb_navigation import PriorRGBPolicy
from execution_domain_four_point_consumer_v2 import with_supported_four_points
from retained_reference_tube_viability_v1 import with_tube_viability_after_measured_reversal
from minimum_prefix_transition_consumer_v1 import with_first_primitive_direction
from cusp_coast_acquisition_v1 import with_cusp_coast_acquisition
from command_domain_cap import with_command_domain_cap
from benchmark.policies import EpisodeContext
from scripts.run_pointgoal_benchmark import load_action_protocol
H=Path(__file__).resolve().parent
r=json.loads((H/'minimum_prefix_transition_navigation_protocol_v1.json').read_text())
for key in ['source_sha256','input_sha256']:
 for f,sha in r[key].items():assert hashlib.sha256(Path(f).read_bytes()).hexdigest()==sha,f
for f in ['minimum_prefix_transition_planning_v1.py','minimum_prefix_transition_consumer_v1.py','run_minimum_prefix_transition_navigation_v1.py','run_minimum_prefix_transition_navigation_stage_v1.py']:ast.parse((H/f).read_text())
proto=load_action_protocol(Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/configs/data_contract.yaml'))
cls=with_command_domain_cap(with_cusp_coast_acquisition(with_first_primitive_direction(with_tube_viability_after_measured_reversal(with_supported_four_points(PriorRGBPolicy)))))
p=cls(proto,memory_mode='current');p.reset(EpisodeContext('golden_e1_0001',False,None,proto.protocol_id))
assert p.current_first_direction is None and not p.tube_viability_armed
out=dict(source_checks=len(r['source_sha256']),input_checks=len(r['input_sha256']),syntax=True,native_import=True,policy_MRO=[str(c) for c in cls.__mro__],reset_no_act=True,not_physics=True)
(H/'minimum_prefix_transition_native_preflight_v2.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out,indent=2))
