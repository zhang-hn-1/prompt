from pathlib import Path
import inspect,json,hashlib,datetime
import run_original_E1_repeated_exposure_memory_navigation_v1 as d
from nonbinding_side_hypothesis_v1 import NonbindingSideHypothesisGrid
from repeated_exposure_memory_v1 import RepeatedExposureMemoryGrid
h=Path(__file__).resolve().parent
protocol=d.load_action_protocol(d.DEFAULT_CONTRACT.resolve())
factory=d.with_command_domain_cap(d.with_cusp_coast_acquisition(d.with_tube_viability_after_measured_reversal(d.with_supported_four_points(d.PriorRGBPolicy))))
a=factory(protocol,memory_mode='current');b=factory(protocol,memory_mode='current')
for p in (a,b):p.reset(d.EpisodeContext('golden_e1_0001',False,None,protocol.protocol_id))
a.grid=RepeatedExposureMemoryGrid([-23.3,-20],10,static_scene_assumption=True);b.grid=NonbindingSideHypothesisGrid([-23.3,-20],10,static_scene_assumption=True)
assert type(a) is type(b)
assert [(c.__module__,c.__qualname__,inspect.getsourcefile(c)) for c in type(a.tracker).__mro__ if c is not object]==[(c.__module__,c.__qualname__,inspect.getsourcefile(c)) for c in type(b.tracker).__mro__ if c is not object]
r=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),decision='PASS_NATIVE_IMPORT_CLASS_IDENTITY_AND_RESET_ONLY',simulation_initialized=False,NN_initialized=False,policy_MRO=[str(c) for c in type(a).__mro__],tracker_MRO=[str(c) for c in type(a.tracker).__mro__ if c is not object],baseline_policy_MRO=[str(c) for c in type(b).__mro__],baseline_tracker_MRO=[str(c) for c in type(b.tracker).__mro__ if c is not object],candidate_grid=str(type(a.grid)),baseline_grid=str(type(b.grid)),candidate_plan_source=inspect.getsourcefile(a.grid.plan),driver_sha256=hashlib.sha256((h/'run_original_E1_repeated_exposure_memory_navigation_v1.py').read_bytes()).hexdigest(),whole_research_complete=False)
(h/'original_E1_repeated_exposure_memory_native_identity_preflight_v1.json').write_text(json.dumps(r,indent=2)+'\n');print(r['decision'])
