from pathlib import Path
from types import SimpleNamespace
import sys,json,numpy as np
D=Path(__file__).resolve().parent;W=D.parents[1];P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm');H=W/'experiments/stereo_memory_navigation_20261004_v1';sys.path[:0]=[str(D),str(H),str(P),str(W/'experiments/visible_surface_connectivity_20261004_v1')]
from scripts.run_pointgoal_benchmark import load_action_protocol
from scripts.run_tracking_acceptance import DEFAULT_CONTRACT
from outgoing_lookahead import make_endpoint_oracle
from cusp_coast_acquisition_v1 import with_cusp_coast_tracker
from field import with_tangent_field,target
p=load_action_protocol(DEFAULT_CONTRACT.resolve());cases=[]
for direction in ['forward','reverse']:
 for y in [-.2,0,.2]:
  t=make_endpoint_oracle(p);t.__class__=with_tangent_field(with_cusp_coast_tracker(type(t)));t._reference=tuple(SimpleNamespace(drive_direction=direction) for _ in range(3));t._reference_xy=np.array([[0,0],[1,0],[2,0]],float);t._path_index=0;t._tracking_audit={};t._field_position=np.array([.2,y]);z=t._lookahead_target(.5);assert np.allclose(z,target([.2,y],[0,0],[1,0],.5));assert t.field_last['current_only'];cases.append(z.tolist())
t._reference=(SimpleNamespace(drive_direction='forward'),SimpleNamespace(drive_direction='reverse'),SimpleNamespace(drive_direction='reverse'));t.field_last=None;assert np.allclose(t._lookahead_target(.5),[1,0]);assert t.field_last is None
(D/'mro_checks.json').write_text(json.dumps(dict(cases=cases,cusp_unchanged=True,MRO=[x.__name__ for x in type(t).__mro__]),indent=2)+'\n');print('6 fullMRO field checks plus cusp fallback passed')

# Near terminal projected remaining .2m, large lateral error must not inflate it.
t._reference=tuple(SimpleNamespace(drive_direction='forward') for _ in range(3));t._reference_xy=np.array([[0,0],[1,0],[2,0]],float);t._path_index=1;t._field_position=np.array([1.8,.3]);t.field_last=None
assert np.allclose(t._lookahead_target(.5),[2,0]);assert t.field_last is None
print('near terminal offtrack fallback passed')
