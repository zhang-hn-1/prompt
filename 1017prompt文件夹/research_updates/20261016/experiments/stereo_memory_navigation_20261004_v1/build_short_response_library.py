"""Build exactly one measured short whole response from prereg development."""
import json,hashlib
from pathlib import Path
import numpy as np
from short_branch import whole_short_entry,development_schedule
from guarded_response import physical_state
H=Path(__file__).resolve().parent
r=H/'short_branch_runs/development';p=np.load(r/'physics.npz');states=json.loads((r/'states.json').read_text());executions=json.loads((r/'execution_states.json').read_text());i=300
assert len(p['simulation_time_s'])==676
req=np.load(r/'requests.npz')['requests'];np.testing.assert_array_equal(req,development_schedule())
state=physical_state(states[60]['position'],p['root_quaternion_world_wxyz'][i],p['linear_velocity_world_m_s'][i],p['angular_velocity_world_rad_s'][i],p['joint_position_rad'][i],p['joint_velocity_rad_s'][i])
c,s=np.cos(state['yaw']),np.sin(state['yaw']);R=np.array([[c,s],[-s,c]])
xy=(p['position_world_xy'][i:]-p['position_world_xy'][i])@R.T
entry=whole_short_entry(dict(state=state,execution=executions[60]),xy,p['simulation_time_s'][i:]-p['simulation_time_s'][i],req[60:])
def conv(v):
    if isinstance(v,np.ndarray):return v.tolist()
    raise TypeError(type(v))
out=H/'short_response_library.json';out.write_text(json.dumps([entry],default=conv,indent=2)+'\n')
files=[r/f for f in ('physics.npz','requests.npz','execution_states.json','states.json','identity.json')]
(H/'short_response_model_manifest.json').write_text(json.dumps(dict(role='one measured development family, no fitting/heldout input',source_indices=[300,675],plan='1.5s main+.3 encodedaux+30 then6s rawzero',library_sha256=hashlib.sha256(out.read_bytes()).hexdigest(),source_sha256={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in files}),indent=2)+'\n')
print('SHORT_LIBRARY_BUILT',len(xy),len(entry['requests']))
