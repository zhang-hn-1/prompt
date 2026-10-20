"""Extract only existing current frame175 evidence; no search/action."""
from pathlib import Path
import json,numpy as np,hashlib
H=Path(__file__).resolve().parent;O=H/'endpoint_navigation_run';i=175
if __name__=='__main__':
 record=json.loads((O/'policy_records.json').read_text())[i]
 inp=np.load(O/'policy_inputs'/f'frame_{i:03d}.npz');mp=np.load(O/'maps'/f'frame_{i:03d}.npz')
 assert record['frame']==i and record['local']['reference_decision']['measured_time_s']==float(mp['time_s'])
 current=dict(frame=i,time_s=float(mp['time_s']),position_world_xy=inp['position_world_xy'].tolist(),yaw_world_rad=float(inp['yaw_world_rad']),pointgoal_base_xy=inp['pointgoal_base_xy'].tolist(),accepted_direction=record['local']['reference_decision']['state_before']['active_direction'])
 evidence=dict(current=current,current_tracking=record['tracking'],original_plan_metadata=record['local']['plan'],state_before=record['local']['reference_decision']['state_before'],selection='single current frame175; later frames never copied')
 (H/'nominal_support_current175.json').write_text(json.dumps(evidence,indent=2)+'\n')
 # Preserve complete current execution identity, not used to manufacture a physical prediction.
 state=json.loads((O/'execution_states.json').read_text())[i]
 (H/'nominal_support_execution175.json').write_text(json.dumps(state,indent=2)+'\n')
 print('current175 extracted, no search/act')
