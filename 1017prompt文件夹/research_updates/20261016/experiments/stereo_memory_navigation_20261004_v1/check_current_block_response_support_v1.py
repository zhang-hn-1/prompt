from pathlib import Path
import sys,json,hashlib
import numpy as np
H=Path(__file__).resolve().parent;P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm');sys.path[:0]=[str(P),str(H.parent/'oracle_feasible_route_20260929_v9/hybrid')]
from guarded_response import physical_state,applicability,TOL
D=H/'E1_continuous_tube_navigation_run_v1';library=json.loads((H/'guarded_response_library.json').read_text());ph=np.load(D/'physics.npz');pred=json.loads((D/'controller_prefix_clone_predictions_v4.json').read_text());results=[]
for row in pred['records']:
 i=row['boundary'];tick=i*5;state=physical_state(ph['root_position_world_m'][tick],ph['root_quaternion_world_wxyz'][tick],ph['linear_velocity_world_m_s'][tick],ph['angular_velocity_world_rad_s'][tick],ph['joint_position_rad'][tick],ph['joint_velocity_rad_s'][tick]);execution=json.loads((D/'execution'/f'frame_{i:03d}.json').read_text());checks=[]
 for e in library:
  gate=applicability(state,execution,e);requested=np.asarray(row['uncapped_request']).reshape(5,2);known=np.asarray(e['requests'])[0].reshape(5,2);checks.append(dict(name=e['name'],original_gate=gate,current_first_block_request_exact=np.array_equal(requested.astype(np.float32),known.astype(np.float32)),current_request=requested[0].tolist(),library_first_request=known[0].tolist(),indivisible_library_duration_s=float(e['times'][-1]),current_known_plan_duration_s=.1))
 results.append(dict(boundary=i,entries=checks,accepted=False,reason='INITIAL_STATE_AND_REQUEST_DOMAIN_UNSUPPORTED_INDIVISIBLE_RESPONSE_NOT_CURRENT_BLOCK_PREDICTOR'))
r=dict(scope='ONE_NECESSARY_READONLY_FIXED221371_EXISTING_LIBRARY_APPLICABILITY_CHECK_NO_NEW_MODEL',results=results,tolerances_unchanged=TOL,old_library_sha256=hashlib.sha256((H/'guarded_response_library.json').read_bytes()).hexdigest(),no_future_current_run_feedback_input=True,no_new_physics=True,no_safety_or_navigation_admission=True);(D/'current_block_response_support_v1.json').write_text(json.dumps(r,indent=2)+'\n');print([(a['boundary'],[(b['name'],b['original_gate']['reasons'],b['current_first_block_request_exact']) for b in a['entries']]) for a in results])
