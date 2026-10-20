"""Post-close same-initial factual AB; never infers alternate future labels."""
from pathlib import Path
import json,hashlib,datetime
import numpy as np
from independent_path_error import distance_to_complete_reference
H=Path(__file__).resolve().parent;A=H/'native_flat_pose_navigation_R1_run_v1';B=H/'native_terminal_bank_navigation_R1_run_v1';O=H/'native_terminal_bank_AB_verification_v1';O.mkdir(exist_ok=False);sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
for D in (A,B):
 for f,s in json.loads((D/'artifact_manifest_post_close.json').read_text())['artifact_sha256'].items():assert sha(f)==s,f
ra=json.loads((A/'policy_records.json').read_text());rb=json.loads((B/'policy_records.json').read_text());pa=dict(np.load(A/'physics.npz'));pb=dict(np.load(B/'physics.npz'));sa=json.loads((A/'execution_states.json').read_text());sb=json.loads((B/'execution_states.json').read_text())
n=min(len(ra),len(rb));first=next((i for i in range(n) if not np.array_equal(ra[i]['request'],rb[i]['request'])),None)
assert first is not None,'candidate did not change any emitted request';k=5*first
# Predicted event555 is scoped to factualbase; newactual event must be independently found.
physical=['simulation_time_s','root_position_world_m','root_quaternion_world_wxyz','joint_position_rad','joint_velocity_rad_s','applied_action','smoothed_target']
fields={key:bool(np.array_equal(pa[key][:k+1],pb[key][:k+1])) for key in physical}
requestprefix=all(np.array_equal(ra[i]['request'],rb[i]['request']) for i in range(first));controllerprefix=all(ra[i]['local']['tracker_state_after_act']==rb[i]['local']['tracker_state_after_act'] for i in range(140,first));execprefix=sa[:first+1]==sb[:first+1]
local=rb[first]['local'];fresh_goal=local['terminal_goal_bank']['parent_request_source']=='goal_braking';parent_request_matches=np.array_equal(local['parent_final_request'],ra[first]['request']);mains_unchanged=np.array_equal(np.asarray(local['parent_final_request'])[::2],np.asarray(rb[first]['request'])[::2]);side_effects_only_terminal=fresh_goal and parent_request_matches and mains_unchanged
sourcefields=0;sourceequal=True
for f in sorted((A/'sensor_packets').glob('source_*.npz')):
 frame=int(f.stem.split('_')[1])
 if frame>first:continue
 g=B/'sensor_packets'/f.name
 if not g.exists():sourceequal=False;continue
 aa=np.load(f);bb=np.load(g)
 if aa.files!=bb.files:sourceequal=False
 for key in aa.files:
  same=key in bb.files and np.array_equal(aa[key],bb[key]);sourceequal &= same;sourcefields+=1
seedA=np.load(A/'frozen_memory_seed.npz');seedB=np.load(B/'frozen_memory_seed.npz');seedequal=seedA.files==seedB.files and all(np.array_equal(seedA[key],seedB[key]) for key in seedA.files)
common=min(len(pa['applied_action']),len(pb['applied_action']));different=np.flatnonzero(np.any(pa['applied_action'][:common]!=pb['applied_action'][:common],axis=1));appliedfirst=int(different[0]) if len(different) else None
# Compare actualpostevent tracks against SAME baseline reference; index is never the evaluator.
ref=np.asarray(ra[first]['local']['installed_reference_xy_after_act'],float);assert ref.ndim==2 and ref.shape[0]>=2
cross={}
for tag,D,ph in [('parent',A,pa),('candidate',B,pb)]:
 xy=ph['root_position_world_m'];times=ph['simulation_time_s'];valid=times>=first*.1-1e-5;e=np.array([distance_to_complete_reference(p[:2],ref) for p in xy[valid]]);goal=np.linalg.norm(xy-[2.2,.15,.45],axis=1);cross[tag]=dict(postevent_frozen_reference_RMS_m=float(np.sqrt((e**2).mean())),postevent_frozen_reference_peak_m=float(e.max()),closest_true_XYZ_goal_m=float(goal.min()),final_true_XYZ_goal_m=float(goal[-1]),first_true_goal_le02_time_s=float(times[np.flatnonzero(goal<=.2)[0]]) if np.any(goal<=.2) else None,final_axis_progress_m=float(xy[-1,0]-xy[0,0]),final_time_s=float(times[-1]))
gates=dict(physical_prefix=all(fields.values()),request_prefix=requestprefix,controller_prefix=controllerprefix,execution_prefix=execprefix,source_prefix=sourceequal,seed=seedequal,current_terminal_only_source=side_effects_only_terminal,execution_behavior_changes=appliedfirst is not None and appliedfirst>k)
r=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),decision='PASS_SAME_INITIAL_TERMINAL_INTERVENTION_AB_INTERFACE_ONLY' if all(gates.values()) else 'REJECT_STRICT_TERMINAL_AB_IDENTITY',gates=gates,first_request_difference_frame=first,common_physical_prefix_end_index=k,physical_prefix_fields=fields,source_fields_compared=sourcefields,first_applied_difference_index=appliedfirst,fresh_goal_braking= fresh_goal,independent_frozen_reference_xy=ref.tolist(),actual_geometry=cross,not_formal35_or_worldmodel=True,not_automatic_navigation_success=True,whole_research_complete=False,input_sha256={str(D/'artifact_manifest_post_close.json'):sha(D/'artifact_manifest_post_close.json') for D in (A,B)},code_sha256=sha(__file__))
(O/'assessment.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
