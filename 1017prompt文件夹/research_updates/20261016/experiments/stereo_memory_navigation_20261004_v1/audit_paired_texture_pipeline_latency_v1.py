"""Existing whole-run wall-clock decomposition; no new inference or act calls."""
from pathlib import Path
import json,numpy as np,hashlib,datetime
H=Path(__file__).resolve().parent;D=H/'live_paired_texture_navigation_run_v1';O=H/'paired_texture_pipeline_latency_v1';O.mkdir(exist_ok=False);rows=json.loads((D/'policy_records.json').read_text());sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
for f,s in json.loads((D/'artifact_manifest_post_close.json').read_text())['artifact_sha256'].items():assert sha(Path(f))==s
q=lambda a:dict(n=len(a),total_s=float(np.sum(a)),p50_p90_max_s=np.percentile(a,[50,90,100]).tolist()) if len(a) else dict(n=0)
groups={}
for name,predicate in [('all',lambda r:True),('execute_includes_next_NN_epoch',lambda r:(r['frame']+1)%5==0),('execute_without_next_NN_epoch',lambda r:(r['frame']+1)%5!=0),('post14_policy',lambda r:r['planner_act_called'])]:
 a=[r for r in rows if predicate(r)];e=np.array([r['execute_capture_NN_wall_s'] for r in a]);total=np.array([r['end_to_end_wall_s'] for r in a]);policy=np.array([r.get('policy_wall_s',0.) for r in a]);residual=total-e-policy;assert (residual>=-1e-8).all()
 groups[name]=dict(execute_render_capture_source_and_NN_bundle=q(e),end_to_end=q(total),policy_only=q(policy),other_observation_map_serialization=q(residual),over100ms_blocks=int((total>.1).sum()))
r=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),decision='NOT_REALTIME_ALL_ACTION_BLOCKS_EXCEED100MS',groups=groups,action_block_simulation_s=.1,NN_acquisition_period_simulation_s=.5,physics_paused_for_computation=True,NN_only_timer_NOT_recorded=True,source_geometry_serialization_render_not_separable_in_bundle=True,no_unique_NN_latency_cause_claim=True,no_controller_or_network_or_capture_change=True,not_another_online_run=True,code_sha256=sha(Path(__file__)),whole_complete=False);(O/'assessment.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
