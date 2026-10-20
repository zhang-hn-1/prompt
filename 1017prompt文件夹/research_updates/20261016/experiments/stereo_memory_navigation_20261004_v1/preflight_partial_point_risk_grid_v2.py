from pathlib import Path
import json,hashlib,numpy as np,dataclasses
from partial_point_risk_grid import PartialPointRiskGrid
from execution_domain_four_point_consumer_v2 import local_curve_for_direction_tube
from benchmark.policies import _default_drive_model
H=Path(__file__).resolve().parent;M=H/'fixed_appearance_raw_point_memory_v1';D=H/'native_fixed_appearance_probe_run_v1';O=H/'partial_point_risk_grid_preflight_v2';O.mkdir(exist_ok=False);(O/'maps').mkdir();sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
seal=json.loads((M/'seal.json').read_text())
for group in ('inputs','source_sha256','outputs'):
 for f,s in seal[group].items():assert sha(f)==s,f
init=np.load(D/'estimated_poses/frame_000.npz');g=PartialPointRiskGrid(init['root_position_world_m'][:2]-20,10);rows=[];inputs={str(M/'seal.json'):sha(M/'seal.json')}
for r in json.loads((M/'memory_shadow_rows.json').read_text()):
 i=r['frame'];f=M/'tentative_memory_snapshots'/f'frame_{i:03d}.json';pf=D/'estimated_poses'/f'frame_{i:03d}.npz';inputs[str(f)]=sha(f);inputs[str(pf)]=sha(pf);memory=json.loads(f.read_text());pose=np.load(pf);t=r['time_s'];g.consume_records(memory['records'],t,pose['root_position_world_m']);assert not (g.labels==0).any();assert not np.isfinite(g.memory.floor_time).any();assert not np.isfinite(g.hypothesis_birth_time).any();assert g.birth is None
 np.savez_compressed(O/'maps'/f'frame_{i:03d}.npz',labels=g.labels,codes=g.codes,hazard_acquisition_time=g.hazard_acquisition_time,origin=g.origin,resolution=g.resolution,safe_start_prior=g.safe_start_prior,ground_prior=g.task_ground_prior,time_s=t)
 rows.append(dict(frame=i,time_s=t,current_cells=int((g.codes==8).sum()),past_cells=int((g.codes==9).sum()),partial_surface_cells=int((g.labels==1).sum())))
# Exactly one frozen final14s source state; no selecting other timestamps or budgets.
yaw=float(np.arctan2(pose['root_rotation_world'][1,0],pose['root_rotation_world'][0,0]));plan,meta=local_curve_for_direction_tube(g,pose['root_position_world_m'][:2],yaw,np.array([2.2,.15]),_default_drive_model(),initial_direction=None)
a=dict(decision='PASS_SINGLE_STATE_NOMINAL_POINT_RISK_PLANNING_ONLY' if plan is not None else 'REJECT_SINGLE_STATE_POINT_RISK_PLANNING',maps=len(rows),final=rows[-1],plan_meta=meta,waypoints=[] if plan is None else [dataclasses.asdict(w) for w in plan.waypoints],archival_development=True,no_online_or_action_validation=True,partial_surface_not_full_volume=True,old_MODEL_gate_remains_rejected=True,formal35_NOTRUN=True,whole_research_complete=False)
(O/'rows.json').write_text(json.dumps(rows,indent=2)+'\n');(O/'assessment.json').write_text(json.dumps(a,indent=2)+'\n');(O/'identity.json').write_text(json.dumps(dict(inputs=inputs,source_sha256={str(H/n):sha(H/n) for n in ['preflight_partial_point_risk_grid_v2.py','partial_point_risk_grid.py','execution_domain_four_point_consumer_v2.py','typed_hypothesis_grid.py','prior_rgb_navigation.py']},outputs={str(f):sha(f) for f in O.rglob('*') if f.is_file()}),indent=2)+'\n');print(json.dumps({k:v for k,v in a.items() if k not in ('waypoints','plan_meta')},indent=2));print(json.dumps({k:v for k,v in meta.items() if k!='route_xy'},indent=2))
