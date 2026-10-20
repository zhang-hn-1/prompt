"""Geometry/request-consumer arithmetic on actual past rows; not policy replay."""
from pathlib import Path
import json,hashlib,math,inspect,sys,numpy as np
H=Path(__file__).resolve().parent;P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm');sys.path.insert(0,str(P));sys.path.insert(0,str(H.parent/'oracle_feasible_route_20260929_v9/hybrid'))
from velocity_pursuit_consumer import proxy_target,with_velocity_pursuit
from outgoing_lookahead import make_endpoint_oracle
from training.action_protocol import load_action_protocol
from bank_mapping import BankMap
protocol=load_action_protocol(P/'configs/data_contract.yaml');tracker=make_endpoint_oracle(protocol);mapping=BankMap(json.loads((H.parent/'oracle_feasible_route_20260929_v9/hybrid/bank_mapping.json').read_text()));run=H/'observation_handoff_observe_run_recovery_v2';records=json.loads((run/'policy_records.json').read_text());states=json.loads((run/'states.json').read_text());rows=[]
for record in records[180:]:
 tr=record['tracking']
 if not (tr.get('valid') and tr.get('velocity_course_valid') and tr.get('reference_motion_tangent_rad') is not None):continue
 i=record['frame'];target=np.array([tr['lookahead_x_m'],tr['lookahead_y_m']]);xy=np.array(states[i]['position'][:2]);v=np.array(states[i]['velocity'][:2]);yaw=tr['root_heading_rad'];main=tr['main_inverse_query_rad_s'];sign=1 if main>0 else -1;speed=tr['world_xy_speed_m_s'];gyro=tr['measured_world_yaw_rate_rad_s']
 def curvature(t):
  d=np.asarray(t)-xy;c,s=math.cos(yaw),math.sin(yaw);b=np.array([c*d[0]+s*d[1],-s*d[0]+c*d[1]]);return 0. if b@b<=1e-9 else float(2*b[1]/(b@b))
 def auxiliary(k):
  correction=float(np.clip(-sign*tracker.yaw_rate_feedback_gain_s*(k*sign*max(speed,.2)-gyro),-math.radians(5),math.radians(5)));choice=mapping.query(main,speed,k);raw=choice['encoded_aux_rad']+correction;return float(np.float32(np.clip(raw,protocol.minimum[1],protocol.maximum[1])))
 oldk=curvature(target);assert math.isclose(oldk,tr['requested_curvature_1_m'],abs_tol=1e-12);oldaux=auxiliary(oldk);assert oldaux==float(np.float32(record['local']['request'][1])),(i,oldaux,record['local']['request'][1])
 proxy,heading,active=proxy_target(target,xy,yaw,v,tr['active_drive_direction']);newk=curvature(proxy);newaux=auxiliary(newk);rows.append(dict(frame=i,old_curvature=oldk,new_curvature=newk,old_protocol_aux=oldaux,new_protocol_aux=newaux,protocol_aux_changes=oldaux!=newaux,proxy=proxy.tolist(),actual_target=target.tolist(),velocity_geometry_heading_rad=heading,active=active,same_saved_main_sign_only=True,not_full_counterfactual_policy_action=True))
assert len(rows)==87
identity=[]
for cls in type(tracker).__mro__:
 try:p=Path(inspect.getsourcefile(cls));identity.append(dict(class_name=cls.__module__+'.'+cls.__qualname__,source=str(p),SHA256=hashlib.sha256(p.read_bytes()).hexdigest()))
 except (TypeError,OSError):pass
result=dict(tracker_MRO=identity,rows=rows,valid_rows=len(rows),changed_protocol_aux_rows=sum(r['protocol_aux_changes'] for r in rows),no_policy_act_calls=True,not_applied_or_physical_prediction=True,no_nav_claim=True,candidate_only_pursuit_target_input=True)
(H/'velocity_pursuit_closed_row_audit.json').write_text(json.dumps(result,indent=2)+'\n');print({k:v for k,v in result.items() if k not in ['rows','tracker_MRO']});print([x['class_name'] for x in identity])
