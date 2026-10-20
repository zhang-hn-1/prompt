"""ONE archived current-state algebraic audit. No act/search/physics/prediction."""
from pathlib import Path
import sys,json,hashlib,math,datetime
import numpy as np
H=Path(__file__).resolve().parent
P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm');sys.path.insert(0,str(P))
from outgoing_lookahead import make_endpoint_oracle
from cusp_projection import make_cusp_oracle
from oracle_rgb_shadow import reference_from_arrays
from depth_local_curve import install_local_reference
from training.action_protocol import load_action_protocol
from benchmark.policies import PolicyObservation
from roll_pid import bank_from_quaternion
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save(n,d):(H/n).write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n')
def geometry(t,ref,row,corrected):
 idx=row['diagnostics']['reference_index'];r=row['tracking'];t._path_index=idx;t._tracking_audit={}
 xy=np.array([r['position_world_x_m'],r['position_world_y_m']]);yaw=r['root_heading_rad']
 target=t._lookahead_target(r['lookahead_requested_m']);d=target-xy;c,s=math.cos(yaw),math.sin(yaw);b=np.array([c*d[0]+s*d[1],-s*d[0]+c*d[1]])
 k=0. if b@b<=1e-9 else 2*b[1]/(b@b)
 direction='forward' if ref['direction'][idx]>0 else 'reverse';sign=1 if direction=='forward' else -1
 planned=ref['command'][idx,0]
 if planned==0:
  for j in range(idx-1,-1,-1):
   if ref['direction'][j]!=ref['direction'][idx]:break
   if ref['command'][j,0]!=0:planned=ref['command'][j,0];break
 magnitude=min(abs(planned),.5) if abs(k)>=.55 else min(abs(planned),1.) if abs(k)>=.25 else abs(planned)
 # Actual inherited hook is stateless and only consumes current scalar inputs.
 speed=r['world_xy_speed_m_s'];proprio=np.zeros((2,17),np.float32)
 obs=PolicyObservation(np.zeros((480,640,3),np.uint8),np.zeros((480,640,3),np.uint8),proprio,np.zeros((2,10),np.float32),xy,yaw)
 magnitude=t._schedule_main_magnitude(obs,baseline_magnitude=magnitude,planned_main_command_rad_s=planned,desired_curvature_1_m=k,cross_track_error_m=row['diagnostics']['cross_track_error_m']);main=sign*magnitude
 model=t.drive_model;aux=model.auxiliary_for_curvature(main,k)
 v1forward=model.curvature(main,aux);support=[z.curvature_1_m for z in model.steering_samples if abs(z.main_command_rad_s-main)<1e-12]
 assert support and abs(v1forward-np.clip(k,min(support),max(support)))<1e-10
 current_input=np.load(H/'cusp_navigation_run/policy_inputs'/f"frame_{row['frame']:03d}.npz");act_speed=float(np.linalg.norm(current_input['proprio_history'][-1,:2]));yaw_request=k*sign*max(act_speed,.2);correction=float(np.clip(-sign*t.yaw_rate_feedback_gain_s*(yaw_request-r['measured_world_yaw_rate_rad_s']),math.radians(-5),math.radians(5)))
 bankmap=H.parent/'oracle_feasible_route_20260929_v9/hybrid';sys.path.insert(0,str(bankmap));from bank_mapping import BankMap
 choice=BankMap(json.loads((bankmap/'bank_mapping.json').read_text())).query(main,speed,k)
 raw=float(np.float32(choice['encoded_aux_rad']+correction));protocol=t.protocol.clip_block(np.tile(np.array([main,raw],np.float32),5)).reshape(-1,2)[0]
 return dict(lookahead_index=t._tracking_audit['lookahead_index'],accumulated_m=t._tracking_audit['lookahead_accumulated_m'],target_world_xy=target.tolist(),target_body_xy=b.tolist(),target_distance_m=float(np.linalg.norm(b)),curvature_body_1_m=float(k),curvature_ordered_space_1_m=float(sign*k),main_request=float(np.float32(main)),v1_inverse_aux_rad=aux,v1_forward_at_inverse_1_m=v1forward,desired_world_z_proxy_rad_s=yaw_request,yaw_feedback_aux_rad=correction,bank_mapping=choice,raw_aux_rad=raw,protocol_aux_rad=float(protocol[1]),actual_float32_policy_speed_m_s=act_speed,diagnostic_float64_mapping_speed_m_s=speed,current_only=True,physical_prediction=False)
def main():
 seal=json.loads((H/'outgoing_lookahead_protocol_v2.json').read_text());assert not (H/'outgoing_lookahead_assessment.json').exists()
 start=datetime.datetime.now(datetime.timezone.utc).isoformat();assert seal['sealed_utc']<start
 for mapping in [seal['source_sha256'],seal['input_sha256']]:
  for f,d in mapping.items():assert sha(f)==d,f
 O=H/'cusp_navigation_run';records=json.loads((O/'policy_records.json').read_text());protocol=load_action_protocol(P/'configs/data_contract.yaml');old=make_cusp_oracle(protocol);new=make_endpoint_oracle(protocol);rows=[];invalid=0
 for r in records:
  if not r['tracking'].get('valid') or r['request']['request_source']!='tracking':invalid+=1;continue
  i=r['frame'];ref=np.load(O/'local_plans'/f'frame_{i:03d}.npz');plan=reference_from_arrays(ref['pose'],ref['direction'],ref['command'],float(r['local']['plan'].get('local_path_length_m',3)),1)
  for t in [old,new]:install_local_reference(t,plan)
  a=geometry(old,ref,r,False);b=geometry(new,ref,r,True);tr=r['tracking'];mp=r['mapping']
  assert a['lookahead_index']==tr['lookahead_index'];np.testing.assert_allclose(a['target_world_xy'],[tr['lookahead_x_m'],tr['lookahead_y_m']],rtol=0,atol=1e-12)
  for k,v in [('curvature_body_1_m',tr['requested_curvature_1_m']),('v1_inverse_aux_rad',tr['v1_inverse_auxiliary_rad']),('desired_world_z_proxy_rad_s',tr['desired_world_yaw_rate_rad_s']),('yaw_feedback_aux_rad',tr['yaw_rate_feedback_auxiliary_rad']),('raw_aux_rad',r['request']['policy_raw_auxiliary_target_rad']),('protocol_aux_rad',r['request']['policy_protocol_auxiliary_target_rad'])]:assert abs(a[k]-v)<1e-10,(i,k,a[k],v)
  assert a['main_request']==r['request']['policy_raw_main_command_rad_s']
  for key in a['bank_mapping']:assert a['bank_mapping'][key]==mp[key],(i,key)
  idx=r['diagnostics']['reference_index'];version=int(ref['version']);bodyk=old.drive_model.curvature(float(ref['command'][idx,0]),float(ref['command'][idx,1])) if ref['command'][idx,0] else 0
  rows.append(dict(frame=i,time_s=i/10,reference_version=version,reference_index=idx,direction=tr['reference_drive_direction'],outgoing_reference_body_curvature_1_m=bodyk,outgoing_nominal_yaw_delta_rad=None if idx+1==len(ref['pose']) else float((ref['pose'][idx+1,2]-ref['pose'][idx,2]+math.pi)%(2*math.pi)-math.pi),actual_body_heading_error_rad=tr['signed_body_heading_error_rad'],actual_root_yaw_rad=tr['root_heading_rad'],actual_world_z_angular_rad_s=tr['measured_world_yaw_rate_rad_s'],actual_course_rad=tr['velocity_course_rad'] if tr['velocity_course_valid'] else None,old=a,candidate=b,raw_request_changes=bool(a['raw_aux_rad']!=b['raw_aux_rad'] or a['main_request']!=b['main_request']),same_reference_and_current_state=True,candidate_successor_truth_used=False))
 assert len(rows)==188 and invalid==162
 ph=np.load(O/'physics.npz');fb=json.loads((O/'feedback/bank_feedback_golden_e1_0001.json').read_text());assert len(fb)==1750;tickrows=[]
 for tick,r in enumerate(fb):
  bank=bank_from_quaternion(ph['root_quaternion_world_wxyz'][tick]);assert abs(bank-r['bank_rad'])<1e-12
  np.testing.assert_equal(ph['smoothed_target'][tick+1,1],np.float32(r['smoothed_aux_rad']));np.testing.assert_equal(ph['applied_action'][tick+1,1],np.float32(r['applied_aux_rad']))
  target=.75*float(np.clip(ph['clipped_target'][tick+1,1],-math.pi/6,math.pi/6));assert abs(target-r['desired_bank_rad'])<1e-12
  tickrows.append(dict(tick=tick,time_s=tick*.02,actual_bank_rad=bank,bank_target_rad=target,actual_bank_error_rad=target-bank,supervisor_active=r['supervisor_active'],pid_clipped=r['pid_clipped'],actual_aux_joint_rad=float(ph['joint_position_rad'][tick,1]),actual_aux_joint_velocity_rad_s=float(ph['joint_velocity_rad_s'][tick,1]),actual_smoothed_aux_rad=r['smoothed_aux_rad'],actual_applied_aux_rad=r['applied_aux_rad']))
 groups=[]
 for version,lo,hi,direction in [(3,197,216,'reverse'),(3,237,250,'forward'),(6,297,314,'reverse'),(6,333,350,'forward')]:
  selected=[x for x in rows if lo<=x['frame']<hi];assert len(selected)==hi-lo
  ticks=tickrows[lo*5:hi*5];yaw0=float(ph['yaw_world_rad'][5*lo]);yaw1=float(ph['yaw_world_rad'][5*hi]);bankerr=[x['actual_bank_error_rad'] for x in ticks];dyaw=float((yaw1-yaw0+math.pi)%(2*math.pi)-math.pi)
  def summary(key):
   q=[x[key] for x in selected];return dict(pp_min=min(x['curvature_body_1_m'] for x in q),pp_max=max(x['curvature_body_1_m'] for x in q),abs_pp_p50=float(np.median([abs(x['curvature_body_1_m']) for x in q])),zero_lookahead_rows=sum(x['accumulated_m']==0 for x in q),clipped_aux_rows=sum(abs(x['raw_aux_rad']-x['protocol_aux_rad'])>1e-7 for x in q),bankmap_clamped_rows=sum(x['bank_mapping']['curvature_clamped'] for x in q))
  groups.append(dict(version=version,start_frame=lo,end_frame=hi,direction=direction,valid_tracking_rows=len(selected),old=summary('old'),candidate_current_algebra_only=summary('candidate'),actual_yaw_delta_rad=dyaw,actual_world_z_integral_rad=float(ph['angular_velocity_world_rad_s'][5*lo:5*hi,2].sum()*.02),actual_bank_error_rms_rad=float(np.sqrt(np.mean(np.square(bankerr)))),actual_bank_error_peak_rad=float(np.max(np.abs(bankerr))),actual_supervisor_fraction=sum(x['supervisor_active'] for x in ticks)/len(ticks),actual_pid_clipped_fraction=sum(x['pid_clipped'] for x in ticks)/len(ticks),actual_main_applied_min=float(ph['applied_action'][5*lo+1:5*hi+1,0].min()),actual_main_applied_max=float(ph['applied_action'][5*lo+1:5*hi+1,0].max())))
 save('outgoing_lookahead_rows.json',rows);save('outgoing_lookahead_actual_feedback.json',tickrows)
 result=dict(started_utc=start,assessed_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),source_checks=len(seal['source_sha256']),input_checks=len(seal['input_sha256']),archived_records=len(records),valid_tracking_rows=len(rows),invalid_or_nontracking_rows=invalid,original_geometry_inverse_mapping_request_rows_exact=len(rows),actual_feedback_rows_exact=len(tickrows),actual_old_navigation_gate_unchanged_rejected=True,new_physical_runs=0,new_planner_searches=0,archived_policy_act_calls=0,changed_current_algebraic_requests=sum(x['raw_request_changes'] for x in rows),boundary_zero_lookahead_rows=sum(x['old']['accumulated_m']==0 and x['candidate']['accumulated_m']>0 for x in rows),groups=groups,selected_current_rows=[x for x in rows if x['frame'] in [197,215,237,249,297,313,333,349]],interface_regression_pass=True,actual_navigation_candidate_tested=False,candidate_physical_prediction=False,whole_research_complete=False)
 save('outgoing_lookahead_assessment.json',result);print(json.dumps({k:v for k,v in result.items() if k!='selected_current_rows'},indent=2))
if __name__=='__main__':main()
