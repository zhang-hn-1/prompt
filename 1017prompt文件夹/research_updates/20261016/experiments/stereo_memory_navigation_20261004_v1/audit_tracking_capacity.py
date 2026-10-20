"""One frozen readonly actual-data measurement; never controller/model prediction."""
from pathlib import Path
import json,hashlib,datetime,sys
import numpy as np
from scipy.ndimage import distance_transform_edt
from tracking_measurement import measure_turn
from reference_continuity_audit import reference_geometry_status
from route_geometry_audit import dense_prefix
H=Path(__file__).resolve().parent;O=H/'endpoint_navigation_run';P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm');sys.path.insert(0,str(P));sha=lambda f:hashlib.sha256(Path(f).read_bytes()).hexdigest()
def main():
 p=json.loads((H/'tracking_capacity_protocol.json').read_text());assert not (H/'tracking_capacity_assessment.json').exists()
 for mapping in [p['source_sha256'],p['input_sha256']]:
  for f,d in mapping.items():assert sha(f)==d,f
 started=datetime.datetime.now(datetime.timezone.utc).isoformat()
 from outgoing_lookahead import make_endpoint_oracle
 from training.action_protocol import load_action_protocol
 oracle=make_endpoint_oracle(load_action_protocol(P/'configs/data_contract.yaml'))
 from bank_mapping import BankMap
 bank_payload=json.loads((H.parent/'oracle_feasible_route_20260929_v9/hybrid/bank_mapping.json').read_text());bank=BankMap(bank_payload)
 records=json.loads((O/'policy_records.json').read_text());ph=np.load(O/'physics.npz');a=np.load(O/'episode_arrays.npz');fb=json.loads((O/'feedback/bank_feedback_golden_e1_0001.json').read_text());xy=a['positions_world_xy'];yaw=a['yaw_world_rad'];rows=[]
 for i in range(237,335):
  r=records[i];d=r['tracking'];assert d['valid'] and r['request']['request_source']=='tracking' and r['local']['local_reference_version']==3
  ref=np.load(O/'local_plans'/f'frame_{i:03d}.npz');j=d['projection_index'];assert d['reference_body_heading_rad']==float(ref['pose'][j,2]) and d['root_heading_rad']==float(yaw[i]);assert ref['direction'][j]==1 and r['diagnostics']['active_drive_direction']=='forward'
  command=ref['command'][j];nominal=float(oracle.drive_model.curvature(*command));k=d['requested_curvature_1_m'];speed=d['world_xy_speed_m_s'];choice=bank.query(d['policy_raw_main_command_rad_s'],speed,k);saved=d['calibrated_bank_mapping']
  for key in choice:assert choice[key]==saved[key],(i,key)
  levels=bank_payload['directions']['forward'];weights=[1-choice['speed_weight'],choice['speed_weight']];support=[sum(w*op(levels[level]['curvature_body_1_m']) for w,level in zip(weights,['low','high'])) for op in [min,max]]
  ticks=fb[5*i:5*(i+1)];sl=slice(5*i+1,5*(i+1)+1)
  rows.append(dict(frame=i,time_s=i/10.,reference_index=j,nominal_command=command.tolist(),nominal_body_curvature_1_m=nominal,actual_requested_body_curvature_1_m=k,current_bank_weighted_endpoint_support_1_m=support,production_map_curvature_clamped=choice['curvature_clamped'],speed_clamped=choice['speed_clamped'],actual_speed_m_s=speed,current_signed_body_heading_error_rad=d['signed_body_heading_error_rad'],current_projection_distance_m=d['projection_distance_m'],desired_bank_mean_rad=float(np.mean([x['desired_bank_rad'] for x in ticks])),actual_bank_mean_rad=float(np.mean([x['bank_rad'] for x in ticks])),bank_error_rms_rad=float(np.sqrt(np.mean([x['error_rad']**2 for x in ticks]))),pid_clipped_ticks=sum(x['pid_clipped'] for x in ticks),supervisor_active_ticks=sum(x['supervisor_active'] for x in ticks),actual_world_z_angular_velocity_mean_rad_s=float(ph['angular_velocity_world_rad_s'][sl,2].mean()),applied_main_mean_rad_s=float(ph['applied_action'][sl,0].mean()),actual_main_joint_mean_rad_s=float(ph['joint_velocity_rad_s'][sl,0].mean())))
 windows=[]
 for start in range(237,335,10):
  end=min(start+10,335);g=measure_turn(xy[start:end+1],yaw[start:end+1]);ticks=fb[5*start:5*end];dt=np.diff(ph['simulation_time_s'][5*start:5*end+1]);wz=float(np.dot(ph['angular_velocity_world_rad_s'][5*start+1:5*end+1,2],dt));rr=rows[start-237:end-237]
  windows.append(dict(start_frame=start,end_frame=end,duration_s=(end-start)/10.,**g,world_z_integral_rad=wz,world_z_integral_per_xy_arc_1_m=wz/g['actual_xy_arc_m'] if g['geometric_available'] else None,nominal_body_curvature_mean_1_m=float(np.mean([x['nominal_body_curvature_1_m'] for x in rr])),requested_body_curvature_median_1_m=float(np.median([x['actual_requested_body_curvature_1_m'] for x in rr])),production_map_clamped_blocks=sum(x['production_map_curvature_clamped'] for x in rr),actual_speed_mean_m_s=float(np.mean([x['actual_speed_m_s'] for x in rr])),desired_bank_mean_rad=float(np.mean([x['desired_bank_rad'] for x in ticks])),actual_bank_mean_rad=float(np.mean([x['bank_rad'] for x in ticks])),bank_error_rms_rad=float(np.sqrt(np.mean([x['error_rad']**2 for x in ticks]))),pid_clipped_fraction=float(np.mean([x['pid_clipped'] for x in ticks]))))
 oldref=np.load(O/'local_plans/frame_334.npz')['pose'][:,:2];guards=[]
 for posframe in [330,335]:
  for mapframe in [330,335]:
   grid=np.load(O/'maps'/f'frame_{mapframe:03d}.npz');guard=reference_geometry_status(xy[posframe],oldref,grid['labels'],grid['origin'],float(grid['resolution']));guards.append(dict(position_frame=posframe,map_frame=mapframe,interpretation_clock=335,all_evidence_at_or_before_clock=True,guard=guard))
   if posframe==mapframe:assert guard==records[posframe]['local']['reference_decision']['guard'],posframe
 grid=np.load(O/'maps/frame_335.npz');guard=guards[-1]['guard'];labels=grid['labels'];origin=grid['origin'];res=float(grid['resolution']);field,indices=distance_transform_edt(np.pad(labels!=1,1),return_indices=True);field=field[1:-1,1:-1]*res;witnesses=[]
 for kind,line in [('suffix',guard['suffix_xy']),('bridge',guard['bridge_xy'])]:
  line=np.asarray(line);samples=dense_prefix(line,length_m=float(np.linalg.norm(np.diff(line,axis=0),axis=1).sum()),spacing_m=.05);cells=np.rint((samples-origin)/res).astype(int)
  for n,(sample,cell) in enumerate(zip(samples,cells)):
   if field[tuple(cell)]>=.6-1e-8:continue
   nearest=indices[:,cell[0]+1,cell[1]+1]-1;assert labels[tuple(nearest)]==1
   stamp=float(grid['hazard_time'][tuple(nearest)]);assert np.isfinite(stamp) and stamp<=float(grid['time_s']);sourceframe=int(np.argmin(abs(a['simulation_time_s']-stamp)));assert sourceframe<=335
   packet=np.load(O/'sensor_packets'/f'frame_{sourceframe:03d}.npz');assert abs(float(packet['time_s'])-stamp)<1e-6
   points=packet['points_world'];root=packet['root_xyz'];mask=(np.linalg.norm(points-root,axis=1)>=.55)&(points[:,2]>.05)&(points[:,2]<=1.5);pc=np.rint((points[:,:2]-origin)/res).astype(int);contributing=points[mask&(pc==nearest).all(1)];assert len(contributing)>0
   witnesses.append(dict(kind=kind,sample_index=n,sample_world_xy=sample.tolist(),sample_rounded_cell=cell.tolist(),rounded_world_xy=(origin+cell*res).tolist(),rounding_displacement_m=float(np.linalg.norm(sample-(origin+cell*res))),raster_clearance_m=float(field[tuple(cell)]),hazard_cell=nearest.tolist(),hazard_world_xy=(origin+nearest*res).tolist(),hazard_provenance_code=int(grid['provenance'][tuple(nearest)]),hazard_time_s=stamp,source_frame=sourceframe,contributing_point_count=len(contributing),contributing_height_min_m=float(contributing[:,2].min()),contributing_height_max_m=float(contributing[:,2].max()),source_packet_sha256=sha(O/'sensor_packets'/f'frame_{sourceframe:03d}.npz')))
 assert witnesses
 result=dict(started_utc=started,completed_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),rows=rows,windows=windows,valid_tracking_denominator=len(rows),geometric_window_denominator=len(windows),geometric_available_windows=sum(w['geometric_available'] for w in windows),production_map_clamped_blocks=sum(x['production_map_curvature_clamped'] for x in rows),protocol_clipped_blocks=sum(records[i]['tracking']['protocol_clipped'] for i in range(237,335)),guard_counterfactuals=guards,raster_witnesses=witnesses,bank_endpoint_support_not_dynamic_model=True,offline_geometry_uses_measured_window_successors_not_prediction=True,policy_act_calls=0,planner_searches=0,new_physical_sequences=0,new_model_fits=0,control_candidate=None,old_navigation_gate_unchanged_rejected=True,whole_research_complete=False,automation='ACTIVE')
 (H/'tracking_capacity_assessment.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k not in ['rows','windows','guard_counterfactuals','raster_witnesses']},indent=2));print('WINDOWS',json.dumps(windows));print('GUARDS',[(x['position_frame'],x['map_frame'],x['guard']['reject_reasons']) for x in guards]);print('WITNESSES',json.dumps(witnesses))
if __name__=='__main__':main()
