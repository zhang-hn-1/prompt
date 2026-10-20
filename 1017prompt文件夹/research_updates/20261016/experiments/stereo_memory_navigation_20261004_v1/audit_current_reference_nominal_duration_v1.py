"""Nominal arithmetic only; never a transit bound or new planning cost trial."""
from pathlib import Path
import json,hashlib,datetime,numpy as np
H=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def main():
 design=H/'nominal_reverse_time_cost_design_v1.json';cfg=json.loads(design.read_text());fp=H.parent/'oracle_feasible_route_20260929_v9/hybrid/bank_mapping.json';rp=H/'reverse_straight_fitted_point_v1.json'
 for p in (fp,rp):assert sha(p)==cfg['input_sha256'][str(p)]
 speed=dict(forward=json.loads(fp.read_text())['directions']['forward']['low']['speed_m_s'],reverse=json.loads(rp.read_text())['measured']['actual_speed_m_s'])
 epochs=json.loads((H/'current_reference_epoch_geometry_assessment_v1.json').read_text());out={};inputs={str(p):sha(p) for p in (fp,rp,design,H/'current_reference_epoch_geometry_assessment_v1.json')}
 for name in ('b0','candidate'):
  rf=H/f'transverse_velocity_feedback_e1_{name}_run_v1/policy_records.json';rows=json.loads(rf.read_text());out[name]=[];inputs[str(rf)]=sha(rf)
  for epoch in epochs['epochs'][name]:
   f=epoch['first_frame'];row=rows[f];w=row['current_reference_waypoints_after_act']
   if len(w)<2:continue
   lengths={d:sum(.3 for x in w[:-1] if x['drive_direction']==d) for d in ('forward','reverse')};times={d:lengths[d]/speed[d] for d in lengths};nominal=sum(times.values());actual_direction_changes=sum(w[i]['drive_direction']!=w[i-1]['drive_direction'] for i in range(1,len(w)-1))
   out[name].append(dict(reference_epoch_sha256=epoch['reference_sha256'],first_frame=f,current_time_s=row['time_s'],remaining_original_budget_s=35-row['time_s'],nominal_supported_primitive_arc_m=lengths,nominal_point_speed_m_s=speed,nominal_arithmetic_duration_s=times,total_nominal_arithmetic_duration_s=nominal,nominal_exceeds_remaining_budget=nominal>35-row['time_s'],raw_outgoing_direction_changes=actual_direction_changes,first_outgoing_direction=w[0]['drive_direction'],runtime_reverse_cap_rad_s=.3,nominal_reverse_carrier_main_rad_s=-.5,stopping_times_included=False,physical_lower_or_upper_bound=False))
 result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),epochs=out,input_sha256=inputs,source_sha256={str(Path(__file__).resolve()):sha(Path(__file__))},notes=['Nonterminal waypoint action carries outgoing primitive; each nominal primitive arc is the frozen .3m, not chord length.','Forward speed is an old low-main aggregate, whereas runtime has current forward boost. Reverse carrier -.5 uses the covered executed -.3 point because runtime caps reverse.','These point-speed arithmetic values exclude stopping and transient lateral/heading response. They are not certified arrival time, feasibility bounds or evidence that35s is physically impossible.','The earlier nominal reverse cost trial remains rejected and is not rerun or rescued. This diagnostic does not grant a new query, control, gain or collection budget.'],no_new_solver_physics_fit=True,no_new_gate=True,whole_mission_complete=False)
 p=H/'current_reference_nominal_duration_assessment_v1.json';assert not p.exists();p.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(out))
if __name__=='__main__':main()
