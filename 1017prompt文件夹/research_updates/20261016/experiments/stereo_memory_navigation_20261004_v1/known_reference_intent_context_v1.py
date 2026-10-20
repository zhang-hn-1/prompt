"""Known installed reference intent; does not invent future controller requests."""
import numpy as np

def known_reference_intent(reference,current_xy,path_index,accepted_direction):
 if accepted_direction not in ('forward','reverse') or isinstance(path_index,bool) or not isinstance(path_index,(int,np.integer)):raise ValueError('known accepted direction/current index')
 xy=np.asarray(current_xy,float);q=np.array([[w['x_m'],w['y_m']] for w in reference],float)
 if xy.shape!=(2,) or q.ndim!=2 or q.shape[1]!=2 or not len(q) or not np.isfinite(q).all() or not np.isfinite(xy).all() or not 0<=path_index<len(q):raise ValueError('finite known reference/current XY')
 if any(w['drive_direction'] not in ('forward','reverse') for w in reference):raise ValueError('outgoing direction required')
 idx=int(path_index);end=idx
 while end<len(q)-1 and reference[end]['drive_direction']==accepted_direction:end+=1
 if end>idx:
  v=q[idx+1]-q[idx];den=float(v@v);u=0. if den==0 else float(np.clip((xy-q[idx])@v/den,0.,1.));project=q[idx]+u*v;prefix=np.vstack([project,q[idx+1:end+1]])
 else:project=q[idx];prefix=q[idx:idx+1]
 nominal=[dict(main_carrier_rad_s=float(w['main_command_rad_s']),auxiliary_nominal_carrier_rad=float(w['auxiliary_angle_rad'])) for w in reference[idx:end]]
 if any(not np.isfinite(list(w.values())).all() for w in nominal):raise ValueError('finite nominal carrier commands')
 return dict(accepted_direction=accepted_direction,current_outgoing_direction=reference[idx]['drive_direction'] if idx<len(q)-1 else None,known_geometry_prefix_xy=prefix.tolist(),same_direction_nominal_remaining_m=float(np.linalg.norm(np.diff(prefix,axis=0),axis=1).sum()),projection_error_m=float(np.linalg.norm(xy-project)),prefix_end='reference_terminal' if end==len(q)-1 else 'unaccepted_direction_boundary',next_direction=None if end==len(q)-1 else reference[end]['drive_direction'],nominal_carriers_not_actual_requests=nominal,future_actual_request_plan=None,physical_prediction_available=False,low_speed_confirmation_not_predicted=True,reference_intent_not_motion_or_safety_certificate=True)
