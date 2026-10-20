import copy,numpy as np
from known_reference_intent_context_v1 import known_reference_intent
REF=[{'x_m':0.,'y_m':0.,'drive_direction':'forward','main_command_rad_s':.5,'auxiliary_angle_rad':0.},{'x_m':.3,'y_m':0.,'drive_direction':'forward','main_command_rad_s':.5,'auxiliary_angle_rad':.1},{'x_m':.6,'y_m':0.,'drive_direction':'reverse','main_command_rad_s':-.5,'auxiliary_angle_rad':0.},{'x_m':.3,'y_m':0.,'drive_direction':'reverse','main_command_rad_s':0.,'auxiliary_angle_rad':0.}]
def test_cusp_ends_known_direction_prefix_without_inventing_request():
 r=known_reference_intent(REF,np.array([.2,0]),0,'forward');assert abs(r['same_direction_nominal_remaining_m']-.4)<1e-10 and r['next_direction']=='reverse';assert len(r['known_geometry_prefix_xy'])==3;assert r['future_actual_request_plan'] is None and not r['physical_prediction_available']
def test_terminal_has_no_outgoing_direction():
 r=known_reference_intent(REF,[.4,0],2,'reverse');assert r['next_direction'] is None and r['prefix_end']=='reference_terminal'
def test_inputs_unmodified_and_no_future_action_argument():
 old=copy.deepcopy(REF);a=known_reference_intent(REF,[.2,0],0,'forward');b=known_reference_intent(copy.deepcopy(REF),[.2,0],0,'forward');assert a==b and old==REF
