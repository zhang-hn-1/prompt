from types import SimpleNamespace as S
import numpy as np
from cusp_coast_acquisition_v1 import decide

def geometry():
 return [S(drive_direction=d) for d in ['forward','forward','reverse','forward']],np.array([[0.,0.],[.3,0.],[.6,0.],[.3,0.]])

def call(xy=(.3,0.),**changes):
 r,p=geometry();kw=dict(reference=r,xy=p,current_index=1,position=np.array(xy),active='forward',armed=True,speed=.18,main_joint_velocity=.44,bank_rad=.1);kw.update(changes);return decide(**kw)

def test_incoming_cusp_acquired_without_changing_reference():
 d=call();assert d['acquire_index']==2 and d['remaining_arc_m']==.3

def test_farther_incoming_edge_and_reverse_and_unarmed_do_not_acquire():
 assert call((.2,0.),current_index=0)['acquire_index'] is None
 assert call(active='reverse')['acquire_index'] is None
 assert call(armed=False)['acquire_index'] is None

def test_current_state_marginal_domain_and_cross_track_required():
 assert call(speed=.3)['acquire_index'] is None
 assert call(main_joint_velocity=.5)['acquire_index'] is None
 assert call(bank_rad=.5)['acquire_index'] is None
 assert call((.3,.21))['acquire_index'] is None

def test_current_opposite_waypoint_preserves_original_handler_ownership():
 assert call(current_index=2)['acquire_index'] is None

def test_forward_only_reference_has_no_cusp_and_cannot_change_index():
 r,p=geometry();r=[S(drive_direction='forward') for _ in r];assert call(reference=r)['acquire_index'] is None
