import numpy as np
import pytest
from terminal_known_goal_target import proposal

def arguments():
 return dict(position=np.array([0.,0.]),yaw=0.,goal=np.array([1.,0.]),reference=np.array([[0.,0.],[1.1,0.]]),directions=['forward','forward'],path_index=0,accepted_direction='forward',target=np.array([1.1,0.]),tracking_valid=True)

def test_current_terminal_goal_only_and_geometry():
 kw=arguments();r=proposal(**kw);assert r['changed'];np.testing.assert_array_equal(r['target'],kw['goal']);assert r['curvature']==0
 kw['goal']=np.array([1.,.1]);r=proposal(**kw);assert r['changed'];assert r['curvature']==pytest.approx(2*.1/1.01)

def test_reverse_sign_and_direction_cusp_never_consumed():
 kw=arguments();kw.update(goal=np.array([-1.,.1]),reference=np.array([[0.,0.],[-1.1,0.]]),target=np.array([-1.1,0.]),directions=['reverse','reverse'],accepted_direction='reverse');r=proposal(**kw);assert r['changed'];assert r['curvature']>0
 kw['directions']=['reverse','forward'];assert not proposal(**kw)['changed']

def test_early_return_middle_waypoint_and_goal_behind_unchanged():
 kw=arguments();kw['tracking_valid']=False;assert not proposal(**kw)['changed']
 kw=arguments();kw['target']=np.array([.5,0.]);assert not proposal(**kw)['changed']
 kw=arguments();kw['position']=np.array([2.,0.]);assert not proposal(**kw)['changed']

def test_outputs_do_not_alias_input_and_finite_required():
 kw=arguments();r=proposal(**kw);r['target'][0]=999;assert kw['goal'][0]==1
 kw['yaw']=float('nan')
 with pytest.raises(ValueError):proposal(**kw)
