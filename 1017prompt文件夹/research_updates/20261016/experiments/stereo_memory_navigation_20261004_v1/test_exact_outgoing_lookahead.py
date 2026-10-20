import numpy as np,pytest
from exact_outgoing_lookahead import exact_direction_target,with_exact_outgoing_lookahead

def test_straight_exact_distance_and_no_mutation():
 p=np.array([[0.,0.],[.3,0],[.6,0],[.9,0]]);old=p.copy();t,d=exact_direction_target(p,['forward']*4,1,[.38,.1],.5);np.testing.assert_allclose(t,[.88,0]);np.testing.assert_equal(p,old);assert d['projected_arc_m']==pytest.approx(.38)
def test_corner_uses_arc_not_chord():
 p=np.array([[0.,0.],[.3,0],[.3,.3],[.3,.6]]);t,d=exact_direction_target(p,['forward']*4,1,[.2,.02],.35);np.testing.assert_allclose(t,[.3,.25]);assert d['actual_arc_lookahead_m']==pytest.approx(.35)
def test_reverse_and_cusp_uses_outgoing_edge():
 p=np.array([[0.,0.],[-.3,0],[-.6,0],[-.3,.3]]);t,d=exact_direction_target(p,['reverse','reverse','forward','forward'],1,[-.35,.02],.5);np.testing.assert_allclose(t,[-.6,0]);assert d['same_direction_end_index']==2
@pytest.mark.parametrize('xy,direction,index,pos,L',[(np.array([[0.,0.],[0.,0.]]),['forward']*2,0,[0,0],.5),(np.array([[0.,0.],[1.,0.]]),['forward']*2,0,[np.nan,0],.5),(np.array([[0.,0.],[1.,0.]]),['forward']*2,0,[0,0],float('nan'))])
def test_bad_geometry_rejected(xy,direction,index,pos,L):
 with pytest.raises(ValueError):exact_direction_target(xy,direction,index,pos,L)
class Base:
 def __init__(self):self.reset(None)
 def reset(self,c):self._path_index=1;self._active_drive_direction='reverse';self._tracking_audit={};self.xy=np.array([[0.,0.],[.3,0],[.6,0],[.9,0]]);self.ref=[type('W',(),{'drive_direction':'forward'})() for _ in range(4)];self.called=0
 def _require_reference(self):return self.ref,self.xy
 def _lookahead_target(self,L):self.called+=1;return np.array([9.,9.])
def test_event_delegates_before_reverse_then_accepted_forward_and_preserves_index():
 b=with_exact_outgoing_lookahead(Base)();b._active_drive_direction='forward';np.testing.assert_equal(b._lookahead_target(.5),[9,9]);b._active_drive_direction='reverse';np.testing.assert_equal(b._lookahead_target(.5),[9,9]);b._active_drive_direction='forward';b._tracking_audit=dict(position_world_x_m=.38,position_world_y_m=.1);np.testing.assert_allclose(b._lookahead_target(.5),[.88,0]);assert b._path_index==1 and b.called==2 and b._exact_lookahead_activated
