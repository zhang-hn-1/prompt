"""Hand-constructed physical correspondence, ambiguity, masks, median and causality."""
import numpy as np
from multiview_cost_trace import trace_rgb
from test_multiview_rgb import texture_pair,options,frame,POSE

def trace(ref,src):return trace_rgb(ref,src,np.array([[32,32]]),**options())

def test_literal_plane_reports_correct_winner_and_source_parallax():
    r,s=texture_pair();o=trace(r,[s])
    assert o['best_index'][0]==2 and o['confidence'][0]
    np.testing.assert_allclose(o['raw_depth_m'][0],2)
    np.testing.assert_allclose(o['source_parallax_deg'][0,2,0],5.710593137499643)
    assert o['source_valid'][0,2,0] and o['cost'][0,2]<1e-5


def test_any_source_patch_mask_is_visible_in_trace_rejection():
    r,s=texture_pair();s['mask'][32,22]=True;o=trace(r,[s])
    assert not o['source_footprint_valid'].any() and not o['confidence'][0]
    assert not np.isfinite(o['cost']).any()


def test_rotation_only_source_has_no_finite_metric_scores():
    r,s=texture_pair();s['pose']=POSE.copy();o=trace(r,[s])
    np.testing.assert_allclose(o['source_parallax_deg'],0,atol=1e-12)
    assert not o['source_valid'].any() and not np.isfinite(o['cost']).any()


def test_along_baseline_repeating_pattern_exposes_all_competitive_depths():
    r,s=texture_pair();grey=np.tile((np.arange(65)*31%251)[:,None],(1,65)).astype(np.uint8)
    r['rgb']=np.repeat(grey[:,:,None],3,2);s['rgb']=r['rgb'].copy();o=trace(r,[s])
    np.testing.assert_allclose(o['cost'],0,atol=1e-6)
    assert o['competitive'].all() and o['interval_ray_width_m'][0]>.1 and not o['confidence'][0]


def test_two_opposite_correlations_use_even_median_without_best_view_cherry_picking():
    r,s=texture_pair();second=dict(s);second['rgb']=255-s['rgb'];second['eye']='left';second['time_s']=-1;second['frame']=-1
    o=trace(r,[s,second]);np.testing.assert_allclose(o['source_cost'][0,2],[0,2],atol=1e-6)
    np.testing.assert_allclose(o['cost'][0,2],1,atol=1e-6);assert o['view_bits'][0,2]==3


def test_future_metadata_guard_and_forbidden_truth_cannot_change_any_trace_field():
    r,s=texture_pair()
    class Future(dict):
        def __getitem__(self,k):
            if k!='time_s':raise AssertionError('future payload read '+k)
            return super().__getitem__(k)
    r['renderer_depth']='poison';s['source_world_points']='poison'
    a=trace(r,[s]);b=trace(r,[s,Future(time_s=1),dict(time_s=0,eye='left',rgb='poison')])
    for k in a:np.testing.assert_equal(a[k],b[k])
