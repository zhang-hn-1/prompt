"""Literal projection, physical ambiguity and data isolation fixtures."""
import numpy as np
import multiview_rgb as mv

POSE=np.diag([1.,-1.,-1.,1.]) # saved USD so optical world axes are identity

def frame(image,pose=POSE,eye='left',time=0.,number=0):
    return dict(rgb=image,pose=pose,mask=np.zeros(image.shape[:2],bool),eye=eye,time_s=time,frame=number)


def test_plane_projection_has_correct_translation_sign_and_metric_parallax():
    source=POSE.copy();source[0,3]=.2
    x,y,angle,positive=mv.project_plane(POSE,source,2.,5,5,10.,2.,2.)
    np.testing.assert_allclose([x[2,2],y[2,2]],[1.,2.])
    np.testing.assert_allclose(angle[2,2],5.710593137499643)
    assert positive[2,2]


def test_pure_camera_rotation_has_zero_metric_parallax():
    source=POSE.copy();source[:3,:3]=np.array([[0,-1,0],[1,0,0],[0,0,1]])@source[:3,:3]
    _,_,angle,_=mv.project_plane(POSE,source,2.,5,5,10.,2.,2.)
    np.testing.assert_allclose(angle,0.,atol=1e-12)


def texture_pair():
    grey=np.random.default_rng(37).integers(0,256,(65,65),dtype=np.uint8)
    rgb=np.repeat(grey[:,:,None],3,axis=2);source=np.zeros_like(rgb);source[:,:-10]=rgb[:,10:]
    pose=POSE.copy();pose[0,3]=.2
    return frame(rgb),frame(source,pose,'right')


def options():
    return dict(depths=np.array([1.8,1.9,2.,2.1,2.2]),focal=100.,cx=32.,cy=32.,patch_radius=2,
                texture_min=1.,parallax_min=.5,min_sources=1,max_cost=.2,slack=.05,max_interval=.1)


def test_textured_translated_pair_recovers_hand_constructed_two_metre_plane():
    reference,source=texture_pair();out=mv.predict_rgb(reference,[source],**options())
    np.testing.assert_allclose(out['raw_depth_m'][32,32],2.)
    assert out['confidence'][32,32] and out['view_bits'][32,32]==1
    assert out['cost'][32,32]<1e-5


def test_any_projected_patch_body_mask_rejects_supported_match():
    reference,source=texture_pair();source['mask'][32,22]=True
    out=mv.predict_rgb(reference,[source],**options())
    assert not out['confidence'][32,32] and not np.isfinite(out['depth_m'][32,32])


def test_constant_and_repeating_along_baseline_textures_remain_unknown():
    constant=np.full((65,65,3),128,np.uint8);pose=POSE.copy();pose[0,3]=.2
    out=mv.predict_rgb(frame(constant),[frame(constant,pose,'right')],**options())
    assert not out['confidence'].any()
    grey=np.tile((np.arange(65)*31%251)[:,None],(1,65)).astype(np.uint8);stripes=np.repeat(grey[:,:,None],3,2)
    out=mv.predict_rgb(frame(stripes),[frame(stripes,pose,'right')],**options())
    assert not out['confidence'][32,32] and out['interval_ray_width_m'][32,32]>.1


def test_actual_predictor_ignores_future_reference_payload_and_forbidden_truth_fields():
    reference,source=texture_pair()
    class Guard(dict):
        def __getitem__(self,key):
            if key!='time_s':raise AssertionError('read excluded payload '+key)
            return super().__getitem__(key)
    future=Guard(time_s=1.);same=frame(None) if False else dict(time_s=0.,frame=0,eye='left',rgb='poison reference')
    reference['renderer_depth']='poison truth';source['source_world_points']='poison truth'
    clean=mv.predict_rgb(reference,[source],**options())
    poisoned=mv.predict_rgb(reference,[source,future,same],**options())
    for k in clean:np.testing.assert_equal(clean[k],poisoned[k])
