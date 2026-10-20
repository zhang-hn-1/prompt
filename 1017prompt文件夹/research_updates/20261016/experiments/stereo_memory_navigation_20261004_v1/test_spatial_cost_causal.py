import numpy as np
import spatial_cost as sc
from test_multiview_rgb import texture_pair,options,frame,POSE

def test_real_textured_and_masked_inputs_preserve_evidence_boundary():
    ref,src=texture_pair();o,_,_,_=sc.predict_rgb(ref,[src],**options())
    assert o['confidence'][32,32] and o['depth_m'][32,32]==2.
    src['mask'][32,22]=True;o,_,_,_=sc.predict_rgb(ref,[src],**options())
    assert not o['confidence'][32,32]

def test_future_payload_truncation_and_forbidden_truth_poison_exactly_equal():
    ref,src=texture_pair()
    class Future(dict):
        def __getitem__(self,key):
            if key!='time_s':raise AssertionError('future payload read')
            return super().__getitem__(key)
    clean,_,_,_=sc.predict_rgb(ref,[src],**options())
    ref['renderer_depth']='poison';src['future_state']='poison'
    poisoned,_,_,_=sc.predict_rgb(ref,[src,Future(time_s=1.)],**options())
    for k in clean:np.testing.assert_equal(clean[k],poisoned[k])

def test_constant_and_repeated_baseline_textures_do_not_become_observations():
    pose=POSE.copy();pose[0,3]=.2
    for grey in [np.full((65,65),128,np.uint8),np.tile((np.arange(65)*31%251)[:,None],(1,65)).astype(np.uint8)]:
        rgb=np.repeat(grey[:,:,None],3,2);out,_,_,_=sc.predict_rgb(frame(rgb),[frame(rgb,pose,'right')],**options())
        assert not out['confidence'][32,32]
