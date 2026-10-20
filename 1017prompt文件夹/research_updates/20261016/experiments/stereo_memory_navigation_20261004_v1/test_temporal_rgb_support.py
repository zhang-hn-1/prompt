import numpy as np
from temporal_rgb_support import supports

def sample(frame, time, x=0., positive=True):
 t=np.eye(4);t[0,3]=x
 return dict(frame=frame,time_s=time,optical_world=t,positive=np.full((480,640),positive),self_mask=np.zeros((480,640),bool))

def test_requires_two_separated_past_views():
 points=np.array([[0.,0.,2.]])
 assert supports(points,10,1.,[sample(0,0),sample(5,.5,.11)])['keep'].tolist()==[True]
 assert supports(points,10,1.,[sample(0,0),sample(5,.5,.09)])['keep'].tolist()==[False]

def test_future_payload_is_never_read():
 points=np.array([[0.,0.,2.]])
 past=[sample(0,0),sample(5,.5,.11)]
 a=supports(points,10,1.,past)
 b=supports(points,10,1.,past+[dict(frame=11,time_s=1.1,optical_world='poison')])
 np.testing.assert_array_equal(a['keep'],b['keep'])
 np.testing.assert_array_equal(a['support_frames'][0],b['support_frames'][0])

def test_occlusion_and_missing_evidence_are_unverified():
 points=np.array([[0.,0.,2.],[100.,0.,2.]])
 a=sample(0,0);b=sample(5,.5,.11);b['self_mask'][:]=True
 assert not supports(points,10,1.,[a,b])['keep'].any()
