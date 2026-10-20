import numpy as np
from runtime_curvature_net_arc_v1 import net_course_arc

def test_signed_circle():
 t=np.linspace(0,1,101)
 for sign in (-1,1):
  p=np.c_[np.cos(sign*t),np.sin(sign*t)]
  r=net_course_arc(p,sign*t,np.ones(len(t)))
  assert abs(r['net_curvature_1_m']-sign)<1e-4
  assert r['segments']==1 and r['covered_samples']==101

def test_invalid_gap_not_bridged():
 p=np.c_[np.arange(6.),np.zeros(6)]
 r=net_course_arc(p,np.array([0.,.1,100.,200.,.3,.4]),np.array([1.,1.,0.,0.,1.,1.]))
 assert r['segments']==2 and r['covered_arc_m']==2
 assert abs(r['course_change_rad']-.2)<1e-10
 assert r['excluded_arc_m']==3
