import numpy as np
from local_binocular_refinement import refine

def images(d):
 y,x=np.mgrid[:80,:80]
 def texture(y,x):return np.clip(128+45*np.sin(.8*y+.3*x)+30*np.cos(1.3*y-.2*x),0,255).astype(np.uint8)
 l=texture(y,x);r=texture(y+d,x)
 return np.repeat(l[:,:,None],3,axis=2),np.repeat(r[:,:,None],3,axis=2)

def call(l,r,d=7.):
 shape=l.shape[:2];return refine(l,r,np.zeros(shape,bool),np.zeros(shape,bool),np.ones(shape,bool),np.array([[40.,40.]]),np.array([d]))

def test_known_vertical_disparity_and_fractional_sampling():
 for d in [9.,9.5]:
  o=call(*images(d),d=7.)
  assert o['keep'][0]
  assert abs(o['disparity_px'][0]-d)<=.5

def test_ambiguous_gradient_abstains():
 y,x=np.mgrid[:80,:80];a=np.repeat((y*3).astype(np.uint8)[:,:,None],3,axis=2)
 assert not call(a,a)['keep'][0]

def test_current_body_mask_and_image_border_abstain():
 l,r=images(9);m=np.zeros((80,80),bool);m[40,40]=True
 o=refine(l,r,m,np.zeros_like(m),np.ones_like(m),np.array([[40,40],[1,1]]),np.array([7.,7.]))
 assert not o['keep'].any()

def test_missing_valid_competitor_is_insufficient_uniqueness():
 l,r=images(9);m=np.ones((80,80),bool);m[28:35,37:44]=False
 o=refine(l,r,np.zeros_like(m),m,np.ones_like(m),np.array([[40,40]]),np.array([7.]))
 assert not o['keep'][0]
