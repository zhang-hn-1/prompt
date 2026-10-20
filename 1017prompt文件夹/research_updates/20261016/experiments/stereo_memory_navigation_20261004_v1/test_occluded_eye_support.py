import numpy as np
from occluded_eye_support import combine_support

def test_preserves_original_cross_eye_past_support():
 p=np.array([[[1,0],[0,1]]],bool);u=np.zeros_like(p)
 assert combine_support(p,u)[0]

def test_repeated_right_positive_with_unobservable_left():
 p=np.array([[[0,1],[0,1]]],bool);u=np.array([[[1,0],[1,0]]],bool)
 assert combine_support(p,u)[0]

def test_usable_class_negative_in_other_eye_blocks_extension():
 p=np.array([[[0,1],[0,1]]],bool);u=np.array([[[1,0],[0,0]]],bool)
 assert not combine_support(p,u)[0]

def test_no_positive_or_one_exposure_does_not_become_hypothesis():
 p=np.zeros((2,2,2),bool);p[1,0,1]=1;u=np.ones_like(p)
 assert not combine_support(p,u).any()
