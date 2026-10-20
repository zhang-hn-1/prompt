import numpy as np
from geometry import relocate

def test_rotated_translated_template():
    xy=np.array([[5.,2.],[6.,2.],[6.,3.]])
    actual=relocate(xy,0.,np.array([10.,20.]),np.pi/2)
    assert np.allclose(actual,[[10,20],[10,21],[9,21]])

def test_input_not_mutated():
    xy=np.array([[1.,2.],[3.,4.]]);before=xy.copy()
    relocate(xy,.3,np.zeros(2),-.6)
    assert np.array_equal(xy,before)
