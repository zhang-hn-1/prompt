import unittest
import numpy as np
from structural_hypothesis_v1 import hypothesis_from_rotated

class TestStructuralHypothesis(unittest.TestCase):
    def test_inverse_rotation_keeps_original_corner(self):
        labels=np.zeros((3,2),int);labels[2,0]=7;prob=np.ones((3,2));r=hypothesis_from_rotated(labels,prob,np.zeros((2,3),bool),[7])
        self.assertEqual(np.argwhere(r['structural_hypothesis']).tolist(),[[0,0]])
    def test_mask_and_confidence_cannot_create_measured_geometry(self):
        label=np.full((3,2),7);prob=np.ones((3,2));prob[2,0]=.89;mask=np.zeros((2,3),bool);mask[1,2]=True;r=hypothesis_from_rotated(label,prob,mask,[7])
        self.assertFalse(r['structural_hypothesis'][0,0]);self.assertFalse(r['structural_hypothesis'][1,2]);self.assertFalse(r['observed_depth']);self.assertFalse(r['certified_free'])
    def test_other_class_not_promoted(self):
        r=hypothesis_from_rotated(np.zeros((3,2),int),np.ones((3,2)),np.zeros((2,3),bool),[7]);self.assertFalse(r['structural_hypothesis'].any())
    def test_bad_shapes_and_nonfinite_confidence_rejected(self):
        with self.assertRaises(ValueError):hypothesis_from_rotated(np.zeros((3,2)),np.ones((3,2)),np.zeros((3,2),bool),[7])
        with self.assertRaises(ValueError):hypothesis_from_rotated(np.zeros((3,2)),np.full((3,2),np.nan),np.zeros((2,3),bool),[7])

if __name__=='__main__':unittest.main()
