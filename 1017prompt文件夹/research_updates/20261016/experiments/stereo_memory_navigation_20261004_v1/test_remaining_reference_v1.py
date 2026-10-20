import unittest
import numpy as np
from remaining_reference_v1 import project_remaining
class ProjectionTests(unittest.TestCase):
 def test_straight_clip(self):
  r=project_remaining([[0,0],[2,0],[3,0]],[1,.2]);self.assertAlmostEqual(r['progress_m'],1);np.testing.assert_allclose(r['remaining_xy'],[[1,0],[2,0],[3,0]])
 def test_cusp_reports_distinct_progress_tie(self):
  r=project_remaining([[0,0],[2,0],[0,0]],[1,0]);self.assertTrue(r['ambiguous_progress']);self.assertEqual(r['tied_progress_m'],[1.,3.]);self.assertEqual(r['progress_m'],1.)
 def test_shared_vertex_is_not_progress_ambiguity(self):
  r=project_remaining([[0,0],[1,0],[1,1]],[1,0]);self.assertFalse(r['ambiguous_progress']);self.assertEqual(r['tied_progress_m'],[1.])
 def test_singleton_and_degenerate(self):
  r=project_remaining([[1,1],[1,1]],[0,0]);self.assertEqual(r['progress_m'],0.);self.assertFalse(r['ambiguous_progress'])
 def test_reject_nonfinite(self):
  with self.assertRaises(ValueError):project_remaining([[0,0],[1,0]],[np.nan,0])
if __name__=='__main__':unittest.main()
