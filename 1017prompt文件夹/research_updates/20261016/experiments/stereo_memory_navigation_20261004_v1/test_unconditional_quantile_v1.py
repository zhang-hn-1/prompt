import unittest
from unconditional_quantile_v1 import quantile
class QuantileTests(unittest.TestCase):
 def test_small_infinite_tail_does_not_erase_finite_p90(self):self.assertEqual(quantile([0.]*57+[float('inf')]*2,.9),0.)
 def test_tail_covering_p90_retained(self):self.assertEqual(quantile([0.]*8+[float('inf')]*2,.9),float('inf'))
 def test_linear_finite_bracket(self):self.assertEqual(quantile([0.,10.],.5),5.)
if __name__=='__main__':unittest.main()
