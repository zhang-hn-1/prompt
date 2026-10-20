import unittest
from condition import ApproachCondition
class Tests(unittest.TestCase):
 def test_coast_once(self):
  x=ApproachCondition('coast');self.assertIsNone(x.update(.8,.2,.4))
  for _ in range(5):self.assertEqual(x.update(.6,.2,.4),(0.,.4))
  self.assertIsNone(x.update(.6,.2,.4));self.assertIsNone(x.update(.4,.1,.4))
 def test_recovery_rebound(self):
  x=ApproachCondition('recovery');x.update(.4,.2,-.4)
  for _ in range(4):self.assertIsNotNone(x.update(.3,.01,-.4))
  x.update(.3,.1,-.4)
  for _ in range(4):self.assertIsNotNone(x.update(.3,.01,-.4))
  self.assertIsNone(x.update(.3,.01,-.4));self.assertTrue(x.done)
if __name__=='__main__':unittest.main()
