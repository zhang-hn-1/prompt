import unittest
from terminal import TerminalFeedback
class TestFeedback(unittest.TestCase):
 def test_far_unchanged(self):
  self.assertIsNone(TerminalFeedback().update(.51,.2,-.4))
 def test_resume_after_short_stop(self):
  t=TerminalFeedback();self.assertEqual(t.update(.3,.3,-.4),(0.,-.4))
  self.assertGreater(t.update(.3,0,.4)[0],0)
  self.assertEqual(t.update(.3,0,.4)[1],-.4)
 def test_inside_brakes_and_exit_resumes(self):
  t=TerminalFeedback();self.assertEqual(t.update(.18,.1,.4),(0.,.4))
  self.assertGreater(t.update(.23,0,.4)[0],0)
 def test_speed_damping_and_bounds(self):
  a=TerminalFeedback().update(.4,.1,.3)[0];b=TerminalFeedback().update(.4,.2,.3)[0]
  self.assertGreater(a,b);self.assertLessEqual(a,.5)
if __name__=='__main__':unittest.main()
