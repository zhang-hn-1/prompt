import unittest
from terminal import TerminalBrake
class TestTerminal(unittest.TestCase):
 def test_trigger_and_latch(self):
  b=TerminalBrake()
  self.assertIsNone(b.update(.8,.2,.4))
  self.assertEqual(b.update(.49,.2,.4),(0.,.4))
  self.assertEqual(b.update(.7,.3,-.4),(0.,.4))
 def test_no_time_or_direction_dependency(self):
  for angle in [-.4,.4]:
   b=TerminalBrake();self.assertEqual(b.update(.15,.05,angle),(0.,angle))
if __name__=='__main__':unittest.main()
