import unittest
import numpy as np
from audit_actual import validate_execution
class ExecutionGate(unittest.TestCase):
 def args(self):
  q=np.arange(20,dtype=float).reshape(2,10)
  p=dict(raw_command_target=np.vstack([np.zeros((1,2)),q.reshape(-1,2)]),action_block_index=np.r_[0,np.repeat(np.arange(2),5)],physics_step_in_block=np.r_[0,np.tile(np.arange(1,6),2)])
  j=dict(completed=[dict(frame=i,request=q[i].tolist(),time_s=i*.1) for i in range(2)],pending=[],incomplete_tail_ignored=False)
  return q,p,j
 def test_complete(self):self.assertEqual(validate_execution(*self.args()),10)
 def test_wrong_physical_target_rejected(self):
  q,p,j=self.args();p['raw_command_target'][3,1]+=1
  with self.assertRaises(AssertionError):validate_execution(q,p,j)
 def test_pending_rejected(self):
  q,p,j=self.args();j['pending']=[2]
  with self.assertRaises(AssertionError):validate_execution(q,p,j)
 def test_changed_journal_rejected(self):
  q,p,j=self.args();j['completed'][1]['request'][0]+=1
  with self.assertRaises(AssertionError):validate_execution(q,p,j)
if __name__=='__main__':unittest.main()
