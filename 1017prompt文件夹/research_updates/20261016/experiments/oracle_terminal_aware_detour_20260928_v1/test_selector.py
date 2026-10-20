import unittest, numpy as np
from pathlib import Path
from selector import select_sequence
import json
class SelectionTests(unittest.TestCase):
 def test_geometry_and_horizon(self):
  h=Path(__file__).resolve().parent;m=json.loads((h/'manifest.json').read_text())
  for kind in ['LR','RL']:
   hold=np.load(h.parent/'oracle_local_detour_20260928_v1'/f'{kind}_hold/physics.npz');turn=np.load(h.parent/'oracle_measured_turn_primitives_20260928_v1'/f'{kind}_measure/physics.npz')
   start=hold['position_world_xy'][800];yaw=hold['yaw_world_rad'][800];goal=np.array(m[kind]['goal']);obstacle=np.array(m[kind]['obstacle']['center_xy'])
   chosen,rows=select_sequence(hold,turn,start,yaw,goal,obstacle,19)
   self.assertEqual(len(rows),9);self.assertEqual(chosen['hold_s'],1.5);self.assertGreater(chosen['clearance_m'],0)
   self.assertIsNone(select_sequence(hold,turn,start,yaw,goal,obstacle,0)[0])
   angle=.7;R=np.array([[np.cos(angle),-np.sin(angle)],[np.sin(angle),np.cos(angle)]]);offset=np.array([3.,-2.])
   other,_=select_sequence(hold,turn,R@start+offset,yaw+angle,R@goal+offset,R@obstacle+offset,19)
   self.assertEqual(other['hold_s'],chosen['hold_s']);np.testing.assert_allclose(other['xy'],chosen['xy']@R.T+offset,atol=1e-12)
if __name__=='__main__':unittest.main()
