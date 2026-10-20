import unittest,numpy as np
from test_paired_evidence_memory_v1 import packet
from paired_prior_memory_v1 import PairedPriorMemory
class Tests(unittest.TestCase):
 def test_annotation_preserves_points_roles_and_provenance(self):
  a=packet();a['world'][0,2]=-.2;b=packet(role='UNKNOWN_unsupported');m=PairedPriorMemory();m.update(a,b,0.);s=m.at(0.)
  x=s['provisional']['records'][0];self.assertEqual(x['world'][0,2],-.2);self.assertTrue(x['prior_inconsistent'][0]);self.assertEqual(x['input_sha256'],a['input_sha256']);self.assertEqual(s['unknown']['records'][0]['evidence_role'],'UNKNOWN_unsupported');self.assertFalse(s['certified_free']);self.assertFalse(s['source_admitted'])
 def test_duplicate_and_expiry_preserved(self):
  m=PairedPriorMemory();m.update(packet(),packet(role='UNKNOWN_unsupported'),0.)
  with self.assertRaises(ValueError):m.update(packet(),packet(role='UNKNOWN_unsupported'),1.)
  self.assertFalse(m.at(10.1)['provisional']['records'])
 def test_second_bad_source_does_not_commit_annotations(self):
  m=PairedPriorMemory();b=packet(role='UNKNOWN_unsupported');b['input_sha256']={}
  with self.assertRaises(ValueError):m.update(packet(),b,0.)
  self.assertFalse(m.at(0.)['provisional']['records'])
 def test_snapshot_input_and_plane_are_independent(self):
  plane=np.array([0.,0,1,0]);m=PairedPriorMemory(plane);plane[3]=10;a=packet();m.update(a,packet(role='UNKNOWN_unsupported'),0.);a['world'][0,2]=8;s=m.at(0.);s['provisional']['records'][0]['signed_plane_distance_m'][0]=9;self.assertAlmostEqual(m.at(0.)['provisional']['records'][0]['signed_plane_distance_m'][0],.2)
 def test_future_before_geometry(self):
  class Bomb(dict):
   def __getitem__(self,k):
    if k=='world':raise AssertionError('future geometry read')
    return super().__getitem__(k)
  with self.assertRaises(ValueError):PairedPriorMemory().update(Bomb(packet(1.,10)),packet(1.,10,'UNKNOWN_unsupported'),0.)
if __name__=='__main__':unittest.main()
