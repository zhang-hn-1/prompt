import unittest,copy
import numpy as np
from paired_evidence_memory_v1 import PairedEvidenceMemory

def packet(t=0.,f=0,role='image_supported_provisional'):
 return dict(time_s=t,frame=f,world=np.array([[1.,0.,.2]]),query_pixel_id=np.array([1]),partner_pixel_id=np.array([2]),input_sha256={'source':'a'*64},evidence_role=role,certified_free=False,source_model_admitted=False,measured_geometry_admitted=False)
class Tests(unittest.TestCase):
 def test_roles_remain_distinct_without_geometry_claim(self):
  m=PairedEvidenceMemory();m.update(packet(),packet(role='UNKNOWN_unsupported'),0.);s=m.at(0.)
  self.assertEqual(len(s['provisional']['records']),1);self.assertEqual(len(s['unknown']['records']),1);self.assertFalse(s['source_admitted']);self.assertFalse(s['certified_free']);self.assertEqual(s['unknown']['records'][0]['evidence_role'],'UNKNOWN_unsupported')
 def test_future_rejected_before_payload(self):
  class Bomb(dict):
   def __getitem__(self,k):
    if k=='world':raise AssertionError('future payload read')
    return super().__getitem__(k)
  with self.assertRaises(ValueError):PairedEvidenceMemory().update(Bomb(packet(1.,10)),packet(1.,10,'UNKNOWN_unsupported'),0.)
 def test_invalid_second_packet_does_not_commit_first(self):
  m=PairedEvidenceMemory();b=packet(role='UNKNOWN_unsupported');b['input_sha256']={}
  with self.assertRaises(ValueError):m.update(packet(),b,0.)
  self.assertEqual(m.at(0.)['provisional']['current_count'],0)
 def test_duplicate_does_not_refresh_birth_and_expiry(self):
  m=PairedEvidenceMemory();m.update(packet(),packet(role='UNKNOWN_unsupported'),0.)
  with self.assertRaises(ValueError):m.update(packet(),packet(role='UNKNOWN_unsupported'),1.)
  self.assertEqual(m.at(10.1)['provisional']['historical_count'],0)
 def test_input_snapshot_copy_and_claim_rejection(self):
  m=PairedEvidenceMemory();p=packet();m.update(p,packet(role='UNKNOWN_unsupported'),0.);p['world'][0,0]=99
  s=m.at(0.);s['provisional']['records'][0]['world'][0,0]=80;self.assertEqual(m.at(0.)['provisional']['records'][0]['world'][0,0],1)
  p=packet(1.,10);p['measured_geometry_admitted']=True
  with self.assertRaises(ValueError):m.update(p,packet(1.,10,'UNKNOWN_unsupported'),1.)
if __name__=='__main__':unittest.main()
