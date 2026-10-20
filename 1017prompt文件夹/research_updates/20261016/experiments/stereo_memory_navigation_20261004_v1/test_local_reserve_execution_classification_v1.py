import pytest
from local_reserve_execution_classification_v1 import classify

def data(n=3,hit=False):
 states=[dict(time_s=i*.1) for i in range(n+1)]
 contacts=[dict(simulation_time_s=i*.02,available=True,contact=hit and i==5*n) for i in range(5*n+1)]
 contacts += [dict(simulation_time_s=i*.1,available=True,contact=False) for i in range(1,n+1)]
 return states,contacts,[i*.02 for i in range(5*n+1)]

def test_early_measured_collision_is_rejected_not_runtime():
 s,c,p=data(hit=True)
 r=classify(dict(status='finished',child_returncode=0),True,dict(complete=False,actions=3,captures=4),s,c,p,3)
 assert r['classification']=='measured_collision_reject' and not r['complete_35']
 assert r['scored_end_s']==pytest.approx(.3)

def test_interrupt_remains_unassessed_even_contact():
 s,c,p=data(hit=True)
 r=classify(dict(status='terminated',child_returncode=-15),True,{},s,c,p,3)
 assert r['classification']=='unassessed_execution'

def test_missing_seal_and_contact_hole_cannot_pass():
 s,c,p=data()
 for seal,rows in [(False,c),(True,c[:-1])]:
  r=classify(dict(status='finished',child_returncode=0),seal,dict(complete=False,actions=3,captures=4),s,rows,p,3)
  assert r['classification']=='unassessed_evidence'

def test_full_complete_and_early_noncollision_are_distinct():
 for n,expected in [(350,'complete_35'),(3,'unassessed_evidence')]:
  s,c,p=data(n)
  r=classify(dict(status='finished',child_returncode=0),True,dict(complete=n==350,actions=n,captures=n+1),s,c,p,n)
  assert r['classification']==expected


def test_short_collision_prefix_never_reads_nonexistent_future_files(tmp_path):
 from local_reserve_execution_classification_v1 import shared_execution_prefix
 a=tmp_path/'a';b=tmp_path/'b';a.mkdir();b.mkdir()
 # Baseline collided before the candidate intervention; no execution files exist there.
 assert shared_execution_prefix(a,b,4,3,10) is False
 assert shared_execution_prefix(a,b,3,3,10) is False
