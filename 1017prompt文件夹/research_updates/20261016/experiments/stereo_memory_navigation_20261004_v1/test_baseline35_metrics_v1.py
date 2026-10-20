from baseline35_metrics_v1 import assess
import pytest

def s(t,x):return dict(time_s=t,position=[x,0.,0.])
def contact(t,on=False,available=True):return dict(time_s=t,available=available,collision=on)
def test_35second_prefix_does_not_credit_later_arrival():
 r=assess([s(0,0),s(35,1),s(40,2)],[contact(0),contact(35)], [0,0],[2,0]);assert r['outcome']=='timeout' and r['goal_remaining_m']==1 and r['scored_end_s']==35

def test_contact_failure_precedes_later_goal():
 r=assess([s(0,0),s(10,2)],[contact(0),contact(5,True),contact(10)], [0,0],[2,0]);assert r['outcome']=='collision' and not r['success']
def test_early_handoff_stop_is_failure_not_timeout_or_success():
 r=assess([s(0,0),s(20,1)],[contact(0),contact(20)], [0,0],[2,0]);assert r['outcome']=='early_stop' and not r['success']
def test_no_available_contacts_cannot_claim_contact_free_success():
 r=assess([s(0,0),s(10,2)],[contact(0,available=False),contact(10,available=False)], [0,0],[2,0]);assert not r['success'] and not r['contacts_all_available']

def test_empty_source_diagnostic_reports_missing_observation_not_free():
 from baseline35_metrics_v1 import observation_gap_diagnostics
 r=observation_gap_diagnostics([dict(time_s=0,points=0),dict(time_s=.5,points=0),dict(time_s=1,points=3)],[])
 assert r['empty_source_epochs']==2 and r['maximum_sampled_empty_span_s']==.5 and not r['absence_is_observed_free'] and not r['full_3d_blindzone_measured']
