from pacing_reference_metrics_v1 import full_contact_evidence

def test_absence_and_partial_coverage_fail_not_vacuous_all():
 assert not full_contact_evidence([],350,35.)
 assert not full_contact_evidence([dict(available=True,contact=False,simulation_time_s=0)],350,35.)

def test_full_available_timeline_and_contact_rules():
 rows=[dict(available=True,contact=False,simulation_time_s=i*.02) for i in range(1751)]
 rows+= [dict(available=True,contact=False,simulation_time_s=i*.1) for i in range(1,351)]
 assert full_contact_evidence(rows,350,35.)
 rows[100]['available']=False;assert not full_contact_evidence(rows,350,35.)
 rows[100]['available']=True;rows[100]['contact']=True;assert not full_contact_evidence(rows,350,35.)
